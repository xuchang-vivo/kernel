// Copyright (c) 2026 vivo Mobile Communication Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! ESP32-C6 I2S0 register-level driver.
//!
//! The ESP32-C6 I2S peripheral has no APB-visible data FIFO; all sample data
//! must flow through GDMA. This driver pairs the I2S0 register block with two
//! GDMA channels (one TX, one RX) and exposes the `blueos_hal::i2s::I2s` trait.
//!
//! The clock tree is: XTAL (40 MHz) → PCR.I2S_TX_CLKM_DIV → I2S core clock.
//! For MCLK = 256×fs the divider is `40_000_000 / (256 * sample_rate)`. The
//! I2S block then derives BCK = MCLK / (2 * half_sample_bits).

use blueos_hal::i2s::{I2s, I2sChannelMode, I2sConfig, I2sFormat};
use blueos_hal::{Configuration, PlatPeri};
use core::cell::UnsafeCell;

use crate::dma::esp32c6_gdma::{
    out_int_regs, wait_for_bit, DmaDescriptor, Esp32c6GdmaChannel, OutInt, PERI_I2S0, DW0_OWNER_DMA,
    DW0_SUC_EOF,
};
use crate::static_ref::StaticRef;
use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite, WriteOnly},
};

/// I2S0 peripheral base address on ESP32-C6.
const I2S0_BASE: usize = 0x6000_C000;
/// PCR (Peripheral Clock Reset) base address on ESP32-C6.
const PCR_BASE: usize = 0x6009_6000;
/// XTAL frequency on ESP32-C6 (40 MHz).
const XTAL_HZ: u32 = 40_000_000;

/// Greatest common divisor (Euclidean algorithm).
const fn gcd(a: u32, b: u32) -> u32 {
    let mut a = a;
    let mut b = b;
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

/// Polling iteration cap for DMA wait.
const POLL_LIMIT: u32 = 10_000_000;

/// TX ring-buffer constants.
///
/// The TX path uses a ring of `RING_SIZE` descriptors, each pointing to a
/// fixed `SEG`-byte buffer. The descriptor `next` pointers form a closed
/// loop so the DMA engine never stops between segments — eliminating the
/// audio gap (FIFO underflow) that occurs when DMA halts at chain end and
/// must be restarted by the CPU.
///
/// `RING_SIZE` must be large enough that the CPU can fill segments ahead
/// of DMA consumption without being blocked. With 16 descriptors × 4080
/// bytes = ~65 KB ≈ 250 ms at 16 kHz, the CPU has a generous window to
/// refill consumed segments while DMA is busy with later ones.
const SEG: usize = 4080;
const RING_SIZE: usize = 16;

register_bitfields! [
    u32,

    /// I2S interrupt bits — shared by INT_RAW / INT_ST / INT_ENA / INT_CLR.
    pub I2sInt [
        RX_DONE OFFSET(0) NUMBITS(1) [],
        TX_DONE OFFSET(1) NUMBITS(1) [],
        RX_HUNG  OFFSET(2) NUMBITS(1) [],
        TX_HUNG  OFFSET(3) NUMBITS(1) [],
    ],

    /// I2S TX configuration register.
    pub TxConf [
        TX_RESET       OFFSET(0)  NUMBITS(1) [],
        TX_FIFO_RESET  OFFSET(1)  NUMBITS(1) [],
        TX_START       OFFSET(2)  NUMBITS(1) [],
        TX_SLAVE_MOD   OFFSET(3)  NUMBITS(1) [],
        TX_MONO        OFFSET(5)  NUMBITS(1) [],
        TX_CHAN_EQUAL  OFFSET(6)  NUMBITS(1) [],
        TX_UPDATE       OFFSET(8)  NUMBITS(1) [],
        TX_MONO_FST_VLD OFFSET(9) NUMBITS(1) [],
        TX_PCM_BYPASS  OFFSET(12) NUMBITS(1) [],
        TX_STOP_EN     OFFSET(13) NUMBITS(1) [],
        TX_TDM_EN      OFFSET(19) NUMBITS(1) [],
        TX_PDM_EN      OFFSET(20) NUMBITS(1) [],
        TX_CHAN_MOD    OFFSET(24) NUMBITS(3) [],
        SIG_LOOPBACK   OFFSET(27) NUMBITS(1) [],
    ],

    /// I2S RX configuration register.
    pub RxConf [
        RX_RESET        OFFSET(0)  NUMBITS(1) [],
        RX_FIFO_RESET   OFFSET(1)  NUMBITS(1) [],
        RX_START        OFFSET(2)  NUMBITS(1) [],
        RX_SLAVE_MOD    OFFSET(3)  NUMBITS(1) [],
        RX_MONO         OFFSET(5)  NUMBITS(1) [],
        RX_UPDATE        OFFSET(8)  NUMBITS(1) [],
        RX_MONO_FST_VLD OFFSET(9)  NUMBITS(1) [],
        RX_PCM_BYPASS   OFFSET(12) NUMBITS(1) [],
        RX_STOP_MODE    OFFSET(13) NUMBITS(2) [],
        RX_TDM_EN       OFFSET(19) NUMBITS(1) [],
        RX_PDM_EN       OFFSET(20) NUMBITS(1) [],
    ],

    /// I2S TX_CONF1 / RX_CONF1 — bit-clock divider, data width, half-sample
    /// bits, TDM channel bits, and MSB-shift (Philips standard).
    pub Conf1 [
        TDM_WS_WIDTH      OFFSET(0)  NUMBITS(7)  [],
        BCK_DIV_NUM       OFFSET(7)  NUMBITS(6)  [],
        BITS_MOD          OFFSET(13) NUMBITS(5)  [],
        HALF_SAMPLE_BITS  OFFSET(18) NUMBITS(6)  [],
        TDM_CHAN_BITS     OFFSET(24) NUMBITS(5)  [],
        MSB_SHIFT         OFFSET(29) NUMBITS(1)  [],
        BCK_NO_DLY        OFFSET(30) NUMBITS(1)  [],
    ],

    /// I2S TX/RX TDM control register.
    pub TdmCtrl [
        TDM_CHAN_EN    OFFSET(0)  NUMBITS(16) [],
        TDM_TOT_CHAN_NUM OFFSET(16) NUMBITS(4) [],
    ],

    /// I2S state register (read-only).
    pub I2sState [
        TX_IDLE OFFSET(0) NUMBITS(1) [],
    ],

    /// PCR.I2S_CONF — APB clock enable and module reset.
    pub PcrI2sConf [
        I2S_CLK_EN OFFSET(0) NUMBITS(1) [],
        I2S_RST_EN OFFSET(1) NUMBITS(1) [],
    ],

    /// PCR.I2S_TX_CLKM_CONF / PCR.I2S_RX_CLKM_CONF — function clock source
    /// and divider. The RX variant has an extra MCLK_SEL bit at offset 23.
    pub PcrI2sClkmConf [
        I2S_CLKM_DIV_NUM OFFSET(12) NUMBITS(8)  [],
        I2S_CLKM_SEL     OFFSET(20) NUMBITS(2)  [],
        I2S_CLKM_EN      OFFSET(22) NUMBITS(1)  [],
        I2S_MCLK_SEL     OFFSET(23) NUMBITS(1)  [],
    ],
];

register_structs! {
    /// I2S0 register block.
    pub I2sRegisters {        (0x00 => _reserved0),
        (0x0c => int_raw: ReadOnly<u32, I2sInt::Register>),
        (0x10 => int_st: ReadOnly<u32, I2sInt::Register>),
        (0x14 => int_ena: ReadWrite<u32, I2sInt::Register>),
        (0x18 => int_clr: WriteOnly<u32, I2sInt::Register>),
        (0x1c => _reserved1),
        (0x20 => rx_conf: ReadWrite<u32, RxConf::Register>),
        (0x24 => tx_conf: ReadWrite<u32, TxConf::Register>),
        (0x28 => rx_conf1: ReadWrite<u32, Conf1::Register>),
        (0x2c => tx_conf1: ReadWrite<u32, Conf1::Register>),
        (0x30 => _rx_clkm_conf),
        (0x34 => _tx_clkm_conf),
        (0x38 => _rx_clkm_div_conf),
        (0x3c => _tx_clkm_div_conf),
        (0x40 => _tx_pcm2pdm_conf),
        (0x44 => _tx_pcm2pdm_conf1),
        (0x48 => _reserved2),
        (0x50 => rx_tdm_ctrl: ReadWrite<u32, TdmCtrl::Register>),
        (0x54 => tx_tdm_ctrl: ReadWrite<u32, TdmCtrl::Register>),
        (0x58 => _rx_timing),
        (0x5c => _tx_timing),
        (0x60 => lc_hung_conf: ReadWrite<u32>),
        (0x64 => rxeof_num: ReadWrite<u32>),
        (0x68 => _conf_sigle_data),
        (0x6c => state: ReadOnly<u32, I2sState::Register>),
        (0x70 => _etm_conf),
        (0x74 => _reserved3),
        (0x80 => date: ReadWrite<u32>),
        //(0xEC => out_eof_bfr_des_addr: ReadOnly<u32, I2sInt::Register>),
        (0x84 => @END),
    }
}

register_structs! {
    /// PCR I2S register block (offsets 0x6c..0x80 within PCR).
    pub PcrI2sRegisters {
        (0x00 => _reserved0),
        (0x6c => i2s_conf: ReadWrite<u32, PcrI2sConf::Register>),
        (0x70 => i2s_tx_clkm_conf: ReadWrite<u32, PcrI2sClkmConf::Register>),
        (0x74 => i2s_tx_clkm_div_conf: ReadWrite<u32>),
        (0x78 => i2s_rx_clkm_conf: ReadWrite<u32, PcrI2sClkmConf::Register>),
        (0x7c => i2s_rx_clkm_div_conf: ReadWrite<u32>),
        (0x80 => @END),
    }
}

/// ESP32-C6 I2S0 driver with two GDMA channels.
///
/// `TX_CH` and `RX_CH` are GDMA channel indices (0, 1, or 2). They must be
/// distinct if both playback and capture are used simultaneously.
///
/// ## TX ring buffer
///
/// The TX path uses a ring of `RING_SIZE` DMA descriptors forming a closed
/// loop (`desc[i].next = &desc[i+1]`, `desc[RING_SIZE-1].next = &desc[0]`).
/// Each descriptor owns a fixed `SEG`-byte buffer. The DMA engine circulates
/// indefinitely, so there is **zero inter-segment gap** — critical for
/// glitch-free audio.
///
/// The CPU tracks a `write_idx` (next descriptor to fill). A descriptor is
/// safe to refill once the DMA engine has consumed it — detected by polling
/// the `owner` bit in `dw0[31]`: DMA clears it to 0 when done, CPU sets it
/// back to 1 after refilling.
///
/// On `drain_and_stop()`, the ring is broken (last descriptor's `next` set
/// to null) and the CPU waits for `OUT_TOTAL_EOF` before halting.
pub struct Esp32c6I2s0<const TX_CH: usize, const RX_CH: usize> {
    registers: StaticRef<I2sRegisters>,
    pcr: StaticRef<PcrI2sRegisters>,
    /// Ring of DMA descriptors forming a closed loop.
    tx_descs: UnsafeCell<[DmaDescriptor; RING_SIZE]>,
    /// Per-descriptor buffers (each `SEG` bytes).
    tx_bufs: UnsafeCell<[[u8; SEG]; RING_SIZE]>,
    /// CPU write cursor: next descriptor index to fill.
    write_idx: UnsafeCell<usize>,
    /// Whether the DMA ring has been started.
    tx_started: UnsafeCell<bool>,
    /// Index of the last descriptor filled (for drain).
    last_write_idx: UnsafeCell<Option<usize>>,
    rx_desc: UnsafeCell<DmaDescriptor>,
}

unsafe impl<const TX_CH: usize, const RX_CH: usize> Send for Esp32c6I2s0<TX_CH, RX_CH> {}
unsafe impl<const TX_CH: usize, const RX_CH: usize> Sync for Esp32c6I2s0<TX_CH, RX_CH> {}

impl<const TX_CH: usize, const RX_CH: usize> Esp32c6I2s0<TX_CH, RX_CH> {
    pub const fn new() -> Self {
        assert!(TX_CH < 3, "ESP32-C6 has only 3 GDMA channels");
        assert!(RX_CH < 3, "ESP32-C6 has only 3 GDMA channels");
        Self {
            registers: unsafe { StaticRef::new(I2S0_BASE as *const I2sRegisters) },
            pcr: unsafe { StaticRef::new(PCR_BASE as *const PcrI2sRegisters) },
            tx_descs: UnsafeCell::new({
                const NULL_DESC: DmaDescriptor = DmaDescriptor {
                    dw0: 0,
                    buffer: core::ptr::null_mut(),
                    next: core::ptr::null_mut(),
                };
                [NULL_DESC; RING_SIZE]
            }),
            tx_bufs: UnsafeCell::new([[0u8; SEG]; RING_SIZE]),
            write_idx: UnsafeCell::new(0),
            tx_started: UnsafeCell::new(false),
            last_write_idx: UnsafeCell::new(None),
            rx_desc: UnsafeCell::new(DmaDescriptor {
                dw0: 0,
                buffer: core::ptr::null_mut(),
                next: core::ptr::null_mut(),
            }),
        }
    }

    /// Configure MCLK and BCK dividers for the given sample rate / bits / channels.
    ///
    /// MCLK = 256 × fs (standard for ES8311). The PCR divider turns the 40 MHz
    /// XTAL into MCLK: `div = XTAL / MCLK`. The I2S BCK divider then produces
    /// BCK = MCLK / (2 × half_sample_bits), where `half_sample_bits = bits ×
    /// channels / 2`.
    fn configure_clock(&self, sample_rate: u32, bits: u8, slot_width: u8, channels: u8) -> blueos_hal::err::Result<()> {
        if sample_rate == 0 || bits == 0 || channels == 0 {
            return Err(blueos_hal::err::HalError::InvalidParam);
        }

        // MCLK = 256 × fs.
        let mclk = sample_rate * 256;
        if mclk > XTAL_HZ {
            return Err(blueos_hal::err::HalError::NotSupport);
        }
        let mclk_div = XTAL_HZ / mclk;
        if mclk_div == 0 || mclk_div > 255 {
            return Err(blueos_hal::err::HalError::NotSupport);
        }

        // half_sample_bits = total_slot * slot_width / 2.
        // ESP-IDF TDM mode uses total_slot (4) instead of active channels (2).
        // For 4 slots × 32-bit = 128, half = 64.
        let total_slot = 4u32;
        let half_sample_bits = total_slot * slot_width as u32 / 2;
        if half_sample_bits == 0 || half_sample_bits > 64 {
            return Err(blueos_hal::err::HalError::NotSupport);
        }

        // BCK = sample_rate * total_slot * slot_width.
        let bck_target = sample_rate * total_slot * slot_width as u32;
        let bck_div_num = mclk / bck_target;
        if bck_div_num == 0 || bck_div_num > 64 {
            return Err(blueos_hal::err::HalError::NotSupport);
        }
        let bck_div_field = bck_div_num.saturating_sub(1) & 0x3f;

        // TX clock: select XTAL (0), set divider, enable.
        self.pcr.i2s_tx_clkm_conf.modify(
            PcrI2sClkmConf::I2S_CLKM_DIV_NUM.val(mclk_div & 0xff)
                + PcrI2sClkmConf::I2S_CLKM_SEL.val(0) // XTAL
                + PcrI2sClkmConf::I2S_CLKM_EN.val(1),
        );

        // RX clock: same, plus MCLK_SEL = 1 (use TX clock for MCLK).
        self.pcr.i2s_rx_clkm_conf.modify(
            PcrI2sClkmConf::I2S_CLKM_DIV_NUM.val(mclk_div & 0xff)
                + PcrI2sClkmConf::I2S_CLKM_SEL.val(0) // XTAL
                + PcrI2sClkmConf::I2S_CLKM_EN.val(1)
                + PcrI2sClkmConf::I2S_MCLK_SEL.val(1),
        );

        // Configure the fractional clock divider (PCR.I2S_TX/RX_CLKM_DIV_CONF).
        //
        // The integer divider (I2S_CLKM_DIV_NUM) alone can only produce
        // f_xtal / N.  For 16 kHz × 256 = 4.096 MHz with a 40 MHz XTAL the
        // ratio is 625/64 = 9.765625, which is not an integer.  The PCR
        // fractional divider fills in the sub-integer part so the ES8311
        // receives an exact MCLK that matches a coeff_div[] table entry.
        //
        // Register layout (ESP32-C6 TRM):
        //   Z    [0:8]   (9 bits, 0-511)
        //   Y    [9:17]  (9 bits, 0-511)
        //   X    [18:26] (9 bits, 0-511)
        //   YN1  [27]    (1 bit)
        //   [28:31] reserved
        //
        // Given the fractional part b/a (already reduced to lowest terms),
        // the field values are:
        //   b <= a/2:  Z=b, Y=a%b, X=floor(a/b)-1, YN1=0
        //   b >  a/2:  Z=a-b, Y=a%(a-b), X=floor(a/(a-b))-1, YN1=1
        //
        // When the division is exact (no fractional part) the register is
        // programmed with x=0, y=0, z=0, yn1=1 (0x0800_0000).
        let div_conf = {
            let remainder = XTAL_HZ % mclk;
            if remainder == 0 {
                0x0800_0000
            } else {
                let a = mclk / gcd(mclk, remainder);
                let b = remainder / gcd(mclk, remainder);
                let (z, y, x, yn1) = if b <= a / 2 {
                    (b, a % b, a / b - 1, 0)
                } else {
                    (a - b, a % (a - b), a / (a - b) - 1, 1)
                };
                (z & 0x1FF) | ((y & 0x1FF) << 9) | ((x & 0x1FF) << 18) | (yn1 << 27)
            }
        };
        log::info!(
            "[I2S] MCLK div: integer={}, fractional=0x{:08x} (target {} Hz)",
            mclk_div,
            div_conf,
            mclk
        );
        self.pcr.i2s_tx_clkm_div_conf.set(div_conf);
        self.pcr.i2s_rx_clkm_div_conf.set(div_conf);

        // Program BCK divider and bit width in CONF1.
        // Match ESP-IDF TDM Philips: 4 slots × 32-bit, data_bit_width=32.
        // bits_mod = data_bit_width - 1 = 31
        // half_sample_bits = total_slot * slot_width / 2 = 4 * 32 / 2 = 64 → 63
        // tdm_ws_width = half_sample_bits - 1 = 63 (AUTO = total_slot*slot_bits/2)
        // tdm_chan_bits = slot_width - 1 = 31
        let bits_field = (slot_width as u32 - 1) & 0x1f;
        let total_slot = 4u32;
        let half_sample_bits = total_slot * slot_width as u32 / 2;
        let hsb_field = (half_sample_bits - 1) & 0x3f;
        let tdm_ws_width = (half_sample_bits - 1) & 0x7f;
        let tdm_chan_bits = (slot_width as u32 - 1) & 0x1f;

        // TX_CONF1: BCK_DIV_NUM | BITS_MOD | HALF_SAMPLE_BITS | MSB_SHIFT | BCK_NO_DLY.
        self.registers.tx_conf1.write(
            Conf1::TDM_WS_WIDTH.val(tdm_ws_width)
                + Conf1::BCK_DIV_NUM.val(bck_div_field)
                + Conf1::BITS_MOD.val(bits_field)
                + Conf1::HALF_SAMPLE_BITS.val(hsb_field)
                + Conf1::TDM_CHAN_BITS.val(tdm_chan_bits)
                + Conf1::MSB_SHIFT.val(1)
                + Conf1::BCK_NO_DLY.val(1),
        );

        // RX_CONF1: same layout.
        self.registers.rx_conf1.write(
            Conf1::TDM_WS_WIDTH.val(tdm_ws_width)
                + Conf1::BCK_DIV_NUM.val(bck_div_field)
                + Conf1::BITS_MOD.val(bits_field)
                + Conf1::HALF_SAMPLE_BITS.val(hsb_field)
                + Conf1::TDM_CHAN_BITS.val(tdm_chan_bits)
                + Conf1::MSB_SHIFT.val(1)
                + Conf1::BCK_NO_DLY.val(1),
        );

        Ok(())
    }

    /// Configure the TX (playback) path for the given format.
    ///
    /// Per TRM 30.9, this only writes the data-mode and channel-mode fields.
    /// The actual TX/RX unit and FIFO reset happens in `configure()` after
    /// `tx_update()`, not here.
    fn configure_tx(&self, cfg: &I2sConfig) {
        // TDM mode: TX_TDM_EN=1, TX_PDM_EN=0 (i2s_ll_tx_enable_tdm).
        // TX_PCM_BYPASS=1, TX_MONO_FST_VLD=1, TX_CHAN_MOD=0 (stereo).
        let mut conf = TxConf::TX_PCM_BYPASS::SET
            + TxConf::TX_TDM_EN::SET
            + TxConf::TX_MONO_FST_VLD::SET
            + TxConf::TX_CHAN_MOD.val(0);
        if matches!(cfg.channel_mode, I2sChannelMode::Mono) {
            conf += TxConf::TX_MONO::SET + TxConf::TX_CHAN_EQUAL::SET;
        }
        self.registers.tx_conf.write(conf);

        // TDM_CTRL: 4 slots total, slot mask 0xF (SLOT0-3).
        let (chan_en, tot_chan) = match cfg.channel_mode {
            I2sChannelMode::Stereo => (0xF, 4 - 1),
            I2sChannelMode::Mono => (0x1, 4 - 1),
        };
        self.registers.tx_tdm_ctrl.write(
            TdmCtrl::TDM_CHAN_EN.val(chan_en) + TdmCtrl::TDM_TOT_CHAN_NUM.val(tot_chan),
        );
    }

    /// Configure the RX (capture) path for the given format.
    ///
    /// Per TRM 30.9, this only writes the data-mode and channel-mode fields.
    /// The actual TX/RX unit and FIFO reset happens in `configure()` after
    /// `rx_update()`, not here.
    fn configure_rx(&self, cfg: &I2sConfig) {
        // TDM mode: RX_TDM_EN=1, RX_PDM_EN=0 (i2s_ll_rx_enable_tdm).
        // RX_PCM_BYPASS=1, RX_MONO_FST_VLD=1, RX_STOP_MODE=2.
        let mut conf = RxConf::RX_PCM_BYPASS::SET
            + RxConf::RX_TDM_EN::SET
            + RxConf::RX_MONO_FST_VLD::SET
            + RxConf::RX_STOP_MODE.val(2);
        if matches!(cfg.channel_mode, I2sChannelMode::Mono) {
            conf += RxConf::RX_MONO::SET;
        }
        self.registers.rx_conf.write(conf);

        // RXEOF_NUM: number of RX data words before in_suc_eof fires.
        self.registers.rxeof_num.set(0x40);

        // TDM_CTRL: 4 slots total, slot mask 0xF (SLOT0-3).
        let (chan_en, tot_chan) = match cfg.channel_mode {
            I2sChannelMode::Stereo => (0xF, 4 - 1),
            I2sChannelMode::Mono => (0x1, 4 - 1),
        };
        self.registers.rx_tdm_ctrl.write(
            TdmCtrl::TDM_CHAN_EN.val(chan_en) + TdmCtrl::TDM_TOT_CHAN_NUM.val(tot_chan),
        );
    }

    /// Apply TX register updates (clock-domain crossing).
    fn tx_update(&self) {
        self.registers.tx_conf.modify(TxConf::TX_UPDATE::SET);
        // TX_UPDATE is self-clearing; wait for it.
        for _ in 0..POLL_LIMIT {
            if self.registers.tx_conf.get() & TxConf::TX_UPDATE.mask == 0 {
                break;
            }
            core::hint::spin_loop();
        }
    }

    /// Apply RX register updates (clock-domain crossing).
    fn rx_update(&self) {
        self.registers.rx_conf.modify(RxConf::RX_UPDATE::SET);
        for _ in 0..POLL_LIMIT {
            if self.registers.rx_conf.get() & RxConf::RX_UPDATE.mask == 0 {
                break;
            }
            core::hint::spin_loop();
        }
    }

    /// Busy-poll until descriptor `idx` is safe to refill.
    ///
    /// With `OUT_AUTO_WRBACK=1`, the DMA engine clears `dw0[31]` (owner bit)
    /// to 0 after consuming a TX descriptor. We spin until that bit reads 0,
    /// meaning the DMA has finished with this descriptor and its buffer is
    /// safe to overwrite.
    ///
    /// Uses `read_volatile` on every iteration to bypass compiler caching.
    fn wait_desc_safe(&self, idx: usize) {
        let descs = unsafe { &*self.tx_descs.get() };
        let desc_ptr = core::ptr::addr_of!(descs[idx].dw0) as *const u32;
        let mut spin = 0u32;
        loop {
            let dw0 = unsafe { core::ptr::read_volatile(desc_ptr) };
            if dw0 & DW0_OWNER_DMA == 0 {
                return;
            }
            spin += 1;
            if spin % 1_000_000 == 0 {
                log::warn!(
                    "[I2S] wait_desc_safe({}) still pending after {} spins, dw0=0x{:08x}",
                    idx, spin, dw0,
                );
            }
            core::hint::spin_loop();
        }
    }

    /// Check if the DMA engine is paused and restart it if so.
    ///
    /// With `OUT_AUTO_WRBACK=1`, the DMA pauses when it encounters a
    /// descriptor whose owner bit is 0 (already consumed, not yet refilled).
    /// We read `OUTLINK_DSCR_ADDR` from `OUT_STATE` to find the DMA's current
    /// position, check that descriptor's owner bit, and if it's 0, issue
    /// `OUTLINK_RESTART` to resume the DMA from that descriptor.
    fn restart_if_paused(&self) {
        let descs = unsafe { &*self.tx_descs.get() };
        let base = (core::ptr::addr_of!(descs[0]) as usize) & 0x3_ffff;
        let desc_size = core::mem::size_of::<DmaDescriptor>();
        let dma_addr = Esp32c6GdmaChannel::<TX_CH>::read_outlink_dscr_addr() as usize;
        if dma_addr < base {
            return;
        }
        let dma_idx = (dma_addr - base) / desc_size;
        if dma_idx >= RING_SIZE {
            return;
        }
        let owner = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(descs[dma_idx].dw0) as *const u32) };
        if owner & DW0_OWNER_DMA == 0 {
            Esp32c6GdmaChannel::<TX_CH>::restart_tx();
        }
    }

    /// Simultaneous TX/RX transfer for loopback testing.
    ///
    /// Starts RX DMA first, then TX DMA, so no incoming samples are lost.
    /// Waits for both to complete. Used with GPIO-matrix loopback (DOUT and
    /// DIN routed to the same pin) to verify the I2S data path.
    pub fn loopback_transfer(
        &self,
        tx_buf: &[u8],
        rx_buf: &mut [u8],
    ) -> blueos_hal::err::Result<()> {
        if tx_buf.is_empty() || rx_buf.is_empty() {
            return Ok(());
        }

        log::info!(
            "[I2S_TEST] loopback_transfer: tx={}, rx={}",
            tx_buf.len(),
            rx_buf.len()
        );

        // Re-configure I2S to ensure clocks and DMA bindings are correct.
        log::info!("[I2S_TEST] re-configuring I2S...");
        self.enable();
        log::info!("[I2S] i2s_conf after enable: 0x{:08x}", self.pcr.i2s_conf.get());
        self.configure(&I2sConfig::default_16k())?;
        log::info!("[I2S_TEST] re-configure done");

        // Prepare TX descriptor: single descriptor, suc_eof = true.
        let tx_desc = unsafe { &mut (*self.tx_descs.get())[0] };
        *tx_desc = DmaDescriptor::for_tx(tx_buf.as_ptr() as *mut u8, tx_buf.len(), true);

        // Prepare RX descriptor.
        let rx_desc = unsafe { &mut *self.rx_desc.get() };
        *rx_desc = DmaDescriptor::for_rx(rx_buf.as_mut_ptr(), rx_buf.len());

        log::info!(
            "[I2S_TEST] tx_desc: dw0=0x{:08x}, buf={:p}",
            tx_desc.dw0,
            tx_desc.buffer
        );
        log::info!(
            "[I2S_TEST] rx_desc: dw0=0x{:08x}, buf={:p}",
            rx_desc.dw0,
            rx_desc.buffer
        );

        // Start RX first so it is ready to capture when TX begins.
        Esp32c6GdmaChannel::<RX_CH>::start_rx(rx_desc);
        self.registers.rx_conf.modify(RxConf::RX_START::SET);
        log::info!("[I2S_TEST] RX DMA started, RX_START set");

        // Start TX.
        Esp32c6GdmaChannel::<TX_CH>::start_tx(tx_desc);
        self.registers.tx_conf.modify(TxConf::TX_START::SET);
        log::info!("[I2S_TEST] TX DMA started, TX_START set");

        // Dump I2S and DMA register state for debugging.
        log::info!(
            "[I2S_TEST] tx_conf=0x{:08x}, rx_conf=0x{:08x}",
            self.registers.tx_conf.get(),
            self.registers.rx_conf.get()
        );
        log::info!(
            "[I2S_TEST] tx_conf1=0x{:08x}, rx_conf1=0x{:08x}",
            self.registers.tx_conf1.get(),
            self.registers.rx_conf1.get()
        );
        log::info!(
            "[I2S_TEST] rxeof_num=0x{:08x}",
            self.registers.rxeof_num.get()
        );
        Esp32c6GdmaChannel::<TX_CH>::dump_channel_state();
        Esp32c6GdmaChannel::<RX_CH>::dump_channel_state();

        // Wait for both to finish.
        let tx_result = Esp32c6GdmaChannel::<TX_CH>::wait_tx_done();
        log::info!("[I2S_TEST] wait_tx_done result: {:?}", tx_result);
        let rx_result = Esp32c6GdmaChannel::<RX_CH>::wait_rx_done();
        log::info!("[I2S_TEST] wait_rx_done result: {:?}", rx_result);

        // Stop TX/RX so the next transfer can restart cleanly.
        self.registers.tx_conf.modify(TxConf::TX_START::CLEAR);
        self.registers.rx_conf.modify(RxConf::RX_START::CLEAR);

        tx_result?;
        rx_result
    }
}

impl<const TX_CH: usize, const RX_CH: usize> PlatPeri for Esp32c6I2s0<TX_CH, RX_CH> {
    fn enable(&self) {
        // 1. Enable I2S APB clock + high-pulse reset.
        self.pcr.i2s_conf.set(0);
        self.pcr.i2s_conf.set(PcrI2sConf::I2S_CLK_EN.mask);
        self.pcr.i2s_conf.set(PcrI2sConf::I2S_CLK_EN.mask | PcrI2sConf::I2S_RST_EN.mask);

        // 2. Reset the DMA channels and bind them to I2S0.
        Esp32c6GdmaChannel::<TX_CH>::reset_tx();
        Esp32c6GdmaChannel::<TX_CH>::set_tx_peri(PERI_I2S0);
        Esp32c6GdmaChannel::<RX_CH>::reset_rx();
        Esp32c6GdmaChannel::<RX_CH>::set_rx_peri(PERI_I2S0);

        // 3. Reset ring-buffer bookkeeping.
        unsafe {
            *self.write_idx.get() = 0;
            *self.tx_started.get() = false;
            *self.last_write_idx.get() = None;
        }
    }
    fn disable(&self) {
        // Stop TX/RX.
        self.registers.tx_conf.modify(TxConf::TX_START::CLEAR);
        self.registers.rx_conf.modify(RxConf::RX_START::CLEAR);
        // Gate the function clocks.
        self.pcr.i2s_tx_clkm_conf.modify(PcrI2sClkmConf::I2S_CLKM_EN::CLEAR);
        self.pcr.i2s_rx_clkm_conf.modify(PcrI2sClkmConf::I2S_CLKM_EN::CLEAR);
    }
}

impl<const TX_CH: usize, const RX_CH: usize> Configuration<I2sConfig>
    for Esp32c6I2s0<TX_CH, RX_CH>
{
    type Target = ();

    fn configure(&self, cfg: &I2sConfig) -> blueos_hal::err::Result<Self::Target> {
        let channels: u8 = match cfg.channel_mode {
            I2sChannelMode::Stereo => 2,
            I2sChannelMode::Mono => 1,
        };

        // 1. Clock tree first (TRM 30.9 — clock must be configured before reset).
        self.configure_clock(cfg.sample_rate, cfg.bits_per_sample, cfg.slot_width, channels)?;

        // 2. TX/RX data mode + channel mode (TRM 30.9 step 4).
        self.configure_tx(cfg);
        self.configure_rx(cfg);

        // 3. Push register updates across the clock domain (TRM 30.9 step 4).
        self.tx_update();
        self.rx_update();

        // 4. Reset TX/RX units and FIFOs (TRM 30.7 / 30.9 step 5).
        //    Module clock must already be configured (done in step 1).
        self.registers.tx_conf
            .modify(TxConf::TX_RESET::SET + TxConf::TX_FIFO_RESET::SET);
        self.registers.tx_conf
            .modify(TxConf::TX_RESET::CLEAR + TxConf::TX_FIFO_RESET::CLEAR);
        self.registers.rx_conf
            .modify(RxConf::RX_RESET::SET + RxConf::RX_FIFO_RESET::SET);
        self.registers.rx_conf
            .modify(RxConf::RX_RESET::CLEAR + RxConf::RX_FIFO_RESET::CLEAR);

        // Clear any pending I2S interrupts and disable them (we poll).
        self.registers.int_clr.write(I2sInt::TX_DONE.val(1) + I2sInt::RX_DONE.val(1));
        self.registers.int_ena.set(0);

        // FIFO hung timeout — keep the default (0x0810) to avoid stalls.
        self.registers.lc_hung_conf.set(0x0810);

        // Ensure I2S module is not in reset state.
        self.pcr.i2s_conf
            .set(PcrI2sConf::I2S_CLK_EN.mask | PcrI2sConf::I2S_RST_EN.mask);
        log::info!("[I2S] i2s_conf after configure: 0x{:08x}", self.pcr.i2s_conf.get());

        Ok(())
    }
}

impl<const TX_CH: usize, const RX_CH: usize> I2s<I2sConfig, ()>
    for Esp32c6I2s0<TX_CH, RX_CH>
{
    fn write(&self, buf: &[u8]) -> blueos_hal::err::Result<()> {
        if buf.is_empty() {
            return Ok(());
        }

        let max_capacity = SEG * RING_SIZE;
        if buf.len() > max_capacity {
            return Err(blueos_hal::err::HalError::InvalidParam);
        }

        let total_len = buf.len();
        let num_segs = (total_len + SEG - 1) / SEG;
        let descs = unsafe { &mut *self.tx_descs.get() };
        let bufs = unsafe { &mut *self.tx_bufs.get() };

        let started = unsafe { *self.tx_started.get() };

        if !started {
            // First write: initialize the full ring.
            // Fill desc[0..num_segs] with real audio data, desc[num_segs..RING_SIZE]
            // with silence (zeros). All descriptors form a closed loop so the DMA
            // engine circulates indefinitely. With OUT_AUTO_WRBACK=1, DMA clears
            // the owner bit after consuming each descriptor, and pauses when it
            // reaches an owner=0 descriptor (until CPU refills and restarts).
            let mut widx = 0usize;
            for i in 0..num_segs {
                let off = i * SEG;
                let len = SEG.min(total_len - off);
                bufs[widx][..len].copy_from_slice(&buf[off..off + len]);
                let next_idx = (widx + 1) % RING_SIZE;
                descs[widx] = DmaDescriptor::for_tx(bufs[widx].as_mut_ptr(), len, true);
                descs[widx].next = core::ptr::addr_of_mut!(descs[next_idx]);
                widx = next_idx;
            }
            // Fill remaining slots with silence so DMA has valid data to play
            // while CPU prepares the next write.
            for i in num_segs..RING_SIZE {
                bufs[i].fill(0);
                let next_idx = (i + 1) % RING_SIZE;
                descs[i] = DmaDescriptor::for_tx(bufs[i].as_mut_ptr(), SEG, true);
                descs[i].next = core::ptr::addr_of_mut!(descs[next_idx]);
            }

            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

            unsafe {
                *self.write_idx.get() = 0;
                *self.last_write_idx.get() = Some((num_segs - 1) % RING_SIZE);
            }

            // Start the DMA ring.
            Esp32c6GdmaChannel::<TX_CH>::clear_out_eof();
            Esp32c6GdmaChannel::<TX_CH>::clear_out_total_eof();
            Esp32c6GdmaChannel::<TX_CH>::start_tx_no_reset(&descs[0]);
            self.registers.tx_conf.modify(TxConf::TX_START::SET);
            unsafe {
                *self.tx_started.get() = true;
            }
            return Ok(());
        }

        // Subsequent writes: refill consumed descriptors one by one.
        // With OUT_AUTO_WRBACK=1, DMA clears owner bit after consuming each
        // descriptor, and pauses when it hits an owner=0 descriptor.
        // We poll the owner bit, refill, set owner=1, then restart if paused.
        let mut widx = unsafe { *self.write_idx.get() };
        for i in 0..num_segs {
            let off = i * SEG;
            let len = SEG.min(total_len - off);

            // Wait until DMA has consumed this descriptor (owner bit cleared).
            self.wait_desc_safe(widx);

            // Refill the buffer.
            bufs[widx][..len].copy_from_slice(&buf[off..off + len]);

            // Rebuild the descriptor: set size, length, suc_eof, and owner=DMA.
            let size = (len as u32) & 0xfff;
            let length = (len as u32) << 12;
            descs[widx].dw0 = size | length | DW0_SUC_EOF | DW0_OWNER_DMA;
            // next pointer is already correct (closed loop).

            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

            widx = (widx + 1) % RING_SIZE;
        }

        // DMA may have paused on an owner=0 descriptor while we were refilling.
        // Check and restart if needed.
        self.restart_if_paused();

        unsafe {
            *self.write_idx.get() = widx;
            *self.last_write_idx.get() = Some((widx + RING_SIZE - 1) % RING_SIZE);
        }

        Ok(())
    }

    fn read(&self, buf: &mut [u8]) -> blueos_hal::err::Result<()> {
        if buf.is_empty() {
            return Ok(());
        }

        let desc = unsafe { &mut *self.rx_desc.get() };
        *desc = DmaDescriptor::for_rx(buf.as_mut_ptr(), buf.len());

        Esp32c6GdmaChannel::<RX_CH>::start_rx(desc);
        self.registers.rx_conf.modify(RxConf::RX_START::SET);

        let result = Esp32c6GdmaChannel::<RX_CH>::wait_rx_done();

        self.registers.rx_conf.modify(RxConf::RX_START::CLEAR);

        result
    }

    fn drain_and_stop(&self) -> blueos_hal::err::Result<()> {
        // Only drain if the ring was actually started.
        if !unsafe { *self.tx_started.get() } {
            return Ok(());
        }

        let last_idx = match unsafe { *self.last_write_idx.get() } {
            Some(idx) => idx,
            None => return Ok(()),
        };

        let descs = unsafe { &mut *self.tx_descs.get() };

        // Break the ring: the last-filled descriptor's `next` becomes null,
        // so the DMA engine will halt after consuming it (OUT_TOTAL_EOF).
        descs[last_idx].next = core::ptr::null_mut();
        descs[last_idx].dw0 |= DW0_SUC_EOF; // ensure suc_eof=1

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        // Busy-poll until DMA consumes the last descriptor and halts.
        // OUT_TOTAL_EOF fires when DMA finishes a descriptor whose next=null.
        loop {
            if Esp32c6GdmaChannel::<TX_CH>::is_out_total_eof() {
                break;
            }
            // Check for descriptor errors via the raw register.
            let int = out_int_regs::<TX_CH>();
            if !wait_for_bit(int, OutInt::OUT_TOTAL_EOF.mask | OutInt::OUT_DSCR_ERR.mask) {
                log::warn!("[I2S] drain: TX timeout");
                return Err(blueos_hal::err::HalError::Timeout);
            }
        }

        // Stop I2S TX.
        self.registers.tx_conf.modify(TxConf::TX_START::CLEAR);

        // Clear interrupt flags.
        Esp32c6GdmaChannel::<TX_CH>::clear_out_total_eof();
        Esp32c6GdmaChannel::<TX_CH>::clear_out_eof();

        // Restore the ring link for future playback.
        let next_idx = (last_idx + 1) % RING_SIZE;
        descs[last_idx].next = core::ptr::addr_of_mut!(descs[next_idx]);

        // Reset ring bookkeeping.
        unsafe {
            *self.write_idx.get() = 0;
            *self.tx_started.get() = false;
            *self.last_write_idx.get() = None;
        }

        Ok(())
    }
}
