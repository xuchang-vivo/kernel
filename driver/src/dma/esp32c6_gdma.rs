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

//! Minimal ESP32-C6 GDMA (General DMA) driver.
//!
//! This driver provides just enough functionality to feed the I2S peripheral
//! with data, since the ESP32-C6 I2S FIFO is only reachable via GDMA. It uses
//! single-descriptor transfers with busy-polling (no interrupt registration).
//!
//! The descriptor format follows the standard ESP32 family layout: a 12-byte
//! (3-word) linked-list node. See the ESP32-C6 Technical Reference Manual.

use core::ptr::addr_of;

use crate::static_ref::StaticRef;
use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite, WriteOnly},
};

/// Per-channel GDMA register snapshot captured at the point of failure.
#[derive(Copy, Clone)]
pub struct GdmaChStatus {
    pub in_raw: u32,
    pub out_raw: u32,
    pub in_conf0: u32,
    pub in_link: u32,
    pub in_state: u32,
    pub out_conf0: u32,
    pub out_link: u32,
    pub out_state: u32,
}

/// Diagnostic snapshot of the GDMA controller: one entry per channel (3 on
/// ESP32-C6) plus the global MISC_CONF register.
#[derive(Copy, Clone)]
pub struct GdmaStatus {
    pub channels: [GdmaChStatus; 3],
    pub misc_conf: u32,
}

impl core::fmt::Display for GdmaStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (ch, s) in self.channels.iter().enumerate() {
            write!(
                f,
                "CH{} IN_RAW=0x{:08x} OUT_RAW=0x{:08x} | ",
                ch, s.in_raw, s.out_raw
            )?;
        }
        for (ch, s) in self.channels.iter().enumerate() {
            write!(
                f,
                "CH{} IN[c0=0x{:08x} lk=0x{:08x} st=0x{:08x}] OUT[c0=0x{:08x} lk=0x{:08x} st=0x{:08x}] | ",
                ch,
                s.in_conf0,
                s.in_link,
                s.in_state,
                s.out_conf0,
                s.out_link,
                s.out_state
            )?;
        }
        write!(f, "MISC=0x{:08x}", self.misc_conf)
    }
}

/// Capture the interrupt RAW registers and channel state for all 3 GDMA
/// channels. Called on timeout/error and returned to the caller for logging.
pub fn capture_gdma_status() -> GdmaStatus {
    let mut channels = [GdmaChStatus {
        in_raw: 0,
        out_raw: 0,
        in_conf0: 0,
        in_link: 0,
        in_state: 0,
        out_conf0: 0,
        out_link: 0,
        out_state: 0,
    }; 3];

    for ch in 0..3usize {
        let in_raw = unsafe {
            core::ptr::read_volatile((DMA_BASE + IN_INT_BASE + ch * INT_STRIDE) as *const u32)
        };
        let out_raw = unsafe {
            core::ptr::read_volatile((DMA_BASE + OUT_INT_BASE + ch * INT_STRIDE) as *const u32)
        };
        let ch_base = DMA_BASE + CH_OFFSET + ch * CH_STRIDE;
        let in_conf0 = unsafe { core::ptr::read_volatile((ch_base + 0x00) as *const u32) };
        let in_link = unsafe { core::ptr::read_volatile((ch_base + 0x10) as *const u32) };
        let in_state = unsafe { core::ptr::read_volatile((ch_base + 0x14) as *const u32) };
        let out_conf0 = unsafe { core::ptr::read_volatile((ch_base + 0x60) as *const u32) };
        let out_link = unsafe { core::ptr::read_volatile((ch_base + 0x70) as *const u32) };
        let out_state = unsafe { core::ptr::read_volatile((ch_base + 0x74) as *const u32) };
        channels[ch] = GdmaChStatus {
            in_raw,
            out_raw,
            in_conf0,
            in_link,
            in_state,
            out_conf0,
            out_link,
            out_state,
        };
    }

    let misc_conf = unsafe { core::ptr::read_volatile((DMA_BASE + 0x64) as *const u32) };

    GdmaStatus { channels, misc_conf }
}

/// GDMA base address on ESP32-C6.
const DMA_BASE: usize = 0x6008_0000;

/// Per-channel register stride (CH[n] starts at `0x70 + n * 0x80`).
const CH_OFFSET: usize = 0x70;
const CH_STRIDE: usize = 0xC0;

/// IN_INT_CH[n] stride: each cluster is 0x10 (RAW, ST, ENA, CLR).
const IN_INT_BASE: usize = 0x00;
const OUT_INT_BASE: usize = 0x30;
const INT_STRIDE: usize = 0x10;

/// Peripheral IDs for `peri_sel` (ESP32-C6).
/// I2S0 = 3, SPI2 = 0, UHCI0 = 2, etc.
pub const PERI_I2S0: u32 = 3;

/// Polling iteration cap before declaring a DMA timeout.
const DMA_POLL_LIMIT: u32 = 10_000_000;

register_bitfields! [
    u32,

    /// RX (input) interrupt bits — shared by RAW / ST / ENA / CLR.
    pub InInt [
        IN_DONE       OFFSET(0) NUMBITS(1) [],
        IN_SUC_EOF    OFFSET(1) NUMBITS(1) [],
        IN_ERR_EOF    OFFSET(2) NUMBITS(1) [],
        IN_DSCR_ERR   OFFSET(3) NUMBITS(1) [],
        IN_DSCR_EMPTY OFFSET(4) NUMBITS(1) [],
        INFIFO_OVF    OFFSET(5) NUMBITS(1) [],
        INFIFO_UDF    OFFSET(6) NUMBITS(1) [],
    ],

    /// TX (output) interrupt bits — shared by RAW / ST / ENA / CLR.
    pub OutInt [
        OUT_DONE      OFFSET(0) NUMBITS(1) [],
        OUT_EOF       OFFSET(1) NUMBITS(1) [],
        OUT_DSCR_ERR  OFFSET(2) NUMBITS(1) [],
        OUT_TOTAL_EOF OFFSET(3) NUMBITS(1) [],
        OUTFIFO_OVF   OFFSET(4) NUMBITS(1) [],
        OUTFIFO_UDF   OFFSET(5) NUMBITS(1) [],
    ],

    /// RX channel configure 0.
    pub InConf0 [
        IN_RST           OFFSET(0) NUMBITS(1) [],
        INDSCR_BURST_EN  OFFSET(2) NUMBITS(1) [],
        IN_DATA_BURST_EN OFFSET(3) NUMBITS(1) [],
        MEM_TRANS_EN     OFFSET(4) NUMBITS(1) [],
    ],

    /// TX channel configure 0.
    pub OutConf0 [
        OUT_RST           OFFSET(0) NUMBITS(1) [],
        OUT_AUTO_WRBACK   OFFSET(2) NUMBITS(1) [],
        OUT_EOF_MODE      OFFSET(3) NUMBITS(1) [],
        OUTDSCR_BURST_EN  OFFSET(4) NUMBITS(1) [],
        OUT_DATA_BURST_EN OFFSET(5) NUMBITS(1) [],
    ],

    /// RX inlink descriptor control.
    pub InLink [
        INLINK_ADDR    OFFSET(0)  NUMBITS(20) [],
        INLINK_START   OFFSET(22) NUMBITS(1) [],
        INLINK_RESTART OFFSET(23) NUMBITS(1) [],
        INLINK_PARK    OFFSET(24) NUMBITS(1) [],
    ],

    /// TX outlink descriptor control.
    pub OutLink [
        OUTLINK_ADDR    OFFSET(0)  NUMBITS(20) [],
        OUTLINK_START   OFFSET(21) NUMBITS(1) [],
        OUTLINK_RESTART OFFSET(22) NUMBITS(1) [],
        OUTLINK_PARK    OFFSET(23) NUMBITS(1) [],
    ],

    /// RX / TX peripheral selection (6-bit ID).
    pub PeriSel [
        PERI_SEL OFFSET(0) NUMBITS(6) [],
    ],

    /// MISC_CONF — global DMA configuration (offset 0x64).
    pub MiscConf [
        AHBM_RST_INTER OFFSET(0) NUMBITS(1) [],
        ARB_PRI_DIS    OFFSET(2) NUMBITS(1) [],
        CLK_EN         OFFSET(3) NUMBITS(1) [],
    ],
];

register_structs! {
    /// One IN_INT / OUT_INT interrupt cluster (RAW, ST, ENA, CLR).
    ///
    /// The same struct layout is used for both RX (input) and TX (output)
    /// interrupt clusters. The field type (`InInt` or `OutInt`) is selected
    /// by the caller — the raw `u32` register layout is identical.
    pub IntCluster {
        (0x00 => raw: ReadOnly<u32>),
        (0x04 => st: ReadOnly<u32>),
        (0x08 => ena: ReadWrite<u32>),
        (0x0c => clr: WriteOnly<u32>),
        (0x10 => @END),
    }
}

register_structs! {
    /// Global DMA registers (before the per-channel blocks).
    ///
    /// Layout: IN_INT_CH[0..3] (0x00..0x30), OUT_INT_CH[0..3]
    /// (0x30..0x60), AHB_TEST (0x60), MISC_CONF (0x64), DATE (0x68).
    pub DmaGlobal {
        (0x00 => _in_int_ch),
        (0x30 => _out_int_ch),
        (0x60 => _ahb_test),
        (0x64 => misc_conf: ReadWrite<u32, MiscConf::Register>),
        (0x68 => _date),
        (0x6c => _reserved),
        (0x70 => @END),
    }
}

register_structs! {
    /// One DMA channel register block (CH[n]).
    ///
    /// The RX (input) side occupies the first 0x60 bytes; a 0x2c-byte reserved
    /// gap separates it from the TX (output) side which starts at +0x60.
    pub DmaChannel {
        // ---- RX (input: peripheral → memory) ----
        (0x00 => in_conf0: ReadWrite<u32, InConf0::Register>),
        (0x04 => _in_conf1),
        (0x08 => _infifo_status),
        (0x0c => _in_pop),
        (0x10 => in_link: ReadWrite<u32, InLink::Register>),
        (0x14 => in_state: ReadOnly<u32>),
        (0x18 => _in_suc_eof_des_addr),
        (0x1c => _in_err_eof_des_addr),
        (0x20 => _in_dscr),
        (0x24 => _in_dscr_bf0),
        (0x28 => _in_dscr_bf1),
        (0x2c => _in_pri),
        (0x30 => in_peri_sel: ReadWrite<u32, PeriSel::Register>),
        (0x34 => _reserved_rx),
        // ---- TX (output: memory → peripheral) ----
        (0x60 => out_conf0: ReadWrite<u32, OutConf0::Register>),
        (0x64 => _out_conf1),
        (0x68 => _outfifo_status),
        (0x6c => _out_push),
        (0x70 => out_link: ReadWrite<u32, OutLink::Register>),
        (0x74 => out_state: ReadOnly<u32>),
        (0x78 => out_eof_des_addr: ReadOnly<u32>),
        (0x7c => _out_eof_bfr_des_addr),
        (0x80 => _out_dscr),
        (0x84 => _out_dscr_bf0),
        (0x88 => _out_dscr_bf1),
        (0x8c => _out_pri),
        (0x90 => out_peri_sel: ReadWrite<u32, PeriSel::Register>),
        (0x94 => _reserved_end),
        (0xa0 => @END),
    }
}

/// ESP32-C6 GDMA linked-list descriptor (3 words = 12 bytes).
///
/// `dw0` bitfield layout (per ESP32-C6 TRM and esp-hal):
/// - `size[11:0]`   — buffer capacity (set by software)
/// - `length[23:12]` — valid bytes (TX: set by SW; RX: written by HW)
/// - `suc_eof[30]`   — success EOF flag
/// - `owner[31]`     — 1 = owned by DMA, 0 = owned by CPU
///
/// `buffer` is the data buffer address (word-aligned, internal SRAM).
/// `next` is the next descriptor address, or null for end-of-list.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct DmaDescriptor {
    pub dw0: u32,
    pub buffer: *mut u8,
    pub next: *mut DmaDescriptor,
}

/// `dw0` bit positions.
const DW0_SIZE_MASK: u32 = 0xfff;
const DW0_SIZE_SHIFT: u32 = 0;
const DW0_LENGTH_SHIFT: u32 = 12;
pub const DW0_SUC_EOF: u32 = 1 << 30;
pub const DW0_OWNER_DMA: u32 = 1 << 31;

impl DmaDescriptor {
    /// Build a TX descriptor for `buf`. `suc_eof` marks the last descriptor so
    /// GDMA raises `OUT_TOTAL_EOF` after consuming it. Ownership is set to DMA.
    pub const fn for_tx(buf: *mut u8, len: usize, suc_eof: bool) -> Self {
        let size = (len as u32) & DW0_SIZE_MASK;
        let length = (len as u32) << DW0_LENGTH_SHIFT;
        let eof = if suc_eof { DW0_SUC_EOF } else { 0 };
        DmaDescriptor {
            dw0: size | length | eof | DW0_OWNER_DMA,
            buffer: buf,
            next: core::ptr::null_mut(),
        }
    }

    /// Build an RX descriptor for `buf`. `size` is the capacity; after the
    /// transfer completes, `length` (bits 23:12 of `dw0`) holds the received
    /// byte count. Ownership is set to DMA.
    pub const fn for_rx(buf: *mut u8, capacity: usize) -> Self {
        let size = (capacity as u32) & DW0_SIZE_MASK;
        DmaDescriptor {
            dw0: size | DW0_OWNER_DMA,
            buffer: buf,
            next: core::ptr::null_mut(),
        }
    }

    /// Received byte count (valid after an RX transfer).
    pub fn received_len(&self) -> usize {
        ((self.dw0 >> DW0_LENGTH_SHIFT) & DW0_SIZE_MASK) as usize
    }
}

/// Returns a `&'static DmaChannel` for channel `CH`.
fn channel_regs<const CH: usize>() -> &'static DmaChannel {
    assert!(CH < 3, "ESP32-C6 has only 3 GDMA channels");
    let base = DMA_BASE + CH_OFFSET + CH * CH_STRIDE;
    unsafe { &*(base as *const DmaChannel) }
}

/// Returns a `&'static IntCluster` for the RX (input) interrupt block of
/// channel `CH`.
fn in_int_regs<const CH: usize>() -> &'static IntCluster {
    let base = DMA_BASE + IN_INT_BASE + CH * INT_STRIDE;
    unsafe { &*(base as *const IntCluster) }
}

/// Returns a `&'static IntCluster` for the TX (output) interrupt block of
/// channel `CH`.
pub fn out_int_regs<const CH: usize>() -> &'static IntCluster {
    let base = DMA_BASE + OUT_INT_BASE + CH * INT_STRIDE;
    unsafe { &*(base as *const IntCluster) }
}

pub fn wait_for_bit(regs: &IntCluster, mask: u32) -> bool {
    for _ in 0..DMA_POLL_LIMIT {
        if regs.raw.get() & mask != 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

/// ESP32-C6 GDMA channel driver.
///
/// `CH` is the channel index (0, 1, or 2). A single channel can serve either
/// TX or RX for one peripheral — the I2S driver uses two instances (one for
/// playback, one for capture).
///
/// All methods are associated functions (no `&self` state) so the driver can
/// be used from a `static` context.
pub struct Esp32c6GdmaChannel<const CH: usize>;

impl<const CH: usize> Esp32c6GdmaChannel<CH> {
    pub const fn new() -> Self {
        assert!(CH < 3, "ESP32-C6 has only 3 GDMA channels");
        Self {}
    }

    /// Initialize the GDMA controller: reset the AHB master interface and
    /// force-enable the register clock.
    ///
    /// This must be called once before any DMA transfer. It mirrors esp-hal's
    /// `init_dma_racey()`. Without `CLK_EN = 1`, the DMA engine's register
    /// clock is only active during CPU writes, which can cause transfers to
    /// stall indefinitely.
    pub fn init_dma() {
        let global = unsafe { &*(DMA_BASE as *const DmaGlobal) };
        // Reset the AHB master FSM, then release.
        global.misc_conf.modify(MiscConf::AHBM_RST_INTER::SET);
        global.misc_conf.modify(MiscConf::AHBM_RST_INTER::CLEAR);
        // Force clock on for all DMA registers.
        global.misc_conf.modify(MiscConf::CLK_EN::SET);
    }

    // ---- RX (input: peripheral → memory) ----

    /// Reset the RX FSM and FIFO.
    pub fn reset_rx() {
        let ch = channel_regs::<CH>();
        ch.in_conf0.modify(InConf0::IN_RST::SET);
        ch.in_conf0.modify(InConf0::IN_RST::CLEAR);
    }

    /// Select which peripheral feeds this RX channel (`peri_sel`).
    pub fn set_rx_peri(peri: u32) {
        let ch = channel_regs::<CH>();
        ch.in_peri_sel.write(PeriSel::PERI_SEL.val(peri & 0x3f));
    }

    /// Start an RX transfer using `desc` as the inlink descriptor. The
    /// descriptor (and its buffer) must remain valid until the transfer
    /// completes.
    pub fn start_rx(desc: &DmaDescriptor) {
        Self::reset_rx();
        // Clear any pending RX interrupt status.
        let int = in_int_regs::<CH>();
        int.clr.set(InInt::IN_DONE.val(1).value | InInt::IN_SUC_EOF.val(1).value | InInt::IN_DSCR_ERR.val(1).value);
        // Ensure descriptor writes are visible to the DMA engine before start.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        // Set the descriptor address and kick off — use modify() to preserve
        // other fields (e.g. INLINK_AUTO_RET). Do NOT set RESTART.
        let ch = channel_regs::<CH>();
        let desc_addr = (addr_of!(*desc) as usize as u32) & ((1 << 20) - 1);
        ch.in_link.modify(InLink::INLINK_ADDR.val(desc_addr));
        ch.in_link.modify(InLink::INLINK_START::SET);
    }

    /// Returns `true` once the RX transfer has completed (`IN_SUC_EOF`).
    pub fn is_rx_done() -> bool {
        in_int_regs::<CH>().raw.get() & InInt::IN_SUC_EOF.mask != 0
    }

    /// Block until the RX transfer completes. Returns `Err` on timeout or
    /// descriptor error. On error the caller may invoke
    /// [`capture_gdma_status`] to obtain a diagnostic snapshot for logging.
    pub fn wait_rx_done() -> blueos_hal::err::Result<()> {
        let int = in_int_regs::<CH>();
        let mask = InInt::IN_SUC_EOF.mask
            | InInt::IN_DSCR_ERR.mask
            | InInt::IN_DSCR_EMPTY.mask;
        loop {
            let raw = int.raw.get();
            if raw & InInt::IN_SUC_EOF.mask != 0 {
                return Ok(());
            }
            if raw & InInt::IN_DSCR_ERR.mask != 0 {
                log::warn!("[GDMA] RX DSCR_ERR, raw=0x{:08x}", raw);
                return Err(blueos_hal::err::HalError::Fail);
            }
            if raw & InInt::IN_DSCR_EMPTY.mask != 0 {
                log::warn!("[GDMA] RX DSCR_EMPTY, raw=0x{:08x}", raw);
                return Err(blueos_hal::err::HalError::Fail);
            }
            if !wait_for_bit(int, mask) {
                log::warn!("[GDMA] RX timeout, raw=0x{:08x}", int.raw.get());
                return Err(blueos_hal::err::HalError::Timeout);
            }
        }
    }

    // ---- TX (output: memory → peripheral) ----

    /// Reset the TX FSM and FIFO.
    pub fn reset_tx() {
        let ch = channel_regs::<CH>();
        ch.out_conf0.modify(OutConf0::OUT_RST::SET);
        ch.out_conf0.modify(OutConf0::OUT_RST::CLEAR);
    }

    /// Select which peripheral this TX channel feeds (`peri_sel`).
    pub fn set_tx_peri(peri: u32) {
        let ch = channel_regs::<CH>();
        ch.out_peri_sel.write(PeriSel::PERI_SEL.val(peri & 0x3f));
    }

    /// Start a TX transfer using `desc` as the outlink descriptor. The
    /// descriptor (and its buffer) must remain valid until the transfer
    /// completes.
    pub fn start_tx(desc: &DmaDescriptor) {
        Self::reset_tx();
        // Clear any pending TX interrupt status.
        let int = out_int_regs::<CH>();
        int.clr.set(OutInt::OUT_DONE.val(1).value | OutInt::OUT_EOF.val(1).value | OutInt::OUT_TOTAL_EOF.val(1).value | OutInt::OUT_DSCR_ERR.val(1).value);
        // Ensure descriptor writes are visible to the DMA engine before start.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        let ch = channel_regs::<CH>();
        // OUT_EOF_MODE=1: OUT_EOF fires after the last data of a suc_eof=1
        // descriptor is *taken from* the GDMA TX channel (pushed to the
        // peripheral FIFO), guaranteeing the data has reached the I2S FIFO.
        //ch.out_conf0.modify(OutConf0::OUT_EOF_MODE::SET);
        let desc_addr = (addr_of!(*desc) as usize as u32) & ((1 << 20) - 1);
        ch.out_link.modify(OutLink::OUTLINK_ADDR.val(desc_addr));
        ch.out_link.modify(OutLink::OUTLINK_START::SET);
    }

    /// Start a TX transfer without resetting the TX FSM/FIFO.
    ///
    /// Identical to [`start_tx`] but skips [`reset_tx`], so the I2S FIFO is
    /// preserved between consecutive transfers. This eliminates the audio gap
    /// (FIFO underflow) that otherwise occurs on every `write()` call when
    /// streaming chunked audio data.
    pub fn start_tx_no_reset(desc: &DmaDescriptor) {
        let int = out_int_regs::<CH>();
        int.clr.set(OutInt::OUT_DONE.val(1).value | OutInt::OUT_EOF.val(1).value | OutInt::OUT_TOTAL_EOF.val(1).value | OutInt::OUT_DSCR_ERR.val(1).value);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        let ch = channel_regs::<CH>();
        ch.out_conf0.modify(OutConf0::OUT_EOF_MODE::SET + OutConf0::OUT_AUTO_WRBACK::SET);
        let desc_addr = (addr_of!(*desc) as usize as u32) & ((1 << 20) - 1);
        ch.out_link.modify(OutLink::OUTLINK_ADDR.val(desc_addr));
        ch.out_link.modify(OutLink::OUTLINK_START::SET);
    }

    /// Returns `true` once the TX transfer has completed (`OUT_EOF` with
    /// `OUT_EOF_MODE=1`, i.e. the last data of the `suc_eof=1` descriptor has
    /// been pushed to the peripheral FIFO).
    pub fn is_tx_done() -> bool {
        out_int_regs::<CH>().raw.get() & OutInt::OUT_EOF.mask != 0
    }

    /// Returns `true` if the `OUT_EOF` interrupt raw bit is set — i.e. a
    /// descriptor with `suc_eof=1` has been consumed by the DMA engine.
    pub fn is_out_eof() -> bool {
        out_int_regs::<CH>().raw.get() & OutInt::OUT_EOF.mask != 0
    }

    /// Clear the `OUT_EOF` interrupt raw bit.
    pub fn clear_out_eof() {
        out_int_regs::<CH>().clr.set(OutInt::OUT_EOF.val(1).value);
    }

    /// Clear the `OUT_TOTAL_EOF` interrupt raw bit.
    pub fn clear_out_total_eof() {
        out_int_regs::<CH>()
            .clr
            .set(OutInt::OUT_TOTAL_EOF.val(1).value);
    }

    /// Returns `true` if the `OUT_TOTAL_EOF` interrupt raw bit is set — i.e.
    /// the DMA engine has consumed the last descriptor of a terminated chain
    /// (`next = null`) and come to a halt.
    pub fn is_out_total_eof() -> bool {
        out_int_regs::<CH>().raw.get() & OutInt::OUT_TOTAL_EOF.mask != 0
    }

    /// Read the `OUT_EOF_DES_ADDR` register — the full 32-bit address of the
    /// descriptor whose `suc_eof=1` data was most recently consumed by the
    /// DMA engine. Combined with the base address of the descriptor array,
    /// the caller can compute which descriptor index was just consumed.
    pub fn read_out_eof_des_addr() -> u32 {
        channel_regs::<CH>().out_eof_des_addr.get()
    }

    /// Restart the TX outlink after the DMA engine paused due to encountering
    /// a descriptor with `owner=0`. The caller must have set `owner=1` on the
    /// next descriptor before calling this.
    pub fn restart_tx() {
        let ch = channel_regs::<CH>();
        ch.out_link.modify(OutLink::OUTLINK_RESTART::SET);
    }

    /// Read the current `OUT_CONF0` register value (for diagnostics).
    pub fn read_out_conf0() -> u32 {
        channel_regs::<CH>().out_conf0.get()
    }

    /// Read the current outlink descriptor address from `OUT_STATE` register.
    ///
    /// `OUT_STATE` bits[17:0] contain the address of the descriptor the DMA
    /// engine is currently processing. By comparing this with the base address
    /// of the descriptor array, the caller can compute which descriptor index
    /// the DMA is currently on, and thus which descriptors are safe to refill.
    pub fn read_outlink_dscr_addr() -> u32 {
        channel_regs::<CH>().out_state.get() & 0x0003_ffff
    }

    /// Check if `OUT_DSCR_ERR` is set — descriptor error (e.g. invalid address).
    pub fn is_out_dscr_err() -> bool {
        out_int_regs::<CH>().raw.get() & OutInt::OUT_DSCR_ERR.mask != 0
    }

    /// Clear the `OUT_DSCR_ERR` interrupt raw bit.
    pub fn clear_out_dscr_err() {
        out_int_regs::<CH>()
            .clr
            .set(OutInt::OUT_DSCR_ERR.val(1).value);
    }

    /// Block until the TX transfer completes. Returns `Err` on timeout or
    /// descriptor error. On error the caller may invoke
    /// [`capture_gdma_status`] to obtain a diagnostic snapshot for logging.
    pub fn wait_tx_done() -> blueos_hal::err::Result<()> {
        let int = out_int_regs::<CH>();
        loop {
            let raw = int.raw.get();
            if raw & OutInt::OUT_TOTAL_EOF.mask != 0 {
                return Ok(());
            }
            if raw & OutInt::OUT_DSCR_ERR.mask != 0 {
                log::warn!("[GDMA] TX DSCR_ERR, raw=0x{:08x}", raw);
                return Err(blueos_hal::err::HalError::Fail);
            }
            if !wait_for_bit(int, OutInt::OUT_TOTAL_EOF.mask | OutInt::OUT_DSCR_ERR.mask) {
                log::warn!("[GDMA] TX timeout, raw=0x{:08x}", int.raw.get());
                return Err(blueos_hal::err::HalError::Timeout);
            }
        }
    }

    // ---- M2M (memory-to-memory) self-test ----

    /// Dump channel register state for debugging.
    pub fn dump_channel_state() {
        let ch = channel_regs::<CH>();
        log::info!(
            "[GDMA ch{}] in_conf0=0x{:08x} in_link=0x{:08x} in_peri_sel=0x{:08x} in_state=0x{:08x}",
            CH,
            ch.in_conf0.get(),
            ch.in_link.get(),
            ch.in_peri_sel.get(),
            ch.in_state.get()
        );
        log::info!(
            "[GDMA ch{}] out_conf0=0x{:08x} out_link=0x{:08x} out_peri_sel=0x{:08x} out_state=0x{:08x}",
            CH,
            ch.out_conf0.get(),
            ch.out_link.get(),
            ch.out_peri_sel.get(),
            ch.out_state.get()
        );
    }

    /// Run a memory-to-memory DMA transfer to verify both outlink (read) and
    /// inlink (write) paths without any peripheral attached.
    ///
    /// `src` is the source buffer (read by outlink), `dst` is the destination
    /// buffer (written by inlink). Both must be word-aligned and reside in
    /// internal SRAM. The caller fills `src` with a known pattern and zeroes
    /// `dst` before calling; after `Ok(())` returns, `dst` should match `src`.
    ///
    /// This method uses the same channel for both TX and RX: it sets
    /// `MEM_TRANS_EN`, starts the outlink first (to feed data into the DMA
    /// internal FIFO), then starts the inlink (to drain the FIFO into `dst`).
    pub fn m2m_transfer(src: &mut [u8], dst: &mut [u8]) -> blueos_hal::err::Result<()> {
        if src.len() != dst.len() || src.is_empty() {
            return Err(blueos_hal::err::HalError::InvalidParam);
        }
        let len = src.len();

        // Ensure the GDMA controller clock is enabled (idempotent).
        Self::init_dma();

        let ch = channel_regs::<CH>();

        // ── TRM Step 1: Reset TX FSM and FIFO pointer ──
        ch.out_conf0.modify(OutConf0::OUT_RST::SET);
        ch.out_conf0.modify(OutConf0::OUT_RST::CLEAR);

        // ── TRM Step 2: Reset RX FSM and FIFO pointer ──
        ch.in_conf0.modify(InConf0::IN_RST::SET);
        ch.in_conf0.modify(InConf0::IN_RST::CLEAR);

        // Select SPI2 (peri_id = 0) as the pseudo-peripheral for both
        // directions. In M2M mode the hardware ignores the peripheral FIFO,
        // but esp-hal still sets peri_sel — we match that to be safe.
        Self::set_tx_peri(1);
        Self::set_rx_peri(1);

        // Enable descriptor burst reads on both sides.
        ch.out_conf0.modify(
            OutConf0::OUTDSCR_BURST_EN::SET
                + OutConf0::OUT_EOF_MODE::SET,
        );
        ch.in_conf0.modify(InConf0::INDSCR_BURST_EN::SET);

        // Build descriptors: outlink reads from `src`, inlink writes to `dst`.
        // Both descriptors have owner = DMA (bit 31 set) so the hardware will
        // process them. The TX descriptor has suc_eof = 1 (TRM step 8).
        let tx_desc = DmaDescriptor::for_tx(src.as_ptr() as *mut u8, len, true);
        let rx_desc = DmaDescriptor::for_rx(dst.as_mut_ptr(), len);

        // Ensure descriptor writes are visible to the DMA engine before start.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        // Clear pending interrupts.
        let in_int = in_int_regs::<CH>();
        in_int.clr.set(
            InInt::IN_DONE.val(1).value
                | InInt::IN_SUC_EOF.val(1).value
                | InInt::IN_ERR_EOF.val(1).value
                | InInt::IN_DSCR_ERR.val(1).value
                | InInt::IN_DSCR_EMPTY.val(1).value,
        );
        let out_int = out_int_regs::<CH>();
        out_int.clr.set(
            OutInt::OUT_DONE.val(1).value
                | OutInt::OUT_EOF.val(1).value
                | OutInt::OUT_TOTAL_EOF.val(1).value
                | OutInt::OUT_DSCR_ERR.val(1).value,
        );

        // ── TRM Step 3: Mount TX outlink — set OUTLINK_ADDR ──
        let tx_desc_addr = (addr_of!(tx_desc) as usize as u32) & ((1 << 20) - 1);
        ch.out_link.modify(OutLink::OUTLINK_ADDR.val(tx_desc_addr));

        // ── TRM Step 4: Mount RX inlink — set INLINK_ADDR ──
        let rx_desc_addr = (addr_of!(rx_desc) as usize as u32) & ((1 << 20) - 1);
        ch.in_link.modify(InLink::INLINK_ADDR.val(rx_desc_addr));

        // ── TRM Step 5: Enable memory-to-memory mode ──
        ch.in_conf0.modify(InConf0::MEM_TRANS_EN::SET);

        // ── TRM Step 6: Start TX channel — OUTLINK_START ──
        ch.out_link.modify(OutLink::OUTLINK_START::SET);

        // ── TRM Step 7: Start RX channel — INLINK_START ──
        ch.in_link.modify(InLink::INLINK_START::SET);

        // ── TRM Step 8: Wait for IN_SUC_EOF (TX descriptor's suc_eof
        //                propagates through M2M to trigger IN_SUC_EOF) ──
        let result = Self::wait_rx_done();

        // Clean up: disable M2M mode.
        ch.in_conf0.modify(InConf0::MEM_TRANS_EN::CLEAR);

        result
    }
}

unsafe impl<const CH: usize> Send for Esp32c6GdmaChannel<CH> {}
unsafe impl<const CH: usize> Sync for Esp32c6GdmaChannel<CH> {}
