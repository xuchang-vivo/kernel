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

//! ESP32-C6 GPSPI2 register-level driver with quad-output support.
//!
//! The CO5300 uses a SPI-flash-shaped QSPI protocol: an 8-bit opcode and
//! 24-bit address are sent on one line, followed by pixel data on four lines.
//! [`blueos_hal::spi::Qspi::write_quad`] switches the data phase to four lines.
//! Chip select is controlled by the display bus so a single-line header and
//! quad payload remain part of one logical transaction.

use crate::spi::{SpiBitOrder, SpiConfig, SpiPhase, SpiPolarity};
use blueos_hal::{Configuration, PlatPeri};

const FIFO_SIZE: usize = 64;
const SPI_CMD_TIMEOUT: usize = 100_000;
const EMPTY_WRITE_PAD: u8 = 0;

const REG_CMD: usize = 0x00;
const REG_ADDR: usize = 0x04;
const REG_CTRL: usize = 0x08;
const REG_CLOCK: usize = 0x0c;
const REG_USER: usize = 0x10;
const REG_USER1: usize = 0x14;
const REG_USER2: usize = 0x18;
const REG_MS_DLEN: usize = 0x1c;
const REG_MISC: usize = 0x20;
const REG_DMA_CONF: usize = 0x30;
const REG_DMA_INT_CLR: usize = 0x38;
const REG_W0: usize = 0x98;
const REG_SLAVE: usize = 0xe0;
const REG_CLK_GATE: usize = 0xe8;

const CMD_UPDATE: u32 = 1 << 23;
const CMD_USR: u32 = 1 << 24;

const CTRL_QUAD_PHASES: u32 = (1 << 6) | (1 << 9) | (1 << 15);
const CTRL_RD_BIT_ORDER_MASK: u32 = 0b11 << 23;
const CTRL_WR_BIT_ORDER_MASK: u32 = 0b11 << 25;

const USER_DOUTDIN: u32 = 1 << 0;
const USER_QPI_MODE: u32 = 1 << 3;
const USER_CK_OUT_EDGE: u32 = 1 << 9;
const USER_FWRITE_DUAL: u32 = 1 << 12;
const USER_FWRITE_QUAD: u32 = 1 << 13;
const USER_SIO: u32 = 1 << 17;
const USER_USR_MOSI: u32 = 1 << 27;
const USER_USR_MISO: u32 = 1 << 28;
const USER_USR_DUMMY: u32 = 1 << 29;
const USER_USR_ADDR: u32 = 1 << 30;
const USER_USR_COMMAND: u32 = 1 << 31;
const USER_PHASE_MASK: u32 = USER_DOUTDIN
    | USER_QPI_MODE
    | USER_FWRITE_DUAL
    | USER_FWRITE_QUAD
    | USER_SIO
    | USER_USR_MOSI
    | USER_USR_MISO
    | USER_USR_DUMMY
    | USER_USR_ADDR
    | USER_USR_COMMAND;

const MISC_CS_DISABLE_MASK: u32 = 0x3f;
const MISC_CK_IDLE_EDGE: u32 = 1 << 29;
const MISC_CS_KEEP_ACTIVE: u32 = 1 << 30;
const MISC_QUAD_DIN_PIN_SWAP: u32 = 1 << 31;

const DMA_RX_ENA: u32 = 1 << 27;
const DMA_TX_ENA: u32 = 1 << 28;
const RX_AFIFO_RST: u32 = 1 << 29;
const BUF_AFIFO_RST: u32 = 1 << 30;
const DMA_TRANS_DONE: u32 = 1 << 12;

const SLAVE_MODE: u32 = 1 << 26;
const SLAVE_SOFT_RESET: u32 = 1 << 27;

const PCR_SPI2_CONF: usize = 0xc0;
const PCR_SPI2_CLKM_CONF: usize = 0xc4;
const PCR_SPI2_CLK_EN: u32 = 1 << 0;
const PCR_SPI2_RST_EN: u32 = 1 << 1;
const PCR_SPI2_CLKM_SEL_MASK: u32 = 0b11 << 20;
const PCR_SPI2_CLKM_SEL_80M: u32 = 1 << 20;
const PCR_SPI2_CLKM_EN: u32 = 1 << 22;

#[inline]
unsafe fn read32(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline]
unsafe fn write32(addr: usize, value: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) };
}

fn wait_until_clear(addr: usize, mask: u32) -> blueos_hal::err::Result<()> {
    for _ in 0..SPI_CMD_TIMEOUT {
        if unsafe { read32(addr) } & mask == 0 {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(blueos_hal::err::HalError::Timeout)
}

/// ESP32-C6 GPSPI2 peripheral.
///
/// `SPI_BASE` is normally `0x6008_1000`, `PCR_BASE` is `0x6009_6000`, and
/// `SOURCE_HZ` is 80 MHz when the PCR clock source is set to PLL/80M.
pub struct Esp32c6Spi2<const SPI_BASE: usize, const PCR_BASE: usize, const SOURCE_HZ: u32> {}

unsafe impl<const SPI_BASE: usize, const PCR_BASE: usize, const SOURCE_HZ: u32> Send
    for Esp32c6Spi2<SPI_BASE, PCR_BASE, SOURCE_HZ>
{
}

unsafe impl<const SPI_BASE: usize, const PCR_BASE: usize, const SOURCE_HZ: u32> Sync
    for Esp32c6Spi2<SPI_BASE, PCR_BASE, SOURCE_HZ>
{
}

impl<const SPI_BASE: usize, const PCR_BASE: usize, const SOURCE_HZ: u32>
    Esp32c6Spi2<SPI_BASE, PCR_BASE, SOURCE_HZ>
{
    pub const fn new() -> Self {
        Self {}
    }

    #[inline]
    fn read_reg(offset: usize) -> u32 {
        unsafe { read32(SPI_BASE + offset) }
    }

    #[inline]
    fn write_reg(offset: usize, value: u32) {
        unsafe { write32(SPI_BASE + offset, value) };
    }

    #[inline]
    fn modify_reg(offset: usize, clear: u32, set: u32) {
        let value = Self::read_reg(offset);
        Self::write_reg(offset, (value & !clear) | set);
    }

    fn write_fifo(data: &[u8]) {
        debug_assert!(data.len() <= FIFO_SIZE);
        for (index, chunk) in data.chunks(4).enumerate() {
            let mut word = 0u32;
            for (byte_index, byte) in chunk.iter().enumerate() {
                word |= (*byte as u32) << (byte_index * 8);
            }
            Self::write_reg(REG_W0 + index * 4, word);
        }
    }

    fn read_fifo(data: &mut [u8]) {
        debug_assert!(data.len() <= FIFO_SIZE);
        for (index, byte) in data.iter_mut().enumerate() {
            let word = Self::read_reg(REG_W0 + (index / 4) * 4);
            *byte = ((word >> ((index % 4) * 8)) & 0xff) as u8;
        }
    }

    fn reset_fifo(tx: bool, rx: bool) {
        let mask = if tx { BUF_AFIFO_RST } else { 0 } | if rx { RX_AFIFO_RST } else { 0 };
        if mask != 0 {
            Self::modify_reg(REG_DMA_CONF, 0, mask);
            Self::modify_reg(REG_DMA_CONF, mask, 0);
        }
    }

    fn start_transfer() -> blueos_hal::err::Result<()> {
        Self::modify_reg(REG_CMD, 0, CMD_UPDATE);
        wait_until_clear(SPI_BASE + REG_CMD, CMD_UPDATE)?;
        Self::write_reg(REG_DMA_INT_CLR, DMA_TRANS_DONE);
        Self::modify_reg(REG_CMD, 0, CMD_USR);
        wait_until_clear(SPI_BASE + REG_CMD, CMD_USR)
    }

    fn configure_clock(baudrate: u32) -> blueos_hal::err::Result<()> {
        if baudrate == 0 || SOURCE_HZ == 0 {
            return Err(blueos_hal::err::HalError::InvalidParam);
        }

        if baudrate >= SOURCE_HZ {
            Self::write_reg(REG_CLOCK, 1 << 31);
            return Ok(());
        }

        // Search the complete C6 divider space and choose the highest clock
        // that does not exceed the requested rate.
        let mut best_hz = 0u32;
        let mut best_pre = 0u32;
        let mut best_n = 0u32;
        for pre in 0..16u32 {
            for n in 1..64u32 {
                let actual = SOURCE_HZ / ((pre + 1) * (n + 1));
                if actual <= baudrate && actual > best_hz {
                    best_hz = actual;
                    best_pre = pre;
                    best_n = n;
                }
            }
        }
        if best_hz == 0 {
            return Err(blueos_hal::err::HalError::NotSupport);
        }

        let high = ((best_n + 1) / 2).saturating_sub(1);
        Self::write_reg(
            REG_CLOCK,
            best_n | (high << 6) | (best_n << 12) | (best_pre << 18),
        );
        Ok(())
    }

    fn configure_data_mode(quad_write: bool) {
        // Commands and addresses remain single-line. Only the write data phase
        // is switched to four lines.
        Self::modify_reg(REG_CTRL, CTRL_QUAD_PHASES, 0);
        Self::modify_reg(
            REG_USER,
            USER_PHASE_MASK,
            USER_USR_MOSI | if quad_write { USER_FWRITE_QUAD } else { 0 },
        );
    }

    fn write_chunks(data: &[u8], quad: bool) -> blueos_hal::err::Result<()> {
        if data.is_empty() {
            return Ok(());
        }
        Self::configure_data_mode(quad);
        for chunk in data.chunks(FIFO_SIZE) {
            Self::reset_fifo(true, false);
            Self::write_reg(REG_MS_DLEN, chunk.len() as u32 * 8 - 1);
            Self::write_fifo(chunk);
            Self::start_transfer()?;
        }
        Ok(())
    }

    fn read_chunks(data: &mut [u8]) -> blueos_hal::err::Result<()> {
        if data.is_empty() {
            return Ok(());
        }

        // A clocked full-duplex read is used for the generic SPI interface.
        // This matches the behavior of the existing ESP32-C3 implementation.
        Self::modify_reg(REG_CTRL, CTRL_QUAD_PHASES, 0);
        Self::modify_reg(
            REG_USER,
            USER_PHASE_MASK,
            USER_DOUTDIN | USER_USR_MOSI | USER_USR_MISO,
        );
        for chunk in data.chunks_mut(FIFO_SIZE) {
            Self::reset_fifo(true, true);
            Self::write_reg(REG_MS_DLEN, chunk.len() as u32 * 8 - 1);
            let padding = [EMPTY_WRITE_PAD; FIFO_SIZE];
            Self::write_fifo(&padding[..chunk.len()]);
            Self::start_transfer()?;
            Self::read_fifo(chunk);
        }
        Ok(())
    }

    fn transfer_chunks(read: &mut [u8], write: &[u8]) -> blueos_hal::err::Result<()> {
        if read.is_empty() {
            return Self::write_chunks(write, false);
        }
        if write.is_empty() {
            return Self::read_chunks(read);
        }

        Self::modify_reg(REG_CTRL, CTRL_QUAD_PHASES, 0);
        Self::modify_reg(
            REG_USER,
            USER_PHASE_MASK,
            USER_DOUTDIN | USER_USR_MOSI | USER_USR_MISO,
        );

        let total = read.len().max(write.len());
        let mut offset = 0usize;
        while offset < total {
            let len = core::cmp::min(FIFO_SIZE, total - offset);
            let mut tx = [EMPTY_WRITE_PAD; FIFO_SIZE];
            if offset < write.len() {
                let write_len = core::cmp::min(len, write.len() - offset);
                tx[..write_len].copy_from_slice(&write[offset..offset + write_len]);
            }

            Self::reset_fifo(true, true);
            Self::write_reg(REG_MS_DLEN, len as u32 * 8 - 1);
            Self::write_fifo(&tx[..len]);
            Self::start_transfer()?;

            if offset < read.len() {
                let read_len = core::cmp::min(len, read.len() - offset);
                let mut rx = [0u8; FIFO_SIZE];
                Self::read_fifo(&mut rx[..len]);
                read[offset..offset + read_len].copy_from_slice(&rx[..read_len]);
            }
            offset += len;
        }
        Ok(())
    }

    fn write_qspi_transaction(header: &[u8; 4], data: &[u8]) -> blueos_hal::err::Result<()> {
        if data.is_empty() {
            return Self::write_chunks(header, false);
        }

        // Encode the flash-shaped header as a one-line 8-bit command plus a
        // one-line 24-bit address, followed by quad write data.
        let address = u32::from_be_bytes([header[1], header[2], header[3], 0]);
        let mut first = true;
        for chunk in data.chunks(FIFO_SIZE) {
            Self::configure_data_mode(true);
            if first {
                Self::modify_reg(REG_USER, 0, USER_USR_COMMAND | USER_USR_ADDR);
                Self::write_reg(REG_USER2, (7u32 << 28) | u32::from(header[0]));
                Self::modify_reg(REG_USER1, 0b1_1111 << 27, 23u32 << 27);
                Self::write_reg(REG_ADDR, address);
                first = false;
            }

            Self::reset_fifo(true, false);
            Self::write_reg(REG_MS_DLEN, chunk.len() as u32 * 8 - 1);
            Self::write_fifo(chunk);
            Self::start_transfer()?;
        }
        Ok(())
    }
}

impl<const SPI_BASE: usize, const PCR_BASE: usize, const SOURCE_HZ: u32> PlatPeri
    for Esp32c6Spi2<SPI_BASE, PCR_BASE, SOURCE_HZ>
{
    fn enable(&self) {
        unsafe {
            let conf_addr = PCR_BASE + PCR_SPI2_CONF;
            let conf = read32(conf_addr) | PCR_SPI2_CLK_EN;
            // Match ESP-IDF's ESP32-C6 SPI LL reset pulse: assert the PCR
            // reset bit, then clear it before accessing the peripheral.  The
            // generated PAC description says "set 0 to reset", but the
            // official low-level implementation and the hardware behavior use
            // a high pulse.  Leaving this bit set makes register polling look
            // successful while SPI2 produces no output on the pins.
            write32(conf_addr, conf | PCR_SPI2_RST_EN);
            write32(conf_addr, conf & !PCR_SPI2_RST_EN);

            let clkm_addr = PCR_BASE + PCR_SPI2_CLKM_CONF;
            let clkm = read32(clkm_addr);
            write32(
                clkm_addr,
                (clkm & !PCR_SPI2_CLKM_SEL_MASK) | PCR_SPI2_CLKM_SEL_80M | PCR_SPI2_CLKM_EN,
            );
        }
        Self::write_reg(REG_CLK_GATE, 0b111);
    }

    fn disable(&self) {
        Self::modify_reg(REG_CLK_GATE, 0b011, 0);
        unsafe {
            let clkm_addr = PCR_BASE + PCR_SPI2_CLKM_CONF;
            write32(clkm_addr, read32(clkm_addr) & !PCR_SPI2_CLKM_EN);
        }
    }
}

impl<const SPI_BASE: usize, const PCR_BASE: usize, const SOURCE_HZ: u32> Configuration<SpiConfig>
    for Esp32c6Spi2<SPI_BASE, PCR_BASE, SOURCE_HZ>
{
    type Target = ();

    fn configure(&self, config: &SpiConfig) -> blueos_hal::err::Result<()> {
        self.enable();

        Self::modify_reg(REG_SLAVE, SLAVE_MODE, SLAVE_SOFT_RESET);
        Self::modify_reg(REG_SLAVE, SLAVE_SOFT_RESET | SLAVE_MODE, 0);
        Self::modify_reg(REG_DMA_CONF, DMA_RX_ENA | DMA_TX_ENA, 0);
        Self::reset_fifo(true, true);

        let idle_high = matches!(config.polarity, SpiPolarity::High);
        Self::modify_reg(
            REG_MISC,
            MISC_CS_DISABLE_MASK | MISC_CK_IDLE_EDGE | MISC_CS_KEEP_ACTIVE | MISC_QUAD_DIN_PIN_SWAP,
            MISC_CS_DISABLE_MASK | if idle_high { MISC_CK_IDLE_EDGE } else { 0 },
        );

        let trailing_edge = matches!(
            (config.polarity, config.phase),
            (SpiPolarity::Low, SpiPhase::Phase1) | (SpiPolarity::High, SpiPhase::Phase0)
        );
        Self::modify_reg(
            REG_USER,
            USER_CK_OUT_EDGE,
            if trailing_edge { USER_CK_OUT_EDGE } else { 0 },
        );

        let lsb_first = matches!(config.bit_order, SpiBitOrder::LsbFirst);
        Self::modify_reg(
            REG_CTRL,
            CTRL_RD_BIT_ORDER_MASK | CTRL_WR_BIT_ORDER_MASK,
            if lsb_first { (1 << 23) | (1 << 25) } else { 0 },
        );
        Self::configure_clock(config.baudrate)?;
        Self::configure_data_mode(false);
        Ok(())
    }
}

impl<const SPI_BASE: usize, const PCR_BASE: usize, const SOURCE_HZ: u32>
    blueos_hal::spi::Spi<SpiConfig, ()> for Esp32c6Spi2<SPI_BASE, PCR_BASE, SOURCE_HZ>
{
    fn transfer(&self, read: &mut [u8], write: &[u8]) -> blueos_hal::err::Result<()> {
        Self::transfer_chunks(read, write)
    }

    fn read(&self, data: &mut [u8]) -> blueos_hal::err::Result<()> {
        Self::read_chunks(data)
    }

    fn write(&self, data: &[u8]) -> blueos_hal::err::Result<()> {
        Self::write_chunks(data, false)
    }
}

impl<const SPI_BASE: usize, const PCR_BASE: usize, const SOURCE_HZ: u32> blueos_hal::spi::Qspi
    for Esp32c6Spi2<SPI_BASE, PCR_BASE, SOURCE_HZ>
{
    fn write_quad(&self, data: &[u8]) -> blueos_hal::err::Result<()> {
        Self::write_chunks(data, true)
    }

    fn write_qspi(&self, header: &[u8; 4], data: &[u8]) -> blueos_hal::err::Result<()> {
        Self::write_qspi_transaction(header, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use blueos_test_macro::test;

    #[test]
    fn wait_until_clear_times_out() {
        // Exercise the polling behavior through a local volatile word rather
        // than touching a peripheral address.
        let value = 1u32;
        let address = core::ptr::addr_of!(value) as usize;
        assert_eq!(
            wait_until_clear(address, 1),
            Err(blueos_hal::err::HalError::Timeout)
        );
    }
}
