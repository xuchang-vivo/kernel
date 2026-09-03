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

use blueos_driver::spi::SpiConfig;
use blueos_hal::PlatPeri;

use crate::devices::bus::BusInterface;

pub struct BlockSpi<T: PlatPeri, G: blueos_hal::gpio::OutputPin> {
    inner: &'static T,
    cs: &'static G,
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()>, G: blueos_hal::gpio::OutputPin> BlockSpi<T, G> {
    pub fn new(
        inner: &'static T,
        cs: &'static G,
        config: &SpiConfig,
    ) -> Result<Self, blueos_hal::err::HalError> {
        inner.configure(config)?;
        Ok(BlockSpi { inner, cs })
    }

    /// Reconfigure the shared SPI peripheral while no device transaction is
    /// active.  SD cards require a slow identification clock before switching
    /// to their normal data rate.
    pub fn configure(&self, config: &SpiConfig) -> Result<(), blueos_hal::err::HalError> {
        self.inner.configure(config)
    }

    pub fn clock_idle(&mut self, bytes: &[u8]) -> Result<(), crate::error::Error> {
        self.inner.write(bytes).map_err(|_| crate::error::code::EIO)
    }

    pub fn assert_cs(&self) {
        self.cs.set_low().ok();
    }

    pub fn deassert_cs(&self) {
        self.cs.set_high().ok();
    }

    pub fn read(&mut self, words: &mut [u8]) -> Result<(), crate::error::Error> {
        self.inner.read(words).map_err(|_| crate::error::code::EIO)
    }

    pub fn write(&mut self, words: &[u8]) -> Result<(), crate::error::Error> {
        self.inner.write(words).map_err(|_| crate::error::code::EIO)
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), crate::error::Error> {
        self.inner
            .transfer(read, write)
            .map_err(|_| crate::error::code::EIO)
    }

    pub fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), crate::error::Error> {
        self.inner
            .write(words)
            .map_err(|_| crate::error::code::EIO)?;
        self.inner.read(words).map_err(|_| crate::error::code::EIO)
    }
}

impl<
        T: blueos_hal::spi::Spi<SpiConfig, ()> + blueos_hal::spi::Qspi,
        G: blueos_hal::gpio::OutputPin,
    > BlockSpi<T, G>
{
    pub fn write_quad(&mut self, words: &[u8]) -> Result<(), crate::error::Error> {
        self.inner
            .write_quad(words)
            .map_err(|_| crate::error::code::EIO)
    }

    /// Send the QSPI command/address header plus payload. As with `write` and
    /// `write_quad`, chip select is owned by the caller so a bus adapter can
    /// combine multiple operations in one transaction.
    pub fn write_qspi(
        &mut self,
        header: &[u8; 4],
        words: &[u8],
    ) -> Result<(), crate::error::Error> {
        self.inner
            .write_qspi(header, words)
            .map_err(|_| crate::error::code::EIO)
    }
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()>, G: blueos_hal::gpio::OutputPin> BusInterface
    for BlockSpi<T, G>
{
}
