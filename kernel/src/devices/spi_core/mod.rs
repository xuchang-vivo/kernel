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

use crate::{
    devices::{bus::BusWrapper, spi_core::block_spi::BlockSpi},
    sync::KernelDelay,
};
use blueos_driver::spi::SpiConfig;
use embedded_hal::spi::Operation;
pub mod block_spi;

pub struct ExclusiveSpiWithCs<
    T: blueos_hal::spi::Spi<SpiConfig, ()>,
    G: blueos_hal::gpio::OutputPin,
> {
    spi: BusWrapper<BlockSpi<T, G>>,
    cs: &'static G,
}

impl<T: blueos_hal::spi::Spi<SpiConfig, ()>, G: blueos_hal::gpio::OutputPin>
    ExclusiveSpiWithCs<T, G>
{
    pub fn new(spi: BusWrapper<BlockSpi<T, G>>, cs: &'static G) -> Self {
        ExclusiveSpiWithCs { spi, cs }
    }

    fn assert_cs(&self) {
        self.cs.set_low().ok();
    }

    fn deassert_cs(&self) {
        self.cs.set_high().ok();
    }

    /// Execute operations while keeping chip select asserted for the complete
    /// callback.  This is needed by SD SPI data phases, where the card keeps
    /// a read token or write-busy state across multiple transfers.
    pub fn with_cs<R, E>(
        &mut self,
        f: impl FnOnce(&mut BlockSpi<T, G>) -> Result<R, E>,
    ) -> Result<R, E> {
        let mut inner = self.spi.0.lock();
        self.assert_cs();
        let result = f(&mut inner);
        self.deassert_cs();
        result
    }

    /// Reconfigure the bus and execute one transaction with chip select held
    /// low.  Configuration is performed while the bus lock is held and before
    /// chip select is asserted, so shared SPI devices cannot observe a partial
    /// speed change.
    pub fn with_cs_config<R, E>(
        &mut self,
        config: &SpiConfig,
        f: impl FnOnce(&mut BlockSpi<T, G>) -> Result<R, E>,
    ) -> Result<R, E>
    where
        E: From<crate::error::Error>,
    {
        let mut inner = self.spi.0.lock();
        inner
            .configure(config)
            .map_err(|_| E::from(crate::error::code::EIO))?;
        self.assert_cs();
        let result = f(&mut inner);
        self.deassert_cs();
        result
    }

    /// Clock the bus with chip select deasserted. SD cards require at least
    /// 74 idle clocks before the first command during card identification.
    pub fn clock_idle(&mut self, bytes: &[u8]) -> Result<(), crate::error::Error> {
        let mut inner = self.spi.0.lock();
        self.deassert_cs();
        inner.clock_idle(bytes)
    }
}

#[cfg(use_embedded_hal_v1)]
impl embedded_hal::spi::Error for crate::error::Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        // FIXME: Map the error code to embedded_hal::spi::ErrorKind
        embedded_hal::spi::ErrorKind::Other
    }
}

#[cfg(use_embedded_hal_v1)]
impl<T: blueos_hal::spi::Spi<SpiConfig, ()>, G: blueos_hal::gpio::OutputPin>
    embedded_hal::spi::ErrorType for ExclusiveSpiWithCs<T, G>
{
    type Error = crate::error::Error;
}

#[cfg(use_embedded_hal_v1)]
impl<T: blueos_hal::spi::Spi<SpiConfig, ()>, G: blueos_hal::gpio::OutputPin>
    embedded_hal::spi::SpiDevice<u8> for ExclusiveSpiWithCs<T, G>
{
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        let mut inner = self.spi.0.lock();
        self.assert_cs();

        let op_res = operations.iter_mut().try_for_each(|op| match op {
            Operation::Read(buf) => inner.read(buf),
            Operation::Write(buf) => inner.write(buf),
            Operation::Transfer(read, write) => inner.transfer(read, write),
            Operation::TransferInPlace(buf) => inner.transfer_in_place(buf),
            Operation::DelayNs(ns) => {
                use embedded_hal::delay::DelayNs;
                let mut kernel = KernelDelay;
                kernel.delay_ns(*ns);
                Ok(())
            }
        });

        self.deassert_cs();
        op_res
    }
}
