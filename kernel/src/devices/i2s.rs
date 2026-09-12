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

//! I2S character device wrapper.
//!
//! Bridges the `blueos_hal::i2s::I2s` HAL trait to the kernel `Device` trait,
//! exposing `/dev/i2s0` for userspace write (playback) and read (capture).

use crate::devices::{Device, DeviceClass, DeviceId, DeviceManager};
use alloc::{string::String, sync::Arc};
use blueos_hal::i2s::{I2s, I2sConfig};
use blueos_hal::{Configuration, PlatPeri};
use embedded_io::ErrorKind;

pub struct I2sDevice<D: 'static> {
    driver: &'static D,
    configured: core::sync::atomic::AtomicBool,
}

impl<D> I2sDevice<D>
where
    D: I2s<I2sConfig, ()> + PlatPeri + Configuration<I2sConfig, Target = ()> + Send + Sync,
{
    pub fn new(driver: &'static D) -> Self {
        Self {
            driver,
            configured: core::sync::atomic::AtomicBool::new(false),
        }
    }

    pub fn register(self, name: &str) -> Result<(), ErrorKind> {
        let device = Arc::new(self);
        DeviceManager::get().register_device(String::from(name), device)
    }

    fn ensure_configured(&self) -> Result<(), ErrorKind> {
        if self
            .configured
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            return Ok(());
        }
        self.driver.enable();
        self.driver
            .configure(&I2sConfig::default_16k())
            .map_err(|_| ErrorKind::InvalidData)?;
        self.configured
            .store(true, core::sync::atomic::Ordering::Relaxed);
        Ok(())
    }
}

impl<D> Device for I2sDevice<D>
where
    D: I2s<I2sConfig, ()> + PlatPeri + Configuration<I2sConfig, Target = ()> + Send + Sync,
{
    fn name(&self) -> String {
        String::from("i2s0")
    }

    fn class(&self) -> DeviceClass {
        DeviceClass::Char
    }

    fn id(&self) -> DeviceId {
        DeviceId::new(1, 10)
    }

    fn open(&self) -> Result<(), ErrorKind> {
        self.ensure_configured()
    }

    fn read(&self, _pos: u64, buf: &mut [u8], _is_nonblocking: bool) -> Result<usize, ErrorKind> {
        self.ensure_configured()?;
        self.driver
            .read(buf)
            .map(|()| buf.len())
            .map_err(|_| ErrorKind::Other)
    }

    fn write(&self, _pos: u64, buf: &[u8], _is_nonblocking: bool) -> Result<usize, ErrorKind> {
        self.ensure_configured()?;
        self.driver
            .write(buf)
            .map(|()| buf.len())
            .map_err(|_| ErrorKind::Other)
    }

    fn close(&self) -> Result<(), ErrorKind> {
        // Drain any in-flight TX data and stop the DMA ring so that the
        // last audio segment is fully played out before the device closes.
        // This is triggered by the close() syscall when userspace drops the
        // file (e.g. std::fs::File::drop → close(fd) → device.close()).
        let _ = self.driver.drain_and_stop();
        Ok(())
    }
}
