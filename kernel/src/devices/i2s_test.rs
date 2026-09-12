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

//! I2S loopback test device.
//!
//! Exposes `/dev/i2s_test` so userspace can verify the I2S TX/RX data path
//! via GPIO-matrix loopback (DOUT and DIN routed to the same GPIO pin).
//!
//! On `write()`, the device calls `Esp32c6I2s0::loopback_transfer()` which
//! starts TX and RX DMA simultaneously. The written data loops back through
//! the GPIO matrix into the RX path, and the kernel compares the received
//! bytes against the written pattern.

use crate::devices::{Device, DeviceClass, DeviceId, DeviceManager};
use alloc::{string::String, sync::Arc, vec};
use blueos_driver::i2s::esp32c6_i2s::Esp32c6I2s0;
use blueos_hal::{Configuration, PlatPeri};
use blueos_hal::i2s::I2sConfig;
use embedded_io::ErrorKind;

/// I2S loopback test device.
///
/// Wraps the real `Esp32c6I2s0` driver and provides a `/dev/i2s_test` node
/// whose `write()` triggers a simultaneous TX/RX transfer. The GPIO matrix
/// routes DOUT back to DIN on the same pin, so the written data appears in
/// the RX buffer for verification.
pub struct I2sTestDevice {
    driver: &'static Esp32c6I2s0<0, 1>,
    configured: core::sync::atomic::AtomicBool,
}

impl I2sTestDevice {
    pub fn new(driver: &'static Esp32c6I2s0<0, 1>) -> Self {
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
        log::info!("[I2S_TEST] ensure_configured: calling driver.enable()");
        self.driver.enable();
        log::info!("[I2S_TEST] ensure_configured: driver.enable() done");
        match self.driver.configure(&I2sConfig::default_16k()) {
            Ok(()) => log::info!("[I2S_TEST] ensure_configured: driver.configure() OK"),
            Err(e) => {
                log::warn!("[I2S_TEST] ensure_configured: driver.configure() FAILED: {:?}", e);
                return Err(ErrorKind::InvalidData);
            }
        }
        self.configured
            .store(true, core::sync::atomic::Ordering::Relaxed);
        Ok(())
    }

}

impl Device for I2sTestDevice {
    fn name(&self) -> String {
        String::from("i2s_test")
    }

    fn class(&self) -> DeviceClass {
        DeviceClass::Char
    }

    fn id(&self) -> DeviceId {
        // Major 1 (char), minor 12 — after gdma_test (11).
        DeviceId::new(1, 12)
    }

    fn open(&self) -> Result<(), ErrorKind> {
        self.ensure_configured()
    }

    /// Write a pattern to trigger loopback transfer.
    ///
    /// Calls `loopback_transfer(tx_buf, rx_buf)` which simultaneously starts
    /// TX and RX DMA. The written data loops back through the GPIO matrix
    /// (DOUT→DIN on the same pin). The received bytes are compared against
    /// the written pattern and the result is logged.
    fn write(&self, _pos: u64, buf: &[u8], _is_nonblocking: bool) -> Result<usize, ErrorKind> {
        log::info!("[I2S_TEST] write() called, buf.len()={}", buf.len());
        self.ensure_configured()?;
        log::info!("[I2S_TEST] ensure_configured() OK");

        let mut rx_buf = vec![0u8; buf.len()];
        match self.driver.loopback_transfer(buf, &mut rx_buf) {
            Ok(()) => log::info!("[I2S_TEST] loopback_transfer() OK"),
            Err(e) => {
                log::warn!("[I2S_TEST] loopback_transfer() FAILED: {:?}", e);
                return Err(ErrorKind::Other);
            }
        }

        if rx_buf == buf {
            log::info!(
                "[I2S_TEST] loopback PASSED: {} bytes matched",
                buf.len()
            );
        } else {
            let mismatches = rx_buf
                .iter()
                .zip(buf.iter())
                .filter(|(a, b)| a != b)
                .count();
            let first = rx_buf
                .iter()
                .zip(buf.iter())
                .position(|(a, b)| a != b);
            log::warn!(
                "[I2S_TEST] loopback FAILED: {}/{} bytes mismatch, first at offset {:?}",
                mismatches,
                buf.len(),
                first
            );
        }

        Ok(buf.len())
    }

    /// Read is not supported — loopback verification happens in `write()`.
    fn read(&self, _pos: u64, _buf: &mut [u8], _is_nonblocking: bool) -> Result<usize, ErrorKind> {
        Err(ErrorKind::Unsupported)
    }
}
