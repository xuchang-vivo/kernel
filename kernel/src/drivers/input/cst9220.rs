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

//! CST9220 touch controller integration.
//!
//! The Waveshare ESP32-C6 Touch AMOLED board connects the CST9220 at I2C
//! address 0x5a.  The protocol and report decoding live in the `cst92xx`
//! crate; this module only adapts that driver to BlueOS's I2C bus and
//! character-device interfaces.

use blueos_driver::i2c::I2cConfig;
use cst92xx::{CST92xx, Orientation, Point, TouchConfig};
use embedded_io::ErrorKind;

use crate::{
    devices::{
        Device, DeviceClass, DeviceData, DeviceId, DeviceManager,
        bus::{Bus, BusWrapper},
        i2c_core::block_i2c::BlockI2c,
    },
    drivers::{DriverModule, InitDriver, Result as DriverResult},
    sync::{KernelDelay, SpinLock},
};
use alloc::{string::String, sync::Arc};

const CST9220_DEVICE_NAME: &str = "cst9220";
const CST9220_DEVICE_MAJOR: usize = 240;
const CST9220_DEVICE_MINOR: usize = 1;
const CST9220_REPORT_VERSION: u8 = 1;

/// Binary report returned by `/dev/cst9220`.
///
/// The layout intentionally matches the existing FT6336U device ABI:
/// byte 0 is the format version, byte 1 is the active touch count, and two
/// five-byte point records follow (`status`, `x` little-endian, `y`
/// little-endian). Status values are 0 = release, 1 = new touch, and 2 =
/// continuing touch.
pub const CST9220_REPORT_SIZE: usize = 12;

/// Adapter from BlueOS's shared-reference GPIO output trait to embedded-hal's
/// mutable `OutputPin` required by `cst92xx`.
struct CstResetPin<G: blueos_hal::gpio::OutputPin + Send + Sync + 'static> {
    pin: &'static G,
}

impl<G: blueos_hal::gpio::OutputPin + Send + Sync + 'static> embedded_hal::digital::ErrorType
    for CstResetPin<G>
{
    type Error = crate::error::Error;
}

impl<G: blueos_hal::gpio::OutputPin + Send + Sync + 'static> embedded_hal::digital::OutputPin
    for CstResetPin<G>
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low().map_err(|_| crate::error::code::EIO)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high().map_err(|_| crate::error::code::EIO)
    }
}

struct Cst9220State<T, G>
where
    T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static,
    G: blueos_hal::gpio::OutputPin + Send + Sync + 'static,
{
    touch: CST92xx<BusWrapper<BlockI2c<T>>, KernelDelay, CstResetPin<G>>,
    previous: [Option<Point>; 2],
}

pub struct Cst9220Device<T, G>
where
    T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static,
    G: blueos_hal::gpio::OutputPin + Send + Sync + 'static,
{
    state: SpinLock<Cst9220State<T, G>>,
}

impl<T, G> Cst9220Device<T, G>
where
    T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static,
    G: blueos_hal::gpio::OutputPin + Send + Sync + 'static,
{
    fn new(touch: CST92xx<BusWrapper<BlockI2c<T>>, KernelDelay, CstResetPin<G>>) -> Self {
        Self {
            state: SpinLock::new(Cst9220State {
                touch,
                previous: [None; 2],
            }),
        }
    }

    fn encode_report(
        state: &mut Cst9220State<T, G>,
        points: [Option<Point>; 2],
        report: &mut [u8; CST9220_REPORT_SIZE],
    ) {
        report[0] = CST9220_REPORT_VERSION;
        report[1] = points.iter().filter(|point| point.is_some()).count() as u8;

        for index in 0..2 {
            let offset = 2 + index * 5;
            match points[index] {
                Some(point) => {
                    let continuing = state
                        .previous
                        .iter()
                        .flatten()
                        .any(|old| old.track_id == point.track_id);
                    report[offset] = if continuing { 2 } else { 1 };
                    report[offset + 1..offset + 3].copy_from_slice(&point.x.to_le_bytes());
                    report[offset + 3..offset + 5].copy_from_slice(&point.y.to_le_bytes());
                    state.previous[index] = Some(point);
                }
                None => {
                    // Preserve the last coordinates in a release record. This
                    // lets consumers handle a finger-up event without a
                    // separate state query.
                    if let Some(old) = state.previous[index].take() {
                        report[offset] = 0;
                        report[offset + 1..offset + 3].copy_from_slice(&old.x.to_le_bytes());
                        report[offset + 3..offset + 5].copy_from_slice(&old.y.to_le_bytes());
                    }
                }
            }
        }
    }
}

impl<T, G> Device for Cst9220Device<T, G>
where
    T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static,
    G: blueos_hal::gpio::OutputPin + Send + Sync + 'static,
{
    fn name(&self) -> String {
        String::from(CST9220_DEVICE_NAME)
    }

    fn class(&self) -> DeviceClass {
        DeviceClass::Char
    }

    fn id(&self) -> DeviceId {
        DeviceId::new(CST9220_DEVICE_MAJOR, CST9220_DEVICE_MINOR)
    }

    fn read(
        &self,
        _pos: u64,
        buf: &mut [u8],
        _is_nonblocking: bool,
    ) -> core::result::Result<usize, ErrorKind> {
        if buf.len() < CST9220_REPORT_SIZE {
            return Err(ErrorKind::InvalidInput);
        }

        let mut state = self.state.lock();
        let points = state.touch.touches().map_err(|error| {
            log::warn!("Failed to scan CST9220 touch data: {:?}", error);
            ErrorKind::Other
        })?;
        let mut report = [0u8; CST9220_REPORT_SIZE];
        Self::encode_report(&mut state, points, &mut report);
        buf[..CST9220_REPORT_SIZE].copy_from_slice(&report);
        Ok(CST9220_REPORT_SIZE)
    }

    fn write(
        &self,
        _pos: u64,
        _buf: &[u8],
        _is_nonblocking: bool,
    ) -> core::result::Result<usize, ErrorKind> {
        Err(ErrorKind::Unsupported)
    }
}

pub struct Cst9220Config<G: blueos_hal::gpio::OutputPin + Send + Sync + 'static> {
    pub rst: &'static G,
}

impl<T, G> InitDriver<BlockI2c<T>> for Cst9220Config<G>
where
    T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static,
    G: blueos_hal::gpio::OutputPin + Send + Sync + 'static,
{
    type Data = ();

    fn init(self, bus: &Bus<BlockI2c<T>>) -> DriverResult<Self::Data> {
        // The panel and controller both report 480x480. Explicitly setting
        // the target resolution also clamps malformed samples to panel bounds.
        // The panel is mounted with the touch axes swapped. The X direction is
        // mirrored relative to the display, while Y already has the correct
        // direction. Keep the device ABI in display coordinates so
        // applications do not apply a second transform.
        let config = TouchConfig {
            orientation: Orientation {
                swap_xy: true,
                mirror_x: true,
                ..Orientation::default()
            },
            ..TouchConfig::default().with_target_resolution(480, 480)
        };
        let mut touch = CST92xx::new(bus.intf.clone(), KernelDelay)
            .with_reset(CstResetPin { pin: self.rst })
            .with_config(config);

        touch.init().map_err(|error| {
            // This runs before the regular logger is guaranteed to be up.  Keep
            // the concrete cst92xx error visible on the board console so an
            // I2C NACK/timeout can be distinguished from an attribute
            // validation failure (the driver API below still exposes EIO to
            // the generic kernel init path).
            crate::kearly_println!("CST9220 init detail: {:?}", error);
            log::warn!("Failed to initialize CST9220: {:?}", error);
            crate::error::code::EIO
        })?;

        let info = touch.chip_info();
        if info.chip_type != cst92xx::registers::CST9220_CHIP_ID
            && info.chip_type != cst92xx::registers::CST9217_CHIP_ID
        {
            log::warn!(
                "Unexpected touch controller ID: 0x{:04X} (expected CST9217/CST9220)",
                info.chip_type
            );
            return Err(crate::error::code::ENODEV);
        }

        log::info!(
            "{} initialized at 0x5A, resolution {}x{}",
            info.model_name(),
            info.resolution_x,
            info.resolution_y
        );
        let device = Arc::new(Cst9220Device::<T, G>::new(touch));
        DeviceManager::get()
            .register_device(String::from(CST9220_DEVICE_NAME), device)
            .map_err(|_| crate::error::code::EIO)?;
        Ok(())
    }
}

pub struct Cst9220DriverModule<G> {
    _marker: core::marker::PhantomData<G>,
}

impl<G> Cst9220DriverModule<G> {
    pub const fn new() -> Self {
        Self {
            _marker: core::marker::PhantomData,
        }
    }
}

impl<T, G> DriverModule<BlockI2c<T>> for Cst9220DriverModule<G>
where
    T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static,
    G: blueos_hal::gpio::OutputPin + Send + Sync + 'static,
{
    type Data = Cst9220Config<G>;

    fn probe(dev: &DeviceData) -> DriverResult<Self::Data> {
        match dev {
            DeviceData::Native(native_dev) => {
                if native_dev.is_attached() {
                    return Err(crate::error::code::ENODEV);
                }

                if let Some(config) = native_dev.config::<Cst9220Config<G>>() {
                    Ok(Cst9220Config { rst: config.rst })
                } else {
                    Err(crate::error::code::ENODEV)
                }
            }
            _ => Err(crate::error::code::ENODEV),
        }
    }
}
