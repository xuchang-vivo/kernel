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

//! ESP32-C6 GPIO output pin driver.
//!
//! Pin function and electrical configuration are handled by
//! `pinctrl::esp32c6_pinctrl`; this type only updates the output latch.

use crate::static_ref::StaticRef;
use tock_registers::{
    interfaces::{Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite},
};

pub(crate) const GPIO_BASE: StaticRef<GpioRegisters> =
    unsafe { StaticRef::new(0x6009_1000 as *const GpioRegisters) };

register_bitfields! [
    u32,

    pub GpioOut [
        DATA OFFSET(0) NUMBITS(31) [],
    ],
    pub GpioEnable [
        DATA OFFSET(0) NUMBITS(31) [],
    ],
    pub GpioIn [
        DATA OFFSET(0) NUMBITS(31) [],
    ],
];

register_structs! {
    pub GpioRegisters {
        (0x000 => pub bt_select: ReadWrite<u32>),
        (0x004 => pub out: ReadWrite<u32, GpioOut::Register>),
        (0x008 => pub out_w1ts: ReadWrite<u32, GpioOut::Register>),
        (0x00c => pub out_w1tc: ReadWrite<u32, GpioOut::Register>),
        (0x010 => _reserved0),
        (0x020 => pub enable: ReadWrite<u32, GpioEnable::Register>),
        (0x024 => pub enable_w1ts: ReadWrite<u32, GpioEnable::Register>),
        (0x028 => pub enable_w1tc: ReadWrite<u32, GpioEnable::Register>),
        (0x02c => _reserved1),
        (0x03c => pub input: ReadOnly<u32, GpioIn::Register>),
        (0x040 => @END),
    }
}

/// Software-controlled output pin on ESP32-C6.
pub struct Esp32c6GpioOutputPin {
    pin: u8,
}

impl Esp32c6GpioOutputPin {
    /// Create an output pin. ESP32-C6 exposes GPIO0 through GPIO30.
    pub const fn new(pin: u8) -> Self {
        assert!(pin <= 30, "ESP32-C6 GPIO number is out of range");
        Self { pin }
    }

    /// Sample the current pad level.
    pub fn is_high(&self) -> bool {
        GPIO_BASE.input.read(GpioIn::DATA) & (1u32 << self.pin) != 0
    }
}

impl blueos_hal::PlatPeri for Esp32c6GpioOutputPin {}

impl blueos_hal::gpio::OutputPin for Esp32c6GpioOutputPin {
    fn set_low(&self) -> blueos_hal::err::Result<()> {
        let regs = &*GPIO_BASE;
        regs.out_w1tc.write(GpioOut::DATA.val(1u32 << self.pin));
        Ok(())
    }

    fn set_high(&self) -> blueos_hal::err::Result<()> {
        let regs = &*GPIO_BASE;
        regs.out_w1ts.write(GpioOut::DATA.val(1u32 << self.pin));
        Ok(())
    }
}
