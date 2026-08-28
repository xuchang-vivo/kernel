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

//! ESP32-C6 IO_MUX and GPIO Matrix pin controller.

use crate::gpio::esp32c6_gpio::{GPIO_BASE, GpioEnable, GpioOut};
use blueos_hal::pinctrl::AlterFuncPin;
use tock_registers::{
    interfaces::{ReadWriteable, Writeable},
    register_bitfields,
    registers::ReadWrite,
};

const IO_MUX_BASE: usize = 0x6009_0000;
const GPIO_MATRIX_BASE: usize = 0x6009_1000;
const GPIO_FUNC_IN_SEL_CFG_OFFSET: usize = 0x154;
const GPIO_FUNC_OUT_SEL_CFG_OFFSET: usize = 0x554;

register_bitfields! [
    u32,

    pub IoMuxFields [
        MCU_SEL OFFSET(12) NUMBITS(3) [],
        FUN_DRV OFFSET(10) NUMBITS(2) [],
        FUN_IE OFFSET(9) NUMBITS(1) [],
        FUN_WPU OFFSET(8) NUMBITS(1) [],
        FUN_WPD OFFSET(7) NUMBITS(1) [],
    ],

    pub GpioPinFields [
        PAD_DRIVER OFFSET(2) NUMBITS(1) [],
    ],

    pub FuncOutSelCfg [
        OUT_SEL OFFSET(0) NUMBITS(8) [],
        INV_SEL OFFSET(8) NUMBITS(1) [],
        OEN_SEL OFFSET(9) NUMBITS(1) [],
        OEN_INV_SEL OFFSET(10) NUMBITS(1) [],
    ],

    pub FuncInSelCfg [
        IN_SEL OFFSET(0) NUMBITS(6) [],
        IN_INV_SEL OFFSET(6) NUMBITS(1) [],
        SEL OFFSET(7) NUMBITS(1) [],
    ],
];

fn write_io_mux(pin: u8, mcu_sel: u32, ie: bool, pu: bool, pd: bool, drv: u32) {
    // The C6 IO_MUX GPIO registers are contiguous: GPIO0 starts at +0x04.
    let addr = IO_MUX_BASE + 0x04 + 4 * pin as usize;
    let reg = unsafe { &*(addr as *const ReadWrite<u32, IoMuxFields::Register>) };
    reg.write(
        IoMuxFields::MCU_SEL.val(mcu_sel)
            + IoMuxFields::FUN_IE.val(ie as u32)
            + IoMuxFields::FUN_WPU.val(pu as u32)
            + IoMuxFields::FUN_WPD.val(pd as u32)
            + IoMuxFields::FUN_DRV.val(drv),
    );
}

fn configure_open_drain(pin: u8, open_drain: bool) {
    let addr = GPIO_MATRIX_BASE + 0x74 + 4 * pin as usize;
    let reg = unsafe { &*(addr as *const ReadWrite<u32, GpioPinFields::Register>) };
    reg.modify(GpioPinFields::PAD_DRIVER.val(open_drain as u32));
}

fn route_signal_out(pin: u8, signal: u32, gpio_controls_output_enable: bool) {
    // GPIO_FUNC0_OUT_SEL_CFG starts at GPIO + 0x554. The register index is
    // the GPIO number, not the peripheral signal number.
    let addr = GPIO_MATRIX_BASE + GPIO_FUNC_OUT_SEL_CFG_OFFSET + 4 * pin as usize;
    let reg = unsafe { &*(addr as *const ReadWrite<u32, FuncOutSelCfg::Register>) };
    reg.write(
        FuncOutSelCfg::OUT_SEL.val(signal)
            + FuncOutSelCfg::INV_SEL.val(0)
            + FuncOutSelCfg::OEN_SEL.val(gpio_controls_output_enable as u32)
            + FuncOutSelCfg::OEN_INV_SEL.val(0),
    );
}

fn route_signal_in(signal: u32, pin: u8) {
    // GPIO_FUNC0_IN_SEL_CFG starts at GPIO + 0x154. The register index is
    // the peripheral input signal number.
    let addr = GPIO_MATRIX_BASE + GPIO_FUNC_IN_SEL_CFG_OFFSET + 4 * signal as usize;
    let reg = unsafe { &*(addr as *const ReadWrite<u32, FuncInSelCfg::Register>) };
    reg.write(
        FuncInSelCfg::IN_SEL.val(pin as u32)
            + FuncInSelCfg::IN_INV_SEL.val(0)
            + FuncInSelCfg::SEL.val(1),
    );
}

/// Pin configuration entry used by `define_pin_states!`.
///
/// GPIO Matrix mode uses IO_MUX selector value 1 and allows peripheral signals
/// to be routed to the board-specific pins supplied by `define_pin_states!`.
pub struct Esp32c6IoMuxPinctrl {
    pin: u8,
    mcu_sel: u32,
    ie: bool,
    pu: bool,
    pd: bool,
    drv: u32,
    out_signal: Option<u32>,
    in_signal: Option<u32>,
    gpio_output: bool,
    open_drain: bool,
}

impl Esp32c6IoMuxPinctrl {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        pin: u8,
        mcu_sel: u32,
        ie: bool,
        pu: bool,
        pd: bool,
        drv: u32,
        out_signal: Option<u32>,
        in_signal: Option<u32>,
        gpio_output: bool,
        open_drain: bool,
    ) -> Self {
        assert!(pin <= 30, "ESP32-C6 GPIO number is out of range");
        assert!(mcu_sel <= 7, "invalid ESP32-C6 IO_MUX function");
        assert!(drv <= 3, "invalid ESP32-C6 drive strength");
        Self {
            pin,
            mcu_sel,
            ie,
            pu,
            pd,
            drv,
            out_signal,
            in_signal,
            gpio_output,
            open_drain,
        }
    }
}

impl AlterFuncPin for Esp32c6IoMuxPinctrl {
    fn init(&self) {
        // A software pin must already be high before output enable is asserted,
        // otherwise active-low LCD CS/RST can glitch during boot.
        if self.gpio_output {
            let regs = &*GPIO_BASE;
            regs.out_w1ts.write(GpioOut::DATA.val(1u32 << self.pin));
        }

        write_io_mux(self.pin, self.mcu_sel, self.ie, self.pu, self.pd, self.drv);
        configure_open_drain(self.pin, self.open_drain);

        if let Some(signal) = self.out_signal {
            route_signal_out(self.pin, signal, self.gpio_output);
        }
        if let Some(signal) = self.in_signal {
            route_signal_in(signal, self.pin);
        }

        if self.gpio_output {
            let regs = &*GPIO_BASE;
            regs.enable_w1ts
                .write(GpioEnable::DATA.val(1u32 << self.pin));
        }
    }
}
