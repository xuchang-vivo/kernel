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

//! Simple ES8311 audio codec driver.
//!
//! The ES8311 is controlled over I2C (address 0x18) and exchanges audio
//! data with the I2S peripheral. This driver handles only the I2C control
//! path — register configuration and verification. Audio data flows through
//! the I2S driver (`/dev/i2s0`).
//!
//! The `init()` method applies the register sequence for 16 kHz / 16-bit /
//! I2S-Philips / MCLK = 256×fs, mirroring the reference ESP-IDF
//! `es8311_open()` + `es8311_set_fs()` + `es8311_start()` sequence from the
//! Waveshare ESP32-C6 Touch AMOLED 2.16 audio test example. The `verify()`
//! method reads back a few registers to confirm the codec is present and
//! responding on the I2C bus.

use blueos_driver::i2c::I2cConfig;
use embedded_hal::i2c::I2c as HalI2c;

#[cfg(soc_esp32c6)]
use blueos_hal::pinctrl::AlterFuncPin;

use crate::devices::{
    bus::{Bus, BusWrapper},
    i2c_core::block_i2c::BlockI2c,
};

/// ES8311 I2C slave address (7-bit, shifted left by the controller).
const ES8311_I2C_ADDR: u8 = 0x18;

/// PA power-control flags mirroring the reference C driver's `es_pa_setting_t`.
const ES_PA_SETUP: u8 = 1;
const ES_PA_ENABLE: u8 = 2;
const ES_PA_DISABLE: u8 = 4;

/// Register-value pair for the ES8311 init sequence.
struct RegVal(u8, u8);

/// ES8311 codec driver — wraps the shared I2C bus and an optional PA control pin.
pub struct Es8311Driver<T, G>
where
    T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static,
    G: blueos_hal::gpio::OutputPin + Send + Sync + 'static,
{
    bus: BusWrapper<BlockI2c<T>>,
    pa_pin: Option<&'static G>,
    /// `false`: enable PA when pin is high; `true`: enable PA when pin is low.
    pa_reverted: bool,
}

impl<T, G> Es8311Driver<T, G>
where
    T: blueos_hal::i2c::I2c<I2cConfig, ()> + 'static,
    G: blueos_hal::gpio::OutputPin + Send + Sync + 'static,
{
    /// Create a new ES8311 driver from a reference to the I2C bus.
    ///
    /// `pa_pin` is `None` when no external PA control GPIO is used (mirrors the
    /// C driver's `pa_pin == -1` early-return). `pa_reverted` selects the PA
    /// enable polarity.
    pub fn new(bus: &Bus<BlockI2c<T>>, pa_pin: Option<&'static G>, pa_reverted: bool) -> Self {
        Self {
            bus: bus.intf.clone(),
            pa_pin,
            pa_reverted,
        }
    }

    /// Write a value to an ES8311 register over I2C.
    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), crate::error::Error> {
        self.bus
            .transaction(ES8311_I2C_ADDR, &mut [embedded_hal::i2c::Operation::Write(&[reg, val])])
    }

    /// Read a value from an ES8311 register over I2C.
    fn read_reg(&mut self, reg: u8) -> Result<u8, crate::error::Error> {
        let mut buf = [0u8; 1];
        self.bus.transaction(
            ES8311_I2C_ADDR,
            &mut [
                embedded_hal::i2c::Operation::Write(&[reg]),
                embedded_hal::i2c::Operation::Read(&mut buf),
            ],
        )?;
        Ok(buf[0])
    }

    /// Read-modify-write a register, applying `f` to the current value.
    fn rmw<F: Fn(u8) -> u8>(&mut self, reg: u8, f: F) -> Result<(), crate::error::Error> {
        let val = self.read_reg(reg)?;
        self.write_reg(reg, f(val))
    }

    /// Control the external PA power pin, mirroring the reference C driver's
    /// `es8311_pa_power()`. Does nothing when `pa_pin` is `None`.
    fn pa_power(&self, setting: u8) {
        let Some(pin) = self.pa_pin else {
            return;
        };
        if setting & ES_PA_ENABLE != 0 {
            let _ = if self.pa_reverted {
                pin.set_low()
            } else {
                pin.set_high()
            };
        }
        if setting & ES_PA_DISABLE != 0 {
            let _ = if self.pa_reverted {
                pin.set_high()
            } else {
                pin.set_low()
            };
        }
    }

    /// Apply the full ES8311 init sequence for 16 kHz / 16-bit / I2S-Philips.
    ///
    /// Mirrors the reference ESP-IDF `es8311_open()` + `es8311_set_fs()` +
    /// `es8311_start()` sequence from the Waveshare ESP32-C6 Touch AMOLED
    /// 2.16 audio test example.
    ///
    /// Configuration assumed (matching the board config):
    /// - `master_mode = false` (codec is I2S slave)
    /// - `use_mclk = true` (external MCLK from the I2S master)
    /// - `invert_mclk = false`
    /// - `invert_sclk = false`
    /// - `digital_mic = false`
    /// - `no_dac_ref = false`
    /// - `codec_mode = BOTH` (ADC + DAC)
    /// - `pa_pin = -1` (no external PA on this board)
    pub fn init(&mut self) -> Result<(), crate::error::Error> {
        // Configure I2C0 pins before any I2C transaction.
        // The Waveshare ESP32-C6 Touch AMOLED 2.16 board wires ES8311 to the
        // same I2C0 bus as CST9220 (GPIO8=SDA, GPIO7=SCL). When CST9220 is not
        // enabled, these pins are not configured by `define_pin_states!`, so
        // we configure them here to make the driver self-contained.
        // Signal 46=I2CEXT0_SDA, 45=I2CEXT0_SCL.
        #[cfg(soc_esp32c6)]
        {
            let sda = blueos_driver::pinctrl::esp32c6_pinctrl::Esp32c6IoMuxPinctrl::new(
                8, 1, true, true, false, 2, Some(46), Some(46), false, true,
            );
            let scl = blueos_driver::pinctrl::esp32c6_pinctrl::Esp32c6IoMuxPinctrl::new(
                7, 1, true, true, false, 2, Some(45), Some(45), false, true,
            );
            sda.init();
            scl.init();
        }

        // ============================================================
        // Phase 1: es8311_open()  (es8311.c lines 496–557)
        // ============================================================

        // Enhance ES8311 I2C noise immunity.
        self.write_reg(0x44, 0x08)?;
        // Due to occasional failures during the first I2C write with the
        // ES8311 chip, a second write is performed to ensure reliability.
        self.write_reg(0x44, 0x08)?;

        self.write_reg(0x01, 0x30)?;
        self.write_reg(0x02, 0x00)?;
        self.write_reg(0x03, 0x10)?;
        self.write_reg(0x16, 0x24)?;
        self.write_reg(0x04, 0x10)?;
        self.write_reg(0x05, 0x00)?;
        self.write_reg(0x0B, 0x00)?;
        self.write_reg(0x0C, 0x00)?;
        self.write_reg(0x10, 0x1F)?;
        self.write_reg(0x11, 0x7F)?;
        self.write_reg(0x00, 0x80)?;

        // master_mode = false → slave mode: clear bit6.
        self.rmw(0x00, |v| v & 0xBF)?;

        // Select clock source for internal mclk.
        // use_mclk = true → regv &= 0x7F; invert_mclk = false → regv &= ~0x40.
        // Start from 0x3F, apply both: 0x3F & 0x7F & !0x40 = 0x3F.
        self.write_reg(0x01, 0x3F)?;

        // SCLK inverted or not: invert_sclk = false → clear bit5.
        self.rmw(0x06, |v| v & !0x20)?;

        self.write_reg(0x13, 0x10)?;
        self.write_reg(0x1B, 0x0A)?;
        self.write_reg(0x1C, 0x6A)?;

        // no_dac_ref = false → set internal reference signal (ADCL + DACR).
        self.write_reg(0x44, 0x58)?;

        // es8311_pa_power(ES_PA_SETUP | ES_PA_ENABLE)
        self.pa_power(ES_PA_SETUP | ES_PA_ENABLE);

        // ============================================================
        // Phase 2: es8311_set_fs(16-bit, 16kHz)
        //          = set_bits_per_sample(16) + config_fmt(NORMAL)
        //            + config_sample(16000)
        //          (es8311.c lines 205–230, 168–203, 409–479)
        // ============================================================

        // --- es8311_set_bits_per_sample(16) ---
        // 16-bit: dac_iface |= 0x0c; adc_iface |= 0x0c.
        self.rmw(0x09, |v| v | 0x0c)?;
        self.rmw(0x0A, |v| v | 0x0c)?;

        // --- es8311_config_fmt(ES_I2S_NORMAL) ---
        // I2S format: dac_iface &= 0xFC; adc_iface &= 0xFC.
        self.rmw(0x09, |v| v & 0xFC)?;
        self.rmw(0x0A, |v| v & 0xFC)?;

        // --- es8311_config_sample(16000) ---
        // coeff_div entry for mclk=4096000, rate=16000:
        //   pre_div=1, pre_multi=1, adc_div=1, dac_div=1, fs_mode=0,
        //   lrck_h=0, lrck_l=0xFF, bclk_div=4, adc_osr=0x10, dac_osr=0x20.

        // REG02: regv &= 0x7; regv |= (pre_div-1)<<5 = 0;
        //        datmp=0 (pre_multi=1); regv |= datmp<<3 = 0.
        self.rmw(0x02, |v| (v & 0x07))?;

        // REG05: regv = (adc_div-1)<<4 | (dac_div-1) = 0x00.
        self.write_reg(0x05, 0x00)?;

        // REG03: regv &= 0x80; regv |= fs_mode<<6 = 0; regv |= adc_osr = 0x10.
        self.rmw(0x03, |v| (v & 0x80) | 0x10)?;

        // REG04: regv &= 0x80; regv |= dac_osr = 0x20.
        self.rmw(0x04, |v| (v & 0x80) | 0x20)?;

        // REG07: regv &= 0xC0; regv |= lrck_h = 0.
        self.rmw(0x07, |v| v & 0xC0)?;

        // REG08: regv = lrck_l = 0xFF.
        self.write_reg(0x08, 0xFF)?;

        // REG06: regv &= 0xE0; bclk_div=4 < 19 → regv |= (bclk_div-1) = 0x03.
        self.rmw(0x06, |v| (v & 0xE0) | 0x03)?;

        // ============================================================
        // Phase 3: es8311_start()  (es8311.c lines 261–327, codec_mode=BOTH)
        // ============================================================

        // regv = 0x80; master_mode=false → regv &= 0xBF → 0x80.
        self.write_reg(0x00, 0x80)?;

        // regv = 0x3F; use_mclk=true → &= 0x7F; invert_mclk=false → &= ~0x40.
        self.write_reg(0x01, 0x3F)?;

        // Read SDPIN/SDPOUT, apply mute-then-unmute-per-mode logic.
        // For BOTH mode: both adc_iface and dac_iface end with bit6 cleared.
        let mut dac_iface = self.read_reg(0x09)?;
        let mut adc_iface = self.read_reg(0x0A)?;
        dac_iface &= 0xBF;
        adc_iface &= 0xBF;
        adc_iface |= 0x40;
        dac_iface |= 0x40;
        // codec_mode == BOTH → clear bit6 on both.
        adc_iface &= !0x40;
        dac_iface &= !0x40;
        self.write_reg(0x09, dac_iface)?;
        self.write_reg(0x0A, adc_iface)?;

        self.write_reg(0x17, 0xBF)?;
        self.write_reg(0x0E, 0x02)?;
        self.write_reg(0x12, 0x00)?;
        self.write_reg(0x14, 0x1A)?;

        // digital_mic = false → clear bit6 on REG14.
        self.rmw(0x14, |v| v & !0x40)?;

        self.write_reg(0x0D, 0x01)?;
        self.write_reg(0x15, 0x40)?;
        self.write_reg(0x37, 0x08)?;
        self.write_reg(0x45, 0x00)?;

        // es8311_pa_power(ES_PA_ENABLE)
        self.pa_power(ES_PA_ENABLE);

        // ============================================================
        // Phase 4: Unmute & set volume (kept from original code).
        //          Equivalent to C driver's set_mute(false) + set_vol(max).
        // ============================================================
        self.write_reg(0x31, 0x00)?; // unmute DAC
        self.write_reg(0x32, 0xA0)?; // volume = max

        Ok(())
    }

    /// Verify the codec is present by reading back a few registers and
    /// checking they match the values written during `init()`.
    ///
    /// Returns `Ok(())` if all checks pass, `Err(EIO)` otherwise.
    pub fn verify(&mut self) -> Result<(), crate::error::Error> {
        // Registers written during init that should retain their values.
        let checks: [RegVal; 4] = [
            RegVal(0x09, 0x0C), // DAC SDP: I2S, 16-bit, unmuted
            RegVal(0x0A, 0x0C), // ADC SDP: I2S, 16-bit, unmuted
            RegVal(0x31, 0x00), // unmute DAC
            RegVal(0x32, 0xA0), // volume = max
        ];

        for RegVal(reg, expected) in checks.iter() {
            match self.read_reg(*reg) {
                Ok(actual) => {
                    if actual != *expected {
                        log::warn!(
                            "[ES8311] verify failed: reg 0x{:02X} = 0x{:02X}, expected 0x{:02X}",
                            reg,
                            actual,
                            expected
                        );
                        return Err(crate::error::code::EIO);
                    }
                    log::info!(
                        "[ES8311] reg 0x{:02X} = 0x{:02X} (OK)",
                        reg,
                        actual
                    );
                }
                Err(e) => {
                    log::warn!("[ES8311] verify failed: cannot read reg 0x{:02X}: {:?}", reg, e);
                    return Err(e);
                }
            }
        }

        log::info!("[ES8311] codec verified: all register checks passed");
        Ok(())
    }
}
