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

use tock_registers::register_bitfields;

register_bitfields! [u32,
    /// SYSTEM_PERIP_CLK_EN0 (0x0010): Peripheral Clock Enable Register 0
    /// Controls clock distribution to the first group of peripherals.
    /// Reset value: 0x0
    /// Access: R/W
    SYSTEM_PERIP_CLK_EN0 [
        /// Enable clock for TIMERS peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_TIMERS_CLK_EN OFFSET(0) NUMBITS(1),
        /// Enable clock for SPI0 and SPI1 peripherals. Write 1 to enable, 0 to disable.
        SYSTEM_SPI01_CLK_EN OFFSET(1) NUMBITS(1),
        /// Enable clock for UART peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_UART_CLK_EN OFFSET(2) NUMBITS(1),
        /// Enable clock for UART1 peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_UART1_CLK_EN OFFSET(5) NUMBITS(1),
        /// Enable clock for SPI2 peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_SPI2_CLK_EN OFFSET(6) NUMBITS(1),
        /// Enable clock for I2C_EXTO peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_EXTO_CLK_EN OFFSET(7) NUMBITS(1),
        /// Enable clock for UHCIO peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_UHCIO_CLK_EN OFFSET(8) NUMBITS(1),
        /// Enable clock for RMT (Remote Control) peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_RMT_CLK_EN OFFSET(9) NUMBITS(1),
        /// Enable clock for LEDC (LED PWM Controller) peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_LEDC_CLK_EN OFFSET(11) NUMBITS(1),
        /// Enable clock for TIMERGROUP peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_TIMERGROUP_CLK_EN OFFSET(13) NUMBITS(1),
        /// Enable clock for TIMERGROUP1 peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_TIMERGROUP1_CLK_EN OFFSET(15) NUMBITS(1),
        /// Enable clock for TWAI (Two-Wire Automotive Interface) / CAN peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_CAN_CLK_EN OFFSET(19) NUMBITS(1),
        /// Enable clock for I2S peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_I2SO_CLK_EN OFFSET(21) NUMBITS(1),
        /// Enable clock for USB DEVICE peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_USB_DEVICE_CLK_EN OFFSET(23) NUMBITS(1),
        /// Enable clock for UART_MEM peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_UART_MEM_CLK_EN OFFSET(24) NUMBITS(1),
        /// Enable clock for SPI3 DMA peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_SPI3_DMA_CLK_EN OFFSET(27) NUMBITS(1),
        /// Enable clock for APB_SARADC (ADC) peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_APB_SARADC_CLK_EN OFFSET(28) NUMBITS(1),
        /// Enable clock for SYSTIMER peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_SYSTIMER_CLK_EN OFFSET(29) NUMBITS(1),
        /// Enable clock for ADC2_ARB (ADC2 Arbiter) peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_ADC2_ARB_CLK_EN OFFSET(30) NUMBITS(1),
    ],

    /// SYSTEM_PERIP_CLK_EN1 (0x0014): Peripheral Clock Enable Register 1
    /// Controls clock distribution to the second group of peripherals (primarily cryptographic engines).
    /// Reset value: 0x0
    /// Access: R/W
    SYSTEM_PERIP_CLK_EN1 [
        /// Enable clock for AES (Advanced Encryption Standard) cipher engine. Write 1 to enable, 0 to disable.
        SYSTEM_CRYPTO_AES_CLK_EN OFFSET(1) NUMBITS(1),
        /// Enable clock for SHA (Secure Hash Algorithm) hashing engine. Write 1 to enable, 0 to disable.
        SYSTEM_CRYPTO_SHA_CLK_EN OFFSET(2) NUMBITS(1),
        /// Enable clock for RSA (Rivest-Shamir-Adleman) cipher engine. Write 1 to enable, 0 to disable.
        SYSTEM_CRYPTO_RSA_CLK_EN OFFSET(3) NUMBITS(1),
        /// Enable clock for DS (Digital Signature) accelerator. Write 1 to enable, 0 to disable.
        SYSTEM_CRYPTO_DS_CLK_EN OFFSET(4) NUMBITS(1),
        /// Enable clock for HMAC (Hash-based Message Authentication Code) engine. Write 1 to enable, 0 to disable.
        SYSTEM_CRYPTO_HMAC_CLK_EN OFFSET(5) NUMBITS(1),
        /// Enable clock for DMA (Direct Memory Access) controller. Write 1 to enable, 0 to disable.
        SYSTEM_DMA_CLK_EN OFFSET(6) NUMBITS(1),
        /// Enable clock for TSENS (Temperature Sensor) peripheral. Write 1 to enable, 0 to disable.
        SYSTEM_TSENS_CLK_EN OFFSET(10) NUMBITS(1),
    ],

    /// SYSTEM_PERIP_RST_EN0 (0x0018): Peripheral Reset Enable Register 0
    /// Controls reset signals to the first group of peripherals.
    /// Writing 1 asserts reset, 0 deasserts. Reset value: 0x0. Access: R/W
    SYSTEM_PERIP_RST_EN0 [
        /// Reset signal for TIMERS peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_TIMERS_RST OFFSET(0) NUMBITS(1),
        /// Reset signal for SPI0 and SPI1 peripherals. Write 1 to assert reset, 0 to deassert.
        SYSTEM_SPI01_RST OFFSET(1) NUMBITS(1),
        /// Reset signal for UART peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_UART_RST OFFSET(2) NUMBITS(1),
        /// Reset signal for UART1 peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_UART1_RST OFFSET(5) NUMBITS(1),
        /// Reset signal for SPI2 peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_SPI2_RST OFFSET(6) NUMBITS(1),
        /// Reset signal for I2C_EXTO peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_EXTO_RST OFFSET(7) NUMBITS(1),
        /// Reset signal for UHCIO peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_UHCIO_RST OFFSET(8) NUMBITS(1),
        /// Reset signal for RMT (Remote Control) peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_RMT_RST OFFSET(9) NUMBITS(1),
        /// Reset signal for LEDC (LED PWM Controller) peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_LEDC_RST OFFSET(11) NUMBITS(1),
        /// Reset signal for TIMERGROUP peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_TIMERGROUP_RST OFFSET(13) NUMBITS(1),
        /// Reset signal for TIMERGROUP1 peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_TIMERGROUP1_RST OFFSET(15) NUMBITS(1),
        /// Reset signal for TWAI (Two-Wire Automotive Interface) / CAN peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_CAN_RST OFFSET(19) NUMBITS(1),
        /// Reset signal for I2S peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_I2SO_RST OFFSET(21) NUMBITS(1),
        /// Reset signal for USB DEVICE peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_USB_DEVICE_RST OFFSET(23) NUMBITS(1),
        /// Reset signal for UART_MEM peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_UART_MEM_RST OFFSET(24) NUMBITS(1),
        /// Reset signal for SPI3 DMA peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_SPI3_DMA_RST OFFSET(27) NUMBITS(1),
        /// Reset signal for APB_SARADC (ADC) peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_APB_SARADC_RST OFFSET(28) NUMBITS(1),
        /// Reset signal for SYSTIMER peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_SYSTIMER_RST OFFSET(29) NUMBITS(1),
        /// Reset signal for ADC2_ARB (ADC2 Arbiter) peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_ADC2_ARB_RST OFFSET(30) NUMBITS(1),
    ],

    /// SYSTEM_PERIP_RST_EN1 (0x001C): Peripheral Reset Enable Register 1
    /// Controls reset signals to the second group of peripherals (primarily cryptographic engines).
    /// Writing 1 asserts reset, 0 deasserts. Reset value: 0x0. Access: R/W
    SYSTEM_PERIP_RST_EN1 [
        /// Reset signal for AES (Advanced Encryption Standard) cipher engine. Write 1 to assert reset, 0 to deassert.
        SYSTEM_CRYPTO_AES_RST OFFSET(1) NUMBITS(1),
        /// Reset signal for SHA (Secure Hash Algorithm) hashing engine. Write 1 to assert reset, 0 to deassert.
        SYSTEM_CRYPTO_SHA_RST OFFSET(2) NUMBITS(1),
        /// Reset signal for RSA (Rivest-Shamir-Adleman) cipher engine. Write 1 to assert reset, 0 to deassert.
        SYSTEM_CRYPTO_RSA_RST OFFSET(3) NUMBITS(1),
        /// Reset signal for DS (Digital Signature) accelerator. Write 1 to assert reset, 0 to deassert.
        SYSTEM_CRYPTO_DS_RST OFFSET(4) NUMBITS(1),
        /// Reset signal for HMAC (Hash-based Message Authentication Code) engine. Write 1 to assert reset, 0 to deassert.
        SYSTEM_CRYPTO_HMAC_RST OFFSET(5) NUMBITS(1),
        /// Reset signal for DMA (Direct Memory Access) controller. Write 1 to assert reset, 0 to deassert.
        SYSTEM_DMA_RST OFFSET(6) NUMBITS(1),
        /// Reset signal for TSENS (Temperature Sensor) peripheral. Write 1 to assert reset, 0 to deassert.
        SYSTEM_TSENS_RST OFFSET(10) NUMBITS(1),
    ],
];

pub struct Esp32ClockControl;

impl Esp32ClockControl {
    pub fn init() -> u32 {
        unsafe extern "C" {
            fn rtc_get_reset_reason(cpu_num: u32) -> u32;
        }

        let rst = unsafe { rtc_get_reset_reason(0) };
        rst
    }
}
