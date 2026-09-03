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

#[cfg(soc_esp32c3)]
pub mod esp32_spi;

#[cfg(soc_esp32c6)]
pub mod esp32c6_spi;

/// SPI clock phase (CPHA).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiPhase {
    Phase0,
    Phase1,
}

/// SPI clock polarity (CPOL).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiPolarity {
    Low,
    High,
}

/// SPI bit order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiBitOrder {
    MsbFirst,
    LsbFirst,
}

/// SPI peripheral configuration — used as `P` for HAL `Spi<P, T>` trait
pub struct SpiConfig {
    pub baudrate: u32,
    pub phase: SpiPhase,
    pub polarity: SpiPolarity,
    pub bit_order: SpiBitOrder,
    pub cs_pin: Option<u8>, // Unused — CS managed by ExclusiveDevice via GPIO OutputPin
}

impl SpiConfig {
    /// Mode 0 (CPOL=0, CPHA=0), MSB-first, 20MHz — typical SPI NOR Flash config.
    pub fn spi_flash_default() -> Self {
        SpiConfig {
            baudrate: 20_000_000,
            phase: SpiPhase::Phase0,
            polarity: SpiPolarity::Low,
            bit_order: SpiBitOrder::MsbFirst,
            cs_pin: None,
        }
    }

    /// Mode 0, MSB-first, 40 MHz for QSPI-connected display panels.
    pub fn qspi_display_default() -> Self {
        SpiConfig {
            baudrate: 40_000_000,
            phase: SpiPhase::Phase0,
            polarity: SpiPolarity::Low,
            bit_order: SpiBitOrder::MsbFirst,
            cs_pin: None,
        }
    }

    /// Mode 0, MSB-first, 400 kHz — the SD card identification clock.
    pub fn sd_card_init() -> Self {
        SpiConfig {
            baudrate: 400_000,
            phase: SpiPhase::Phase0,
            polarity: SpiPolarity::Low,
            bit_order: SpiBitOrder::MsbFirst,
            cs_pin: None,
        }
    }

    /// Mode 0, MSB-first, 20 MHz — a conservative SD-over-SPI data clock.
    pub fn sd_card_default() -> Self {
        SpiConfig {
            baudrate: 20_000_000,
            phase: SpiPhase::Phase0,
            polarity: SpiPolarity::Low,
            bit_order: SpiBitOrder::MsbFirst,
            cs_pin: None,
        }
    }
}
