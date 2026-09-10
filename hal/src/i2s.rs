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

/// I2S data format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2sFormat {
    /// I2S / Philips standard.
    Philips,
    /// Left-justified.
    LeftJustified,
    /// PCM (short sync).
    PcmShort,
    /// PCM (long sync).
    PcmLong,
}

/// I2S channel mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2sChannelMode {
    /// Two-channel stereo (L/R).
    Stereo,
    /// Mono — duplicate on both halves.
    Mono,
}

/// I2S peripheral configuration — used as `P` for HAL `I2s<P, T>` trait.
pub struct I2sConfig {
    pub sample_rate: u32,
    pub bits_per_sample: u8,
    pub channel_mode: I2sChannelMode,
    pub format: I2sFormat,
    /// Master-clock (MCLK) multiplier relative to the sample rate (e.g. 256, 384).
    /// `0` disables MCLK output.
    pub mclk_multiple: u32,
    /// Slot width on the I2S bus (bits per channel slot). The data bits
    /// (`bits_per_sample`) are left-justified within the slot. The reference
    /// ESP-IDF example uses 32-bit slots with 16-bit data
    /// (`I2S_STD_MSB_SLOT_DEFAULT_CONFIG(32, ...)`), which doubles BCK
    /// compared to 16-bit slots.
    pub slot_width: u8,
}

impl I2sConfig {
    /// 16 kHz, 16-bit data in 32-bit slots, stereo, Philips, MCLK = 256×fs.
    pub fn default_16k() -> Self {
        I2sConfig {
            sample_rate: 16_000,
            bits_per_sample: 16,
            channel_mode: I2sChannelMode::Stereo,
            format: I2sFormat::Philips,
            mclk_multiple: 256,
            slot_width: 32,
        }
    }

    /// 48 kHz, 16-bit data in 32-bit slots, stereo, Philips, MCLK = 256×fs.
    pub fn default_48k() -> Self {
        I2sConfig {
            sample_rate: 48_000,
            bits_per_sample: 16,
            channel_mode: I2sChannelMode::Stereo,
            format: I2sFormat::Philips,
            mclk_multiple: 256,
            slot_width: 32,
        }
    }
}

/// I2S peripheral trait — half-duplex write (playback) and read (capture).
///
/// The trait composes `PlatPeri` (enable/disable) and `Configuration<P>`
/// (apply `I2sConfig`). Implementations are expected to handle DMA internally
/// so callers can stream whole buffers without worrying about FIFO depth.
pub trait I2s<P, T>: super::PlatPeri + super::Configuration<P, Target = T> {
    /// Write (play) `buf` bytes to the I2S TX path. Blocks until the buffer
    /// has been handed to the hardware (DMA completion or FIFO drained).
    fn write(&self, buf: &[u8]) -> super::err::Result<()>;

    /// Read (capture) `buf.len()` bytes from the I2S RX path. Blocks until
    /// `buf` is filled.
    fn read(&self, buf: &mut [u8]) -> super::err::Result<()>;
}
