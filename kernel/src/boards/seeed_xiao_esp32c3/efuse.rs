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

use blueos_driver::static_ref::StaticRef;
use tock_registers::{interfaces::Readable, register_structs, registers::ReadOnly};

const EFUSE_BASE: usize = 0x6000_8800;

register_structs! {
    /// Register block for the eFuse controller.
    EfuseRegisters {
        /// eFuse read control register.
        (0x00 => _reserved0),
        /// eFuse read data register.
        (0x044 => rd_mac_spi_sys_0: ReadOnly<u32>),
        (0x048 => rd_mac_spi_sys_1: ReadOnly<u32>),
        (0x04C => rd_mac_spi_sys_2: ReadOnly<u32>),
        (0x050 => rd_mac_spi_sys_3: ReadOnly<u32>),
        (0x054 => rd_mac_spi_sys_4: ReadOnly<u32>),
        (0x058 => rd_mac_spi_sys_5: ReadOnly<u32>),
        /// Reserved space.
        (0x05C => @END),
    }
}

pub struct MacAddress([u8; 6]);

impl core::fmt::Display for MacAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (i, b) in self.0.iter().enumerate() {
            if i != 0 {
                f.write_str(":")?;
            }
            write!(f, "{b:02x}")?;
        }
        Ok(())
    }
}

impl MacAddress {
    fn new(bytes: [u8; 6]) -> Self {
        Self(bytes)
    }

    pub fn bytes(&self) -> &[u8; 6] {
        &self.0
    }

    pub fn get_local_mac(&self) -> [u8; 6] {
        let bytes = self.bytes();
        let mut local_mac = *bytes;
        let base = bytes[0];

        for i in 0..64 {
            let derived = (base | 0x02) ^ (i << 2);
            if derived != base {
                local_mac[0] = derived;
                break;
            }
        }
        local_mac
    }
}

static EFUSE_REGISTERS: StaticRef<EfuseRegisters> =
    unsafe { StaticRef::new(EFUSE_BASE as *const EfuseRegisters) };

pub(super) fn read_mac_address() -> MacAddress {
    let mac0 = EFUSE_REGISTERS.rd_mac_spi_sys_0.get();

    let mac1 = EFUSE_REGISTERS.rd_mac_spi_sys_1.get();

    let mac1_bytes = mac1.to_le_bytes();
    let mac0_bytes = mac0.to_le_bytes();

    MacAddress::new([
        mac1_bytes[1],
        mac1_bytes[0],
        mac0_bytes[3],
        mac0_bytes[2],
        mac0_bytes[1],
        mac0_bytes[0],
    ])
}
