// Copyright (c) 2025 vivo Mobile Communication Co., Ltd.
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

mod config;
pub(crate) mod efuse;
#[cfg(enable_net)]
pub mod wifi;
use crate::{
    arch,
    arch::riscv::{local_irq_enabled, trap_entry, Context},
    scheduler,
    sync::SpinLock,
    time,
};
use blueos_driver::{
    interrupt_controller::Interrupt, power::esp32c3_power_domain::PowerDomain,
    uart::esp32_usb_serial::Esp32UsbSerialIsr,
};
use blueos_hal::{isr::IsrDesc, Has8bitDataReg};
use core::{
    ffi::c_void,
    num::NonZeroU32,
    sync::atomic::{AtomicPtr, Ordering},
};
use esp_rom_sys as _;

#[no_mangle]
static mut ESP_HAL_SYSTIMER_CORRECTION: NonZeroU32 = NonZeroU32::new(0x8000_0000).unwrap();

// FIXME: Only support unit0 for now
pub type ClockImpl =
    blueos_driver::systimer::esp32_sys_timer::Esp32SysTimer<0x6002_3000, 16_000_000>;

core::arch::global_asm!(
    "
.section .trap
.type _vector_table, @function

.option push
.balign 0x4
.option norelax
.option norvc

_vector_table:
    j {trap_entry}          // 0: Exception 
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    j {trap_entry}          
    ",
    trap_entry = sym trap_entry,
);

#[inline]
fn init_vector_table() {
    unsafe extern "C" {
        static _vector_table: u32;
    }
    let mut v = core::ptr::addr_of!(_vector_table) as usize;
    v |= 1; // set the least significant bit to enable vectored mode
    unsafe {
        core::arch::asm!(
            "csrw mtvec, {0}",
            in(reg) v,
            options(nostack, preserves_flags),
        );
    }
}

pub(crate) fn handle_intc_irq(ctx: &Context, mcause: usize, mtval: usize) {
    let cpu_id = arch::current_cpu_id();
    match mcause & 0xff {
        0 | 1 => {
            super::wifi::os_adapter::ISR_INTERRUPT_1.dispatch();
        }
        TARGET0_INT_NUM => {
            ClockImpl::clear_interrupt();
            crate::time::handle_clock_interrupt();
        }
        USB_SERIAL_JTAG_INT_NUM => {
            ESP32_USB_SERIAL_ISR.service_isr();
        }
        _ => {
            crate::kearly_println!(
                "CPU {}: Unexpected interrupt: mcause=0x{:x}, mtval=0x{:x}",
                cpu_id,
                mcause,
                mtval
            );
        }
    }
}

const TARGET0_INT_NUM: usize = 16;

const USB_SERIAL_JTAG_INT_NUM: usize = 15;

const RTC_CNTL_BASE: usize = 0x6000_8000;
const RTC_CNTL_WDTWRITECT_REG: usize = RTC_CNTL_BASE + 0xA8;
const RTC_CNTL_WDTCONFIG0_REG: usize = RTC_CNTL_BASE + 0x90;

const USB_SERIAL_JTAG_IRQ: Interrupt = Interrupt::new(26, USB_SERIAL_JTAG_INT_NUM);
const SYSTIMER_TARGET0_IRQ: Interrupt = Interrupt::new(37, TARGET0_INT_NUM);

pub(super) fn random_u32() -> u32 {
    let mut data = [0u8; 4];
    random(&mut data);
    u32::from_le_bytes(data)
}

pub(super) fn random(data: &mut [u8]) {
    use blueos_driver::rng::esp32c3_rng::Esp32c3Rng;
    static RNG: SpinLock<Esp32c3Rng> = SpinLock::new(Esp32c3Rng::new());

    let wait_timer_cycles = 16_000_000 * 32 / 80_000_000;
    let until_tick = time::Tick::after(time::Tick(wait_timer_cycles));

    let mut remaining = data.len();
    let mut offset = 0;
    while remaining > 0 {
        loop {
            if until_tick.is_elapsed() {
                break;
            }
            core::hint::spin_loop();
        }
        let random_bytes = RNG.lock().read_one().to_le_bytes();
        let bytes_to_copy = random_bytes.len().min(remaining);
        data[offset..offset + bytes_to_copy].copy_from_slice(&random_bytes[..bytes_to_copy]);
        offset += bytes_to_copy;
        remaining -= bytes_to_copy;
    }
}

pub(crate) fn init() {
    assert!(!local_irq_enabled());

    crate::boot::init_runtime();
    crate::boot::init_heap();
    init_vector_table();

    blueos_driver::systimer::esp32_sys_timer::Esp32SysTimer::<0x6002_3000, 16_000_000>::init();

    unsafe {
        // disable WDT to avoid unexpected reset
        core::ptr::write_volatile(RTC_CNTL_WDTWRITECT_REG as *mut u32, 0x50D83AA1);
        core::ptr::write_volatile(RTC_CNTL_WDTCONFIG0_REG as *mut u32, 0);
        core::ptr::write_volatile(RTC_CNTL_WDTWRITECT_REG as *mut u32, 0);
    }

    get_device!(intc).allocate_irq(SYSTIMER_TARGET0_IRQ);
    get_device!(intc).allocate_irq(USB_SERIAL_JTAG_IRQ);

    get_device!(intc).set_threshold(1);

    get_device!(intc).set_priority(USB_SERIAL_JTAG_IRQ, 15);
    get_device!(intc).set_priority(SYSTIMER_TARGET0_IRQ, 15);
    get_device!(intc).enable_irq(SYSTIMER_TARGET0_IRQ);
    get_device!(intc).enable_irq(USB_SERIAL_JTAG_IRQ);

    let power_domain = PowerDomain::new(0x6000_8000);
    power_domain.enable_wifi();

    unsafe {
        use esp_wifi_sys_esp32c3::include::{
            esp_wifi_internal_set_log_level, wifi_log_level_t_WIFI_LOG_VERBOSE,
        };

        esp_wifi_internal_set_log_level(wifi_log_level_t_WIFI_LOG_VERBOSE);

        // open wifi clk
        // modified from https://github.com/esp-rs/esp-hal/blob/63ff86ca206fc1bd25699527ed30094f3bb9a872/esp-radio/src/radio_clocks/clocks_ll/esp32c3.rs#L35-L42
        const SYSTEM_WIFI_CLK_I2C_CLK_EN: u32 = 1 << 5;
        const SYSTEM_WIFI_CLK_UNUSED_BIT12: u32 = 1 << 12;
        const WIFI_BT_SDIO_CLK: u32 = SYSTEM_WIFI_CLK_I2C_CLK_EN | SYSTEM_WIFI_CLK_UNUSED_BIT12;
        let tmp = core::ptr::read_volatile(0x6002_6014 as *const u32);
        core::ptr::write_volatile(
            0x6002_6014 as *mut u32,
            tmp & !WIFI_BT_SDIO_CLK | 0x00FB9FCF,
        );
    }
}

crate::define_peripheral! {
    (console_uart, blueos_driver::uart::esp32_usb_serial::Esp32UsbSerial,
     blueos_driver::uart::esp32_usb_serial::Esp32UsbSerial::new()),
    (intc, blueos_driver::interrupt_controller::esp32_intc::Esp32Intc,
     blueos_driver::interrupt_controller::esp32_intc::Esp32Intc::new(0x600c_2000)),
}

crate::define_pin_states!(None);

#[inline(always)]
pub(crate) fn send_ipi(_hart: usize) {}

#[inline(always)]
pub(crate) fn clear_ipi(_hart: usize) {}

static ESP32_USB_SERIAL_ISR: Esp32UsbSerialIsr<0x6004_3000, crate::drivers::serial::Serial> =
    Esp32UsbSerialIsr::<0x6004_3000, _> {
        data: &crate::drivers::serial::TTY_SERIAL,
        tx_isr: Some(crate::drivers::serial::Serial::xmitchars),
        rx_isr: Some(crate::drivers::serial::Serial::recvchars),
    };

pub struct Handler {
    f: AtomicPtr<c_void>,
    arg: AtomicPtr<c_void>,
}

impl Handler {
    pub const fn new() -> Self {
        Self {
            f: AtomicPtr::new(core::ptr::null_mut()),
            arg: AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    pub fn set(&self, f: *const c_void, arg: *const c_void) {
        self.arg.store(arg.cast_mut(), Ordering::Relaxed);
        self.f.store(f.cast_mut(), Ordering::Release);
    }

    pub fn dispatch(&self) {
        let f = self.f.load(Ordering::Acquire);
        if !f.is_null() {
            let func = unsafe {
                core::mem::transmute::<*const c_void, unsafe extern "C" fn(*mut c_void)>(f)
            };
            let arg = self.arg.load(Ordering::Relaxed);
            unsafe { func(arg) };
        }
    }
}
