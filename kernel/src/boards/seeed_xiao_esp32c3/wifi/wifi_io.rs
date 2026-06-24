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

use super::hal::ram;
use core::sync::atomic::{AtomicU32, Ordering};
use esp_wifi_sys_esp32c3::{
    c_types,
    include::{esp_err_t, esp_wifi_internal_free_rx_buffer},
};
use smoltcp::phy::{RxToken, TxToken};

static STA_RX_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static AP_RX_LOG_COUNT: AtomicU32 = AtomicU32::new(0);

#[ram]
pub(crate) unsafe extern "C" fn esp_wifi_tx_done_cb(
    ifidx: u8,
    data: *mut u8,
    data_len: *mut u16,
    tx_status: bool,
) {
    let len = if data_len.is_null() {
        0
    } else {
        core::ptr::read_volatile(data_len)
    };
    let ethertype = if !data.is_null() && len >= 14 {
        let data = core::slice::from_raw_parts(data as *const u8, len as usize);
        u16::from_be_bytes([data[12], data[13]])
    } else {
        0
    };

    if ethertype == 0x888e {
        log::info!(
            "wifi_tx_done: ifidx={} len={} status={} ethertype=0x{:04x}",
            ifidx,
            len,
            tx_status,
            ethertype,
        );
    }
}

pub(crate) unsafe extern "C" fn recv_cb_ap(
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
) -> esp_err_t {
    let count = AP_RX_LOG_COUNT.fetch_add(1, Ordering::Relaxed);
    if count < 8 {
        log_rx_packet("ap", count, buffer, len, eb);
    }
    esp_wifi_internal_free_rx_buffer(eb);
    0
}

pub(crate) unsafe extern "C" fn recv_cb_sta(
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
) -> esp_err_t {
    let count = STA_RX_LOG_COUNT.fetch_add(1, Ordering::Relaxed);
    if count < 32 {
        log_rx_packet("sta", count, buffer, len, eb);
    }
    esp_wifi_internal_free_rx_buffer(eb);
    0
}

unsafe fn log_rx_packet(
    iface: &str,
    count: u32,
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
) {
    let data = core::slice::from_raw_parts(buffer as *const u8, len as usize);
    let ethertype = if data.len() >= 14 {
        u16::from_be_bytes([data[12], data[13]])
    } else {
        0
    };

    if ethertype == 0x888e || count < 8 {
        log::info!(
            "wifi_rx[{}#{}]: len={} eb={:p} ethertype=0x{:04x}",
            iface,
            count,
            len,
            eb,
            ethertype,
        );
    }
}

pub struct WifiTxToken {}

pub struct WifiRxToken {}

impl RxToken for WifiRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        todo!()
    }
}

impl TxToken for WifiTxToken {
    fn consume<R, F>(self, _len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        todo!()
    }
}
