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
use alloc::collections::VecDeque;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use esp_wifi_sys_esp32c3::{
    c_types,
    include::{
        esp_err_t, esp_wifi_internal_free_rx_buffer, esp_wifi_internal_tx,
        wifi_interface_t_WIFI_IF_STA, ESP_ERR_NO_MEM, ESP_OK,
    },
};
use smoltcp::phy::{RxToken, TxToken};
use spin::Mutex;

const WIFI_MTU: usize = 1500;
const WIFI_RX_QUEUE_SIZE: usize = 8;
const WIFI_TX_QUEUE_SIZE: usize = 4;
const WIFI_FRAME_LOG_LIMIT: usize = 16;

static STA_CONNECTED: AtomicBool = AtomicBool::new(false);
static STA_RX_QUEUE: Mutex<VecDeque<PacketBuffer>> = Mutex::new(VecDeque::new());
static WIFI_TX_INFLIGHT: AtomicUsize = AtomicUsize::new(0);
static WIFI_TX_LOG_COUNT: AtomicUsize = AtomicUsize::new(0);
static WIFI_RX_LOG_COUNT: AtomicUsize = AtomicUsize::new(0);

#[ram]
pub(crate) unsafe extern "C" fn esp_wifi_tx_done_cb(
    _ifidx: u8,
    _data: *mut u8,
    _data_len: *mut u16,
    _tx_status: bool,
) {
    WIFI_TX_INFLIGHT
        .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |x| {
            Some(x.saturating_sub(1))
        })
        .ok();
}

pub(crate) unsafe extern "C" fn recv_cb_ap(
    _buffer: *mut c_types::c_void,
    _len: u16,
    eb: *mut c_types::c_void,
) -> esp_err_t {
    esp_wifi_internal_free_rx_buffer(eb);
    0
}

pub(crate) unsafe extern "C" fn recv_cb_sta(
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
) -> esp_err_t {
    let packet = PacketBuffer { buffer, len, eb };
    let count = WIFI_RX_LOG_COUNT.fetch_add(1, Ordering::Relaxed);
    if count < WIFI_FRAME_LOG_LIMIT {
        log_frame("rx", count, packet.as_slice());
    }
    let queued = {
        let mut queue = STA_RX_QUEUE.lock();
        if queue.len() < WIFI_RX_QUEUE_SIZE {
            queue.push_back(packet);
            true
        } else {
            false
        }
    };

    if queued {
        ESP_OK as esp_err_t
    } else {
        ESP_ERR_NO_MEM as esp_err_t
    }
}

pub(crate) fn set_sta_connected(connected: bool) {
    STA_CONNECTED.store(connected, Ordering::Release);
}

pub(crate) fn sta_connected() -> bool {
    STA_CONNECTED.load(Ordering::Acquire)
}

pub(crate) fn sta_can_send() -> bool {
    sta_connected() && WIFI_TX_INFLIGHT.load(Ordering::SeqCst) < WIFI_TX_QUEUE_SIZE
}

pub(crate) fn sta_can_recv() -> bool {
    sta_connected() && !STA_RX_QUEUE.lock().is_empty()
}

pub(crate) fn sta_rx_token() -> Option<WifiRxToken> {
    let packet = STA_RX_QUEUE.lock().pop_front()?;
    Some(WifiRxToken { packet })
}

struct PacketBuffer {
    buffer: *mut c_types::c_void,
    len: u16,
    eb: *mut c_types::c_void,
}

unsafe impl Send for PacketBuffer {}

impl Drop for PacketBuffer {
    fn drop(&mut self) {
        unsafe {
            esp_wifi_internal_free_rx_buffer(self.eb);
        }
    }
}

impl PacketBuffer {
    fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.buffer as *const u8, self.len as usize) }
    }
}

fn read_u16_be(bytes: &[u8], offset: usize) -> u16 {
    u16::from_be_bytes([bytes[offset], bytes[offset + 1]])
}

fn read_u32_be(bytes: &[u8], offset: usize) -> u32 {
    u32::from_be_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

fn checksum_add_bytes(mut sum: u32, bytes: &[u8]) -> u32 {
    let mut offset = 0;
    while offset + 1 < bytes.len() {
        sum += read_u16_be(bytes, offset) as u32;
        offset += 2;
    }
    if offset < bytes.len() {
        sum += (bytes[offset] as u32) << 8;
    }
    sum
}

fn checksum_fold(mut sum: u32) -> u16 {
    while (sum >> 16) != 0 {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    sum as u16
}

fn checksum_valid(sum: u32) -> bool {
    checksum_fold(sum) == 0xffff
}

fn ipv4_checksum_valid(frame: &[u8], ip_header_len: usize) -> bool {
    let ip_offset = 14;
    if frame.len() < ip_offset + ip_header_len {
        return false;
    }
    checksum_valid(checksum_add_bytes(
        0,
        &frame[ip_offset..ip_offset + ip_header_len],
    ))
}

fn tcp_checksum_valid(frame: &[u8], ip_header_len: usize) -> bool {
    let ip_offset = 14;
    let tcp_offset = ip_offset + ip_header_len;
    if frame.len() < ip_offset + 20 || frame.len() < tcp_offset {
        return false;
    }

    let total_len = read_u16_be(frame, 16) as usize;
    if total_len < ip_header_len || frame.len() < ip_offset + total_len {
        return false;
    }

    let tcp_len = total_len - ip_header_len;
    let mut sum = 0;
    sum = checksum_add_bytes(sum, &frame[26..34]);
    sum += frame[23] as u32;
    sum += tcp_len as u32;
    sum = checksum_add_bytes(sum, &frame[tcp_offset..tcp_offset + tcp_len]);
    checksum_valid(sum)
}

fn log_tcp(prefix: &str, count: usize, frame: &[u8], ip_header_len: usize) {
    let tcp_offset = 14 + ip_header_len;
    if frame.len() < tcp_offset + 20 {
        return;
    }

    let src_port = read_u16_be(frame, tcp_offset);
    let dst_port = read_u16_be(frame, tcp_offset + 2);
    let seq = read_u32_be(frame, tcp_offset + 4);
    let ack = read_u32_be(frame, tcp_offset + 8);
    let flags = frame[tcp_offset + 13];
    let checksum = read_u16_be(frame, tcp_offset + 16);
    log::info!(
        "wifi {} tcp #{} {} -> {} flags=0x{:02x} seq={} ack={} checksum=0x{:04x} checksum_ok={}",
        prefix,
        count,
        src_port,
        dst_port,
        flags,
        seq,
        ack,
        checksum,
        tcp_checksum_valid(frame, ip_header_len)
    );
}

fn log_ipv4(prefix: &str, count: usize, frame: &[u8], ethertype: u16) {
    let proto = frame[23];
    let ip_header_len = ((frame[14] & 0x0f) as usize) * 4;
    let ip_checksum = read_u16_be(frame, 24);
    log::info!(
        "wifi {} frame #{} len={} dst_mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} src_mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ethertype=0x{:04x} ipv4 proto={} src={}.{}.{}.{} dst={}.{}.{}.{} ip_checksum=0x{:04x} ip_checksum_ok={}",
        prefix,
        count,
        frame.len(),
        frame[0],
        frame[1],
        frame[2],
        frame[3],
        frame[4],
        frame[5],
        frame[6],
        frame[7],
        frame[8],
        frame[9],
        frame[10],
        frame[11],
        ethertype,
        proto,
        frame[26],
        frame[27],
        frame[28],
        frame[29],
        frame[30],
        frame[31],
        frame[32],
        frame[33],
        ip_checksum,
        ipv4_checksum_valid(frame, ip_header_len)
    );

    if proto == 6 {
        log_tcp(prefix, count, frame, ip_header_len);
    }
}

fn log_arp(prefix: &str, count: usize, frame: &[u8], ethertype: u16) {
    let op = read_u16_be(frame, 20);
    log::info!(
        "wifi {} frame #{} len={} dst_mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} src_mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ethertype=0x{:04x} arp op={} sha={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} spa={}.{}.{}.{} tha={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} tpa={}.{}.{}.{}",
        prefix,
        count,
        frame.len(),
        frame[0],
        frame[1],
        frame[2],
        frame[3],
        frame[4],
        frame[5],
        frame[6],
        frame[7],
        frame[8],
        frame[9],
        frame[10],
        frame[11],
        ethertype,
        op,
        frame[22],
        frame[23],
        frame[24],
        frame[25],
        frame[26],
        frame[27],
        frame[28],
        frame[29],
        frame[30],
        frame[31],
        frame[32],
        frame[33],
        frame[34],
        frame[35],
        frame[36],
        frame[37],
        frame[38],
        frame[39],
        frame[40],
        frame[41]
    );
}

fn log_frame(prefix: &str, count: usize, frame: &[u8]) {
    if frame.len() < 14 {
        log::info!("wifi {} frame #{} len={} short", prefix, count, frame.len());
        return;
    }

    let ethertype = read_u16_be(frame, 12);
    match ethertype {
        0x0800 if frame.len() >= 34 => log_ipv4(prefix, count, frame, ethertype),
        0x0806 if frame.len() >= 42 => log_arp(prefix, count, frame, ethertype),
        _ => log::info!(
            "wifi {} frame #{} len={} ethertype=0x{:04x}",
            prefix,
            count,
            frame.len(),
            ethertype
        ),
    }
}

pub struct WifiTxToken {}

pub struct WifiRxToken {
    packet: PacketBuffer,
}

impl RxToken for WifiRxToken {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(self.packet.as_slice())
    }
}

impl TxToken for WifiTxToken {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buffer = [0u8; WIFI_MTU];
        let frame = &mut buffer[..len.min(WIFI_MTU)];
        let result = f(frame);

        if len > WIFI_MTU {
            log::warn!("wifi tx frame dropped: len={} mtu={}", len, WIFI_MTU);
            return result;
        }

        let count = WIFI_TX_LOG_COUNT.fetch_add(1, Ordering::Relaxed);
        if count < WIFI_FRAME_LOG_LIMIT {
            log_frame("tx", count, frame);
        }

        WIFI_TX_INFLIGHT.fetch_add(1, Ordering::SeqCst);
        let ret = unsafe {
            esp_wifi_internal_tx(
                wifi_interface_t_WIFI_IF_STA,
                frame.as_mut_ptr() as *mut c_types::c_void,
                len as u16,
            )
        };
        if count < WIFI_FRAME_LOG_LIMIT {
            log::info!("wifi tx frame #{} esp_wifi_internal_tx ret={}", count, ret);
        }
        if ret != (ESP_OK as i32) {
            WIFI_TX_INFLIGHT
                .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |x| {
                    Some(x.saturating_sub(1))
                })
                .ok();
            log::warn!("esp_wifi_internal_tx failed: ret={} len={}", ret, len);
        }

        result
    }
}
