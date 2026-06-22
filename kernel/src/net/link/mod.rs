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

//! Link layer module for the layered network architecture.
//!
//! This module defines the `LinkLayer` trait, `LinkRegistry`, and the
//! `downcast_ref` helper for accessing device-specific traits.
//!
//! `LinkLayer` is the pure L2 abstraction — it has NO dependency on
//! `smoltcp`. Concrete link types implement `LinkLayer` for device
//! control (name, MTU, MAC, etc.) and separately implement
//! `smoltcp::phy::Device` + `SmoltcpDevice` (from `crate::net::smoltcp::link`)
//! for the protocol stack.
//!
//! `NetIface` holds separate `Arc<RwLock<dyn LinkLayer>>` and
//! `Arc<RwLock<dyn SmoltcpDevice>>` references to the same concrete device.
//!
//! # Key design decisions
//!
//! - **No ioctl**: `LinkLayer` does not expose any type-unsafe `ioctl(cmd, arg)`
//!   method. Device-specific operations are accessed via `Any::downcast_ref`.
//! - **Any bound**: `LinkLayer: Any + 'static` enables safe downcasting.
//! - **dyn-compatible**: `LinkLayer` is dyn-compatible.

pub(crate) mod ethernet_ops;
pub(crate) mod loopback;
pub(crate) mod medium;
#[cfg(virtio)]
pub(crate) mod virtio;
pub(crate) mod wifi_ops;

use core::{
    any::Any,
    sync::atomic::{AtomicUsize, Ordering},
};

use alloc::{string::String, sync::Arc, vec::Vec};
use core::fmt;
use spin;

pub(crate) use self::medium::Medium;

use crate::net::{
    iface::{InterfaceFlags, NetIfaceControl, NetIfaceError, NetIfaceResult},
    link::{
        ethernet_ops::EthernetOps,
        wifi_ops::{WifiOps, WifiScanResult},
    },
};

/// A hardware address (MAC or similar).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HwAddr {
    pub bytes: [u8; 8],
    pub len: u8,
}

impl HwAddr {
    pub fn from_ethernet(mac: [u8; 6]) -> Self {
        let mut bytes = [0u8; 8];
        bytes[..6].copy_from_slice(&mac);
        HwAddr { bytes, len: 6 }
    }

    pub fn as_ethernet(&self) -> Option<[u8; 6]> {
        if self.len == 6 {
            let mut mac = [0u8; 6];
            mac.copy_from_slice(&self.bytes[..6]);
            Some(mac)
        } else {
            None
        }
    }
}

impl fmt::Display for HwAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, b) in self.bytes[..self.len as usize].iter().enumerate() {
            if i > 0 {
                write!(f, ":")?;
            }
            write!(f, "{:02x}", b)?;
        }
        Ok(())
    }
}

/// Link-layer device trait for the layered network architecture.
///
/// Replaces the old `NetDevice` enum. Concrete link types implement both
/// `smoltcp::phy::Device` (with GATs) and `LinkLayer` separately — `Device`
/// is NOT a supertrait here because GATs make it not dyn-compatible.
///
/// The `Any` bound enables downcasting to concrete types and device-specific
/// operation traits (e.g., `WifiOps`, `EthernetOps`).
pub trait LinkLayer: Send + Sync + Any + 'static {
    /// Human-readable device name (e.g., "lo", "eth0").
    fn name(&self) -> String;
    /// Medium type (Ethernet, Ip, Ieee802154).
    fn medium(&self) -> Medium;
    /// Maximum transmission unit in bytes.
    fn mtu(&self) -> usize;
    /// Hardware address (MAC for Ethernet, None for loopback).
    fn hw_addr(&self) -> Option<HwAddr>;
    /// Whether the device can currently send.
    fn can_send(&self) -> bool;
    /// Whether the device can currently receive.
    fn can_recv(&self) -> bool;

    /// Optional: return a reference to this device's `WifiOps` implementation.
    fn as_wifi(&mut self) -> Option<&mut dyn WifiOps> {
        None
    }

    /// Optional: return a reference to this device's `EthernetOps` implementation.
    fn as_ethernet(&mut self) -> Option<&mut dyn EthernetOps> {
        None
    }
}

/// Downcast helper for `dyn LinkLayer`.
impl dyn LinkLayer {
    /// Downcast to a concrete `LinkLayer` implementation.
    pub fn downcast_ref<T: 'static>(&self) -> Option<&T> {
        (self as &dyn Any).downcast_ref::<T>()
    }
}

/// Global registry of link-layer devices.
///
/// All devices are registered during `net::init()` (single-threaded, before the
/// network thread starts). After that, the registry is logically immutable —
/// no further `push()` calls are made, so `Mutex` contention is zero at runtime.
pub(crate) struct LinkRegistry {
    devices: spin::Mutex<Vec<Arc<spin::RwLock<dyn LinkLayer>>>>,
}

impl LinkRegistry {
    pub const fn new() -> Self {
        LinkRegistry {
            devices: spin::Mutex::new(Vec::new()),
        }
    }

    /// Push a single device into the registry during init.
    pub fn push(&self, device: Arc<spin::RwLock<dyn LinkLayer>>) {
        self.devices.lock().push(device);
    }

    pub fn get(&self, index: usize) -> Option<Arc<spin::RwLock<dyn LinkLayer>>> {
        self.devices.lock().get(index).cloned()
    }

    pub fn iter(&self) -> Vec<Arc<spin::RwLock<dyn LinkLayer>>> {
        self.devices.lock().clone()
    }

    pub fn len(&self) -> usize {
        self.devices.lock().len()
    }

    pub fn find_by_name(&self, name: &str) -> Option<Arc<spin::RwLock<dyn LinkLayer>>> {
        self.devices
            .lock()
            .iter()
            .find(|dev| dev.read().name() == name)
            .cloned()
    }
}

/// Global link registry instance.
pub(crate) static LINK_REGISTRY: LinkRegistry = LinkRegistry::new();

const WIFI_SCAN_CACHE_IDLE: usize = 0;
const WIFI_SCAN_CACHE_SCANNING: usize = 1;
const WIFI_SCAN_CACHE_READY: usize = 2;

/// Global cache for WiFi scan results. The payload is populated by the WiFi
/// driver's async ScanDone handler and read by `SIOCGIWSCAN` (from `sockfs`).
static WIFI_SCAN_CACHE: spin::Mutex<Option<Vec<WifiScanResult>>> = spin::Mutex::new(None);
static WIFI_SCAN_CACHE_STATE: AtomicUsize = AtomicUsize::new(WIFI_SCAN_CACHE_IDLE);

/// Global cache for the WiFi passphrase, set by SIOCSIWENCODE and consumed by
/// the subsequent SIOCSIWESSID (WifiConnect) ioctl.
static WIFI_PASSPHRASE_CACHE: spin::Mutex<Option<String>> = spin::Mutex::new(None);

/// Mark scan results as pending before starting a new async scan.
pub(crate) fn mark_scan_results_pending() {
    WIFI_SCAN_CACHE_STATE.store(WIFI_SCAN_CACHE_SCANNING, Ordering::Release);
}

/// Mark scan results as unavailable when starting the scan task fails.
pub(crate) fn mark_scan_results_unavailable() {
    WIFI_SCAN_CACHE_STATE.store(WIFI_SCAN_CACHE_IDLE, Ordering::Release);
}

/// Replace the global WiFi scan cache with the latest results.
pub(crate) fn update_scan_results_cache(results: Vec<WifiScanResult>) {
    *WIFI_SCAN_CACHE.lock() = Some(results);
    WIFI_SCAN_CACHE_STATE.store(WIFI_SCAN_CACHE_READY, Ordering::Release);
}

/// Copy cached WiFi scan results into a user-space buffer.
///
/// `buf` is a raw pointer to the user-space destination, `buf_len` is its
/// capacity. Returns the number of bytes written on success, or `EINVAL` /
/// `EAGAIN` / `ENOSPC` on error.
pub(crate) fn copy_scan_results_to_user(
    buf: *mut u8,
    buf_len: usize,
) -> Result<usize, crate::error::Error> {
    match WIFI_SCAN_CACHE_STATE.load(Ordering::Acquire) {
        WIFI_SCAN_CACHE_READY => {}
        WIFI_SCAN_CACHE_SCANNING => return Err(crate::error::code::EAGAIN),
        _ => return Err(crate::error::code::EINVAL),
    }

    let cache = WIFI_SCAN_CACHE.lock();
    let results = cache.as_ref().ok_or(crate::error::code::EINVAL)?;

    // Wire format: little-endian u32 count followed by N serialized entries.
    // Each entry: [u32 ssid_len][ssid bytes][u8[6] bssid][i8 signal][u16 channel][u8 security].
    let mut needed = core::mem::size_of::<u32>();
    for ap in results.iter() {
        needed += core::mem::size_of::<u32>()      // ssid_len
            + ap.ssid.len()                         // ssid bytes
            + 6usize                                 // bssid
            + 1usize                                 // signal_dbm
            + 2usize                                 // channel
            + 1usize;                                // security
    }

    if buf.is_null() {
        return Ok(needed); // query-size-only call
    }
    if buf_len < needed {
        return Err(crate::error::code::ENOSPC);
    }

    let mut offset = 0usize;

    // Write count
    let count = results.len() as u32;
    unsafe {
        core::ptr::write_unaligned(buf.add(offset) as *mut u32, count);
    }
    offset += core::mem::size_of::<u32>();

    for ap in results.iter() {
        // ssid_len
        let ssid_len = ap.ssid.len() as u32;
        unsafe {
            core::ptr::write_unaligned(buf.add(offset) as *mut u32, ssid_len);
        }
        offset += core::mem::size_of::<u32>();

        // ssid bytes
        if !ap.ssid.is_empty() {
            unsafe {
                core::ptr::copy_nonoverlapping(ap.ssid.as_ptr(), buf.add(offset), ap.ssid.len());
            }
            offset += ap.ssid.len();
        }

        // bssid
        unsafe {
            core::ptr::copy_nonoverlapping(ap.bssid.as_ptr(), buf.add(offset), 6);
        }
        offset += 6;

        // signal_dbm
        unsafe {
            core::ptr::write_unaligned(buf.add(offset) as *mut i8, ap.signal_dbm);
        }
        offset += 1;

        // channel
        unsafe {
            core::ptr::write_unaligned(buf.add(offset) as *mut u16, ap.channel);
        }
        offset += 2;

        // security
        let sec = ap.security as u8;
        unsafe {
            core::ptr::write_unaligned(buf.add(offset) as *mut u8, sec);
        }
        offset += 1;
    }

    Ok(needed)
}

/// Handle a type-safe network control command, dispatching to the correct
/// link device by interface name.
///
/// Bridges the POSIX ioctl path (via `Operation::NetControl`) to `LinkLayer`
/// device-specific trait operations.
pub(crate) fn handle_control(cmd: NetIfaceControl) -> Result<NetIfaceResult, NetIfaceError> {
    match cmd {
        NetIfaceControl::GetFlags | NetIfaceControl::GetMacAddress | NetIfaceControl::GetMtu => {
            // Simple getters use the first registered device.
            let link_arc = LINK_REGISTRY.get(0).ok_or(NetIfaceError::DeviceNotFound)?;
            let link = link_arc.read();
            match cmd {
                NetIfaceControl::GetFlags => Ok(NetIfaceResult::Flags(InterfaceFlags {
                    up: link.can_send() || link.can_recv(),
                    running: true,
                    promiscuous: false,
                })),
                NetIfaceControl::GetMacAddress => {
                    let hw = link
                        .hw_addr()
                        .and_then(|h| h.as_ethernet())
                        .unwrap_or([0u8; 6]);
                    Ok(NetIfaceResult::MacAddress(hw))
                }
                NetIfaceControl::GetMtu => Ok(NetIfaceResult::Mtu(link.mtu())),
                _ => unreachable!(),
            }
        }
        // WiFi operations: look up the device by interface name.
        NetIfaceControl::WifiScan(ref config) => {
            let ifname = config
                .ifname
                .iter()
                .take_while(|&&b| b != 0)
                .copied()
                .collect::<alloc::vec::Vec<u8>>();
            let ifname = core::str::from_utf8(&ifname).unwrap_or("");
            let link_arc = LINK_REGISTRY
                .find_by_name(ifname)
                .ok_or(NetIfaceError::DeviceNotFound)?;
            let mut link = link_arc.write();
            let wifi = link
                .as_wifi()
                .ok_or(NetIfaceError::DeviceTraitNotAvailable)?;
            let results = wifi
                .scan(config)
                .map_err(|_| NetIfaceError::DeviceTraitNotAvailable)?;

            Ok(NetIfaceResult::WifiScanResult(results))
        }
        // ── WiFi passphrase cache (SIOCSIWENCODE) ──
        NetIfaceControl::WifiPassphrase(ref passphrase) => {
            *WIFI_PASSPHRASE_CACHE.lock() = Some(passphrase.clone());
            Ok(NetIfaceResult::Void)
        }
        // ── WiFi connect (SIOCSIWESSID) ──
        NetIfaceControl::WifiConnect { ref ifname, ref ssid } => {
            let ifname = ifname
                .iter()
                .take_while(|&&b| b != 0)
                .copied()
                .collect::<alloc::vec::Vec<u8>>();
            let ifname = core::str::from_utf8(&ifname).unwrap_or("");
            let link_arc = LINK_REGISTRY
                .find_by_name(ifname)
                .ok_or(NetIfaceError::DeviceNotFound)?;
            let mut link = link_arc.write();
            let wifi = link
                .as_wifi()
                .ok_or(NetIfaceError::DeviceTraitNotAvailable)?;

            let passphrase = WIFI_PASSPHRASE_CACHE
                .lock()
                .take()
                .unwrap_or_default();

            wifi.connect(ssid, &passphrase)
                .map_err(|_| NetIfaceError::DeviceTraitNotAvailable)?;

            Ok(NetIfaceResult::Void)
        }
        // ── WiFi disconnect ──
        NetIfaceControl::WifiDisconnect => {
            let link_arc = LINK_REGISTRY.get(0).ok_or(NetIfaceError::DeviceNotFound)?;
            let mut link = link_arc.write();
            let wifi = link
                .as_wifi()
                .ok_or(NetIfaceError::DeviceTraitNotAvailable)?;
            wifi.disconnect()
                .map_err(|_| NetIfaceError::DeviceTraitNotAvailable)?;
            Ok(NetIfaceResult::Void)
        }
        // ── WiFi signal strength ──
        NetIfaceControl::WifiSignalStrength => {
            let link_arc = LINK_REGISTRY.get(0).ok_or(NetIfaceError::DeviceNotFound)?;
            let link = link_arc.read();
            // We need &mut for as_wifi, so use write lock
            drop(link);
            let mut link = link_arc.write();
            let wifi = link
                .as_wifi()
                .ok_or(NetIfaceError::DeviceTraitNotAvailable)?;
            let rssi = wifi
                .signal_strength()
                .map_err(|_| NetIfaceError::DeviceTraitNotAvailable)?;
            Ok(NetIfaceResult::WifiSignalStrength(rssi))
        }
        _ => Err(NetIfaceError::NotSupported),
    }
}
