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

// Bindgen-generated auth-mode constants use C macro names (lower case),
// triggering clippy::non_upper_case_globals in pattern matches.
// The names are not under our control, so allow the lint for the entire file.
#![allow(non_upper_case_globals)]

use alloc::vec::Vec;

use esp_wifi_sys_esp32c3::include::{
    wifi_ap_record_t, wifi_auth_mode_t_WIFI_AUTH_OPEN, wifi_auth_mode_t_WIFI_AUTH_WEP,
    wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE, wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
    wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK, wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK,
    wifi_auth_mode_t_WIFI_AUTH_WPA_PSK, wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK,
};

use crate::net::link::wifi_ops::{WifiScanResult, WifiSecurity};

/// Convert a raw `wifi_ap_record_t` (from esp-wifi scan) into our internal
/// `WifiScanResult`. This is the single point of translation between the
/// ESP-IDF driver format and the BlueOS‑independent scan result type.
pub fn from_ap_record(record: &wifi_ap_record_t) -> WifiScanResult {
    let str_len = record
        .ssid
        .iter()
        .position(|&c| c == 0)
        .unwrap_or(record.ssid.len());
    let ssid = unsafe { core::str::from_utf8_unchecked(&record.ssid[..str_len]) };

    WifiScanResult {
        ssid: ssid.into(),
        bssid: record.bssid,
        signal_dbm: record.rssi,
        channel: record.primary as u16,
        security: to_wifi_security(record.authmode),
    }
}

/// Convert a batch of `wifi_ap_record_t` slices into a `Vec<WifiScanResult>`.
/// Useful after `esp_wifi_scan_get_ap_records`.
pub fn from_ap_records(records: &[wifi_ap_record_t]) -> Vec<WifiScanResult> {
    records.iter().map(from_ap_record).collect()
}

fn to_wifi_security(authmode: u32) -> WifiSecurity {
    match authmode {
        wifi_auth_mode_t_WIFI_AUTH_OPEN => WifiSecurity::Open,
        wifi_auth_mode_t_WIFI_AUTH_WEP => WifiSecurity::Wep,
        wifi_auth_mode_t_WIFI_AUTH_WPA_PSK | wifi_auth_mode_t_WIFI_AUTH_WPA_WPA2_PSK => {
            WifiSecurity::Wpa
        }
        wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK | wifi_auth_mode_t_WIFI_AUTH_WPA2_ENTERPRISE => {
            WifiSecurity::Wpa2
        }
        wifi_auth_mode_t_WIFI_AUTH_WPA3_PSK | wifi_auth_mode_t_WIFI_AUTH_WPA2_WPA3_PSK => {
            WifiSecurity::Wpa3
        }
        _ => WifiSecurity::Unknown,
    }
}
