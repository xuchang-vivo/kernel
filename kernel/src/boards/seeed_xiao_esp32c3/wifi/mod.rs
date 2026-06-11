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

mod event;
pub(crate) mod os_adapter;
mod wifi_io;
use crate::{
    kearly_println,
    net::{
        link::{
            wifi_ops::{WifiOps, WifiScanConfig, WifiScanResult},
            HwAddr, LinkLayer, Medium,
        },
        smoltcp::link::SmoltcpDevice,
        NetError,
    },
    scheduler, thread,
    thread::{Entry, SystemThreadStorage, ThreadKind, ThreadNode},
};
use alloc::{string::String, vec, vec::Vec};
use core::{
    ffi::{c_char, VaList as valist},
    fmt::{Debug, Write},
    mem::MaybeUninit,
    ptr,
    ptr::addr_of,
    str,
};
use esp_hal as hal;
use esp_wifi_sys_esp32c3::include::{
    __BindgenBitfieldUnit, esp_event_base_t, esp_interface_t_ESP_IF_WIFI_AP,
    esp_interface_t_ESP_IF_WIFI_STA, esp_supplicant_init, esp_wifi_init_internal,
    esp_wifi_internal_reg_rxcb, esp_wifi_scan_start, esp_wifi_set_config, esp_wifi_set_country,
    esp_wifi_set_mode, esp_wifi_set_protocols, esp_wifi_set_ps, esp_wifi_set_tx_done_cb,
    esp_wifi_start, g_wifi_default_wpa_crypto_funcs, wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
    wifi_config_t, wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL, wifi_country_t,
    wifi_init_config_t, wifi_interface_t_WIFI_IF_STA, wifi_mode_t_WIFI_MODE_NULL,
    wifi_mode_t_WIFI_MODE_STA, wifi_osi_funcs_t, wifi_pmf_config_t, wifi_protocols_t,
    wifi_ps_type_t_WIFI_PS_NONE, wifi_scan_config_t, wifi_scan_threshold_t,
    wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE, wifi_scan_type_t_WIFI_SCAN_TYPE_PASSIVE,
    wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL, wifi_sta_config_t, ESP_OK,
    ESP_WIFI_OS_ADAPTER_MAGIC, ESP_WIFI_OS_ADAPTER_VERSION, WIFI_ENABLE_ENTERPRISE,
    WIFI_ENABLE_WPA3_SAE, WIFI_INIT_CONFIG_MAGIC,
};
use libc::IW_SCAN_TYPE_ACTIVE;
use os_adapter::*;
use smoltcp::{
    iface::{Interface, SocketSet},
    phy::{Device, Medium as SmoltcpMedium},
    time::Instant,
    wire::HardwareAddress,
};
use wifi_io::*;

pub const WIFI_PROTOCOL_11B: u32 = 1;
pub const WIFI_PROTOCOL_11G: u32 = 2;
pub const WIFI_PROTOCOL_11N: u32 = 4;
pub const WIFI_PROTOCOL_LR: u32 = 8;
pub const WIFI_PROTOCOL_11A: u32 = 16;
pub const WIFI_PROTOCOL_11AC: u32 = 32;
pub const WIFI_PROTOCOL_11AX: u32 = 64;

enum WifiState {
    Idle,
    Scanning,
    ScanDone,
}

pub struct WifiController {
    state: WifiState,
}

static mut WIFI_INIT_STORAGE: SystemThreadStorage = SystemThreadStorage::new(ThreadKind::Normal);
static mut WIFI_INIT: MaybeUninit<ThreadNode> = MaybeUninit::zeroed();

extern "C" fn wifi_inner_init() {
    crate::boards::wifi::wifi_init().expect("Failed to initialize Wi-Fi");
}

impl WifiController {
    pub fn new() -> Self {
        let wifi_init = crate::thread::build_static_thread(
            unsafe { &mut WIFI_INIT },
            unsafe { &mut WIFI_INIT_STORAGE },
            0,
            thread::IDLE,
            Entry::C(wifi_inner_init),
            ThreadKind::Normal,
        );
        let ok = scheduler::queue_ready_thread(thread::IDLE, wifi_init);
        debug_assert_eq!(ok, Ok(()));
        Self {
            state: WifiState::Idle,
        }
    }

    fn apply_sta_config() -> Result<(), NetError> {
        unsafe {
            let mut cfg = wifi_config_t {
                sta: wifi_sta_config_t {
                    ssid: [0; 32],
                    password: [0; 64],
                    scan_method: 0,
                    bssid_set: false,
                    bssid: [0; 6],
                    channel: 0,
                    listen_interval: 3,
                    sort_method: wifi_sort_method_t_WIFI_CONNECT_AP_BY_SIGNAL,
                    threshold: wifi_scan_threshold_t {
                        rssi: -99,
                        authmode: wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
                        rssi_5g_adjustment: 0,
                    },
                    pmf_cfg: wifi_pmf_config_t {
                        capable: true,
                        required: false,
                    },
                    sae_pwe_h2e: 3,
                    _bitfield_align_1: [0; 0],
                    _bitfield_1: __BindgenBitfieldUnit::new([0; 4]),
                    failure_retry_cnt: 1,
                    _bitfield_align_2: [0; 0],
                    _bitfield_2: __BindgenBitfieldUnit::new([0; 4]),
                    sae_pk_mode: 0, // ??
                    sae_h2e_identifier: [0; 32],
                },
            };

            let ret = esp_wifi_set_config(
                esp_interface_t_ESP_IF_WIFI_STA,
                &cfg as *const wifi_config_t as *mut wifi_config_t,
            );
            if ret != (ESP_OK as i32) {
                return Err(NetError::NoRoute);
            }
        };

        Ok(())
    }

    fn set_config() -> Result<(), NetError> {
        struct ResetModeOnDrop;
        impl ResetModeOnDrop {
            /// Prevent resetting the Wi-Fi mode when the guard is dropped.
            fn defuse(self) {
                core::mem::forget(self);
            }
        }
        impl Drop for ResetModeOnDrop {
            fn drop(&mut self) {
                unsafe { esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL) };
            }
        }

        let reset_mode_on_error = ResetModeOnDrop;

        let ret = unsafe { esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_STA) };
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }
        Self::apply_sta_config()?;

        let p = wifi_protocols_t {
            ghz_2g: (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N) as u16,
            ghz_5g: 0,
        };
        let ret = unsafe {
            esp_wifi_set_protocols(
                wifi_interface_t_WIFI_IF_STA,
                &p as *const wifi_protocols_t as *mut wifi_protocols_t,
            )
        };

        let ret = unsafe { esp_wifi_start() };
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        reset_mode_on_error.defuse();

        Ok(())
    }

    pub fn mac_address(&self) -> [u8; 6] {
        let mut mac = [0u8; 6];
        let tmp = crate::boards::efuse::read_mac_address();
        mac.copy_from_slice(tmp.bytes());
        mac
    }
}

impl LinkLayer for WifiController {
    fn name(&self) -> String {
        String::from("wlan0")
    }

    fn medium(&self) -> Medium {
        Medium::Wifi
    }

    fn mtu(&self) -> usize {
        1500
    }

    fn hw_addr(&self) -> Option<HwAddr> {
        None
    }

    fn can_send(&self) -> bool {
        true
    }

    fn can_recv(&self) -> bool {
        true
    }

    fn as_wifi(&mut self) -> Option<&mut dyn WifiOps> {
        Some(self)
    }
}

impl Device for WifiController {
    type RxToken<'a>
        = WifiRxToken
    where
        Self: 'a;
    type TxToken<'a>
        = WifiTxToken
    where
        Self: 'a;

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        None
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        None
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.max_transmission_unit = self.mtu();
        caps
    }
}

impl SmoltcpDevice for WifiController {
    fn create_smoltcp_iface(&mut self) -> (Interface, SocketSet<'static>) {
        use smoltcp::iface::Config;

        let config = Config::new(HardwareAddress::Ethernet(smoltcp::wire::EthernetAddress(
            self.mac_address(),
        )));
        let mut iface = Interface::new(
            config,
            self,
            Instant::from_millis(i64::try_from(crate::time::now().as_millis()).unwrap_or(0)),
        );
        let sockets = SocketSet::new(vec![]);
        (iface, sockets)
    }

    fn poll_smoltcp(
        &mut self,
        _timestamp: Instant,
        iface: &mut Interface,
        sockets: &mut SocketSet,
    ) {
    }
}

impl WifiOps for WifiController {
    fn scan(&mut self, config: &WifiScanConfig) -> Result<Vec<WifiScanResult>, NetError> {
        log::debug!("Starting WiFi scan with config: {:?}", config);

        let mut cfg: wifi_scan_config_t = unsafe { core::mem::zeroed() };
        cfg.scan_type = if config.scan_type == IW_SCAN_TYPE_ACTIVE as u8 {
            wifi_scan_type_t_WIFI_SCAN_TYPE_ACTIVE
        } else {
            wifi_scan_type_t_WIFI_SCAN_TYPE_PASSIVE
        };

        let ret = unsafe { esp_wifi_scan_start(&cfg as *const wifi_scan_config_t, false) };
        if ret != (ESP_OK as i32) {
            log::error!("Failed to start WiFi scan: error code {}", ret);
            Err(NetError::NoRoute)
        } else {
            self.state = WifiState::Scanning;
            Ok(Vec::new())
        }
    }

    fn connect(&mut self, ssid: &str, passphrase: &str) -> Result<(), NetError> {
        todo!()
    }

    fn disconnect(&mut self) -> Result<(), NetError> {
        todo!()
    }

    fn signal_strength(&self) -> Result<i8, NetError> {
        todo!()
    }
}

/// Information about a connected station.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ssid {
    ssid: [u8; 32],
    len: u8,
}

impl Ssid {
    pub(crate) fn new(ssid: &str) -> Self {
        let mut ssid_bytes = [0u8; 32];
        let bytes = ssid.as_bytes();
        let len = usize::min(32, bytes.len());
        ssid_bytes[..len].copy_from_slice(bytes);

        Self::from_raw(&ssid_bytes, len as u8)
    }

    pub(crate) fn from_raw(ssid: &[u8], len: u8) -> Self {
        let mut ssid_bytes = [0u8; 32];
        let len = usize::min(32, len as usize);
        ssid_bytes[..len].copy_from_slice(&ssid[..len]);

        Self {
            ssid: ssid_bytes,
            len: len as u8,
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.ssid[..self.len as usize]
    }

    /// The length (in bytes) of the SSID.
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Returns true if the SSID is empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The SSID as a string slice.
    pub fn as_str(&self) -> &str {
        let part = &self.ssid[..self.len as usize];
        match str::from_utf8(part) {
            Ok(s) => s,
            Err(e) => {
                let (valid, _) = part.split_at(e.valid_up_to());
                unsafe { str::from_utf8_unchecked(valid) }
            }
        }
    }
}

impl Debug for Ssid {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_char('"')?;
        f.write_str(self.as_str())?;
        f.write_char('"')
    }
}

impl From<alloc::string::String> for Ssid {
    fn from(ssid: alloc::string::String) -> Self {
        Self::new(&ssid)
    }
}

impl From<&str> for Ssid {
    fn from(ssid: &str) -> Self {
        Self::new(ssid)
    }
}

impl From<&[u8]> for Ssid {
    fn from(ssid: &[u8]) -> Self {
        Self::from_raw(ssid, ssid.len() as u8)
    }
}

const WIFI_FEATURE_CAPS: u64 = (WIFI_ENABLE_WPA3_SAE | WIFI_ENABLE_ENTERPRISE) as u64;

#[unsafe(no_mangle)]
static mut __ESP_RADIO_WIFI_EVENT: esp_event_base_t = c"WIFI_EVENT".as_ptr();

#[unsafe(no_mangle)]
pub(super) static mut __ESP_RADIO_G_WIFI_FEATURE_CAPS: u64 = WIFI_FEATURE_CAPS;

#[no_mangle]
pub(crate) static __ESP_RADIO_G_WIFI_OSI_FUNCS: wifi_osi_funcs_t = wifi_osi_funcs_t {
    _version: ESP_WIFI_OS_ADAPTER_VERSION as i32,
    _env_is_chip: Some(env_is_chip),
    _set_intr: Some(set_intr),
    _clear_intr: Some(clear_intr),
    _set_isr: Some(set_isr),
    _ints_on: Some(ints_on),
    _ints_off: Some(ints_off),
    _is_from_isr: Some(is_from_isr),
    _spin_lock_create: Some(spin_lock_create),
    _spin_lock_delete: Some(spin_lock_delete),
    _wifi_int_disable: Some(wifi_int_disable),
    _wifi_int_restore: Some(wifi_int_restore),
    _task_yield_from_isr: Some(task_yield_from_isr),
    _semphr_create: Some(semphr_create),
    _semphr_delete: Some(semphr_delete),
    _semphr_take: Some(semphr_take),
    _semphr_give: Some(semphr_give),
    _wifi_thread_semphr_get: Some(wifi_thread_semphr_get),
    _mutex_create: Some(mutex_create),
    _recursive_mutex_create: Some(recursive_mutex_create),
    _mutex_delete: Some(mutex_delete),
    _mutex_lock: Some(mutex_lock),
    _mutex_unlock: Some(mutex_unlock),
    _queue_create: Some(queue_create),
    _queue_delete: Some(queue_delete),
    _queue_send: Some(queue_send),
    _queue_send_from_isr: Some(queue_send_from_isr),
    _queue_send_to_back: Some(queue_send_to_back),
    _queue_send_to_front: Some(queue_send_to_front),
    _queue_recv: Some(queue_recv),
    _queue_msg_waiting: Some(queue_msg_waiting),
    _event_group_create: Some(event_group_create),
    _event_group_delete: Some(event_group_delete),
    _event_group_set_bits: Some(event_group_set_bits),
    _event_group_clear_bits: Some(event_group_clear_bits),
    _event_group_wait_bits: Some(event_group_wait_bits),
    _task_create_pinned_to_core: Some(task_create_pinned_to_core),
    _task_create: Some(task_create),
    _task_delete: Some(task_delete),
    _task_delay: Some(task_delay),
    _task_ms_to_tick: Some(task_ms_to_tick),
    _task_get_current_task: Some(task_get_current_task),
    _task_get_max_priority: Some(task_get_max_priority),
    _malloc: Some(malloc),
    _free: Some(free),
    _event_post: Some(event_post),
    _get_free_heap_size: Some(get_free_heap_size),
    _rand: Some(rand),
    _dport_access_stall_other_cpu_start_wrap: Some(dport_access_stall_other_cpu_start_wrap),
    _dport_access_stall_other_cpu_end_wrap: Some(dport_access_stall_other_cpu_end_wrap),
    _wifi_apb80m_request: Some(wifi_apb80m_request),
    _wifi_apb80m_release: Some(wifi_apb80m_release),
    _phy_disable: Some(phy_disable),
    _phy_enable: Some(phy_enable),
    _phy_update_country_info: Some(phy_update_country_info),
    _read_mac: Some(read_mac),
    _timer_arm: Some(ets_timer_arm),
    _timer_disarm: Some(ets_timer_disarm),
    _timer_done: Some(ets_timer_done),
    _timer_setfn: Some(ets_timer_setfn),
    _timer_arm_us: Some(ets_timer_arm_us),
    _wifi_reset_mac: Some(wifi_reset_mac),
    _wifi_clock_enable: Some(wifi_clock_enable),
    _wifi_clock_disable: Some(wifi_clock_disable),
    _wifi_rtc_enable_iso: Some(wifi_rtc_enable_iso),
    _wifi_rtc_disable_iso: Some(wifi_rtc_disable_iso),
    _esp_timer_get_time: Some(__esp_radio_esp_timer_get_time),
    _nvs_set_i8: Some(nvs_set_i8),
    _nvs_get_i8: Some(nvs_get_i8),
    _nvs_set_u8: Some(nvs_set_u8),
    _nvs_get_u8: Some(nvs_get_u8),
    _nvs_set_u16: Some(nvs_set_u16),
    _nvs_get_u16: Some(nvs_get_u16),
    _nvs_open: Some(nvs_open),
    _nvs_close: Some(nvs_close),
    _nvs_commit: Some(nvs_commit),
    _nvs_set_blob: Some(nvs_set_blob),
    _nvs_get_blob: Some(nvs_get_blob),
    _nvs_erase_key: Some(nvs_erase_key),
    _get_random: Some(get_random),
    _get_time: Some(get_time),
    _random: Some(random),
    _slowclk_cal_get: Some(slowclk_cal_get),
    _log_write: None,
    _log_writev: None,
    _log_timestamp: Some(log_timestamp),
    _malloc_internal: Some(malloc_internal),
    _realloc_internal: Some(realloc_internal),
    _calloc_internal: Some(calloc_internal_wrapper),
    _zalloc_internal: Some(zalloc_internal),
    _wifi_malloc: Some(wifi_malloc),
    _wifi_realloc: Some(wifi_realloc),
    _wifi_calloc: Some(wifi_calloc),
    _wifi_zalloc: Some(wifi_zalloc),
    _wifi_create_queue: Some(wifi_create_queue),
    _wifi_delete_queue: Some(wifi_delete_queue),
    _coex_init: Some(coex_init),
    _coex_deinit: Some(coex_deinit),
    _coex_enable: Some(coex_enable),
    _coex_disable: Some(coex_disable),
    _coex_status_get: Some(coex_status_get),
    _coex_condition_set: None,
    _coex_wifi_request: Some(coex_wifi_request),
    _coex_wifi_release: Some(coex_wifi_release),
    _coex_wifi_channel_set: Some(coex_wifi_channel_set),
    _coex_event_duration_get: Some(coex_event_duration_get),
    _coex_pti_get: Some(coex_pti_get),
    _coex_schm_status_bit_clear: Some(coex_schm_status_bit_clear),
    _coex_schm_status_bit_set: Some(coex_schm_status_bit_set),
    _coex_schm_interval_set: Some(coex_schm_interval_set),
    _coex_schm_interval_get: Some(coex_schm_interval_get),
    _coex_schm_curr_period_get: Some(coex_schm_curr_period_get),
    _coex_schm_curr_phase_get: Some(coex_schm_curr_phase_get),
    _coex_register_start_cb: Some(coex_register_start_cb),
    _coex_schm_process_restart: Some(coex_schm_process_restart),
    _coex_schm_register_cb: Some(coex_schm_register_cb_wrapper),
    _coex_schm_flexible_period_set: Some(coex_schm_flexible_period_set),
    _coex_schm_flexible_period_get: Some(coex_schm_flexible_period_get),
    _coex_schm_get_phase_by_idx: Some(coex_schm_get_phase_by_idx),

    _magic: ESP_WIFI_OS_ADAPTER_MAGIC as i32,
};

#[no_mangle]
pub(crate) static mut G_WIFI_CONFIG: MaybeUninit<wifi_init_config_t> = MaybeUninit::zeroed();

pub fn wifi_init() -> Result<(), NetError> {
    unsafe {
        G_WIFI_CONFIG.write(wifi_init_config_t {
            osi_funcs: (&__ESP_RADIO_G_WIFI_OSI_FUNCS) as *const wifi_osi_funcs_t
                as *mut wifi_osi_funcs_t,

            wpa_crypto_funcs: g_wifi_default_wpa_crypto_funcs,
            static_rx_buf_num: 10,
            dynamic_rx_buf_num: 32,
            tx_buf_type: esp_wifi_sys_esp32c3::include::CONFIG_ESP_WIFI_TX_BUFFER_TYPE as i32,
            static_tx_buf_num: 0,
            dynamic_tx_buf_num: 32,
            rx_mgmt_buf_type: esp_wifi_sys_esp32c3::include::CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF
                as i32,
            rx_mgmt_buf_num: esp_wifi_sys_esp32c3::include::CONFIG_ESP_WIFI_RX_MGMT_BUF_NUM_DEF
                as i32,
            cache_tx_buf_num: esp_wifi_sys_esp32c3::include::WIFI_CACHE_TX_BUFFER_NUM as i32,
            csi_enable: cfg!(feature = "csi") as i32,
            ampdu_rx_enable: true as i32,
            ampdu_tx_enable: true as i32,
            amsdu_tx_enable: false as i32,
            nvs_enable: 0,
            nano_enable: 0,
            rx_ba_win: 6,
            wifi_task_core_id: 0,
            beacon_max_len: esp_wifi_sys_esp32c3::include::WIFI_SOFTAP_BEACON_MAX_LEN as i32,
            mgmt_sbuf_num: esp_wifi_sys_esp32c3::include::WIFI_MGMT_SBUF_NUM as i32,
            feature_caps: __ESP_RADIO_G_WIFI_FEATURE_CAPS,
            sta_disconnected_pm: false,
            espnow_max_encrypt_num:
                esp_wifi_sys_esp32c3::include::CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM as i32,

            tx_hetb_queue_num: 3,
            dump_hesigb_enable: false,

            magic: WIFI_INIT_CONFIG_MAGIC as i32,
        });

        let ret = esp_wifi_init_internal(addr_of!(G_WIFI_CONFIG) as *const wifi_init_config_t);
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        let ret = esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_NULL);
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        let ret = esp_supplicant_init();
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        let ret = esp_wifi_set_tx_done_cb(Some(esp_wifi_tx_done_cb));
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        let ret = esp_wifi_internal_reg_rxcb(esp_interface_t_ESP_IF_WIFI_AP, Some(recv_cb_ap));
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        let ret = esp_wifi_internal_reg_rxcb(esp_interface_t_ESP_IF_WIFI_STA, Some(recv_cb_sta));
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        let country_info = wifi_country_t {
            cc: [b'C' as i8, b'N' as i8, 0],
            schan: 1,
            nchan: 13,
            max_tx_power: 20,
            policy: wifi_country_policy_t_WIFI_COUNTRY_POLICY_MANUAL,
        };
        let ret = esp_wifi_set_country(&country_info as *const wifi_country_t);
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        let ret = esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE);
        if ret != (ESP_OK as i32) {
            return Err(NetError::NoRoute);
        }

        WifiController::set_config()?;

        log::debug!("WiFi initialized successfully");
        Ok(())
    }
}
