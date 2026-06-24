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

// C bridge for ESP WiFi driver logging.
// Provides vsnprintf formatting that Rust cannot do directly with va_list.
// We declare vsnprintf manually instead of including <stdio.h> because
// the clang --target=riscv32 cross-compiler cannot find the GCC sysroot's
// stdio.h when -nostdlib is set.

#include <stdarg.h>
#include <stddef.h>

// Manual declaration of vsnprintf — provided by the final linked binary
// (the ESP WiFi static libraries already contain vsnprintf implementations).
int vsnprintf(char *str, size_t size, const char *format, va_list ap);

// Buffer size for formatted log messages
#define WIFI_LOG_BUF_SIZE 256

// External Rust logging function declared in os_adapter.rs
extern void blueos_wifi_log_output(unsigned int level, const char *tag,
                                   const char *msg);

// Bridge function: format the va_list message and call Rust log output
// This is the _log_writev callback registered in wifi_osi_funcs_t
void wifi_log_writev_bridge(unsigned int level, const char *tag,
                             const char *format, va_list args) {
    char buf[WIFI_LOG_BUF_SIZE];
    int len = vsnprintf(buf, sizeof(buf), format, args);
    if (len < 0) {
        buf[0] = '\0';
    } else if (len >= (int)sizeof(buf)) {
        // Truncated — ensure null terminator
        buf[sizeof(buf) - 1] = '\0';
    }
    blueos_wifi_log_output(level, tag, buf);
}

// Bridge function: format the varargs message and call Rust log output
// This is the _log_write callback registered in wifi_osi_funcs_t
// NOTE: wifi_log in libnet80211.a always calls _log_writev first, then
// _log_write with the same arguments. We implement _log_write for completeness
// but it will produce duplicate output. The Rust log_write wrapper is a no-op
// to avoid this duplication.
void wifi_log_write_bridge(unsigned int level, const char *tag,
                           const char *format, ...) {
    va_list args;
    va_start(args, format);
    wifi_log_writev_bridge(level, tag, format, args);
    va_end(args);
}
