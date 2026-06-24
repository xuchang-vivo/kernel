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

// This code is based on [esp-radio] https://github.com/esp-rs/esp-hal/blob/main/esp-radio/src/wifi/os_adapter/mod.rs
// Copyright 2021 esp-rs
// https://github.com/esp-rs/esp-hal/blob/b0ea8c5b58aa66281c1325112219de737dc446d8/LICENSE-APACHE

use crate::{
    arch,
    boards::{efuse::read_mac_address, get_device, random_u32, Handler},
    scheduler::{self, wait_queue, InsertToEnd, WaitEntry},
    sync::{mqueue::MessageQueue, SpinLock},
    thread::{Entry, Stack, ThreadNode, SUSPENDED},
    time::Tick,
    types::{Arc, ThreadPriority},
    with_iou,
};
use alloc::boxed::Box;
use blueos_driver::interrupt_controller::Interrupt;
use core::{
    cell::UnsafeCell,
    ffi::{c_char, c_uint, c_void},
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use esp_radio_rtos_driver::{
    queue::{CompatQueue, QueueHandle, QueuePtr},
    register_queue_implementation, register_scheduler_implementation,
    register_semaphore_implementation, register_timer_implementation,
    register_wait_queue_implementation,
    semaphore::{
        CompatSemaphore, SemaphoreHandle, SemaphoreImplementation, SemaphoreKind, SemaphorePtr,
    },
    timer::{CompatTimer, TimerHandle, TimerPtr},
    wait_queue::{WaitQueueHandle, WaitQueueImplementation, WaitQueuePtr},
    SchedulerImplementation, ThreadPtr,
};
use esp_wifi_sys_esp32c3::include::{
    esp_event_base_t, ets_timer, timeval, OSI_FUNCS_TIME_BLOCKING,
};

use super::event::{EventInfo, WifiEvent};

extern "C" {
    static mut g_ic: u8;
}

fn wake_null_timer_addr() -> u32 {
    unsafe { core::ptr::addr_of!(g_ic).add(0x1dec) as u32 }
}

static WIFI_OS_WAIT_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static WIFI_OS_NOTIFY_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static WIFI_OS_SEM_PI_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static WIFI_OS_QUEUE_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static WIFI_OS_TASK_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static WIFI_OS_TIMER_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static WIFI_OS_TIMER_ARM_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static WIFI_TIMER_BACKEND_LOG_COUNT: AtomicU32 = AtomicU32::new(0);
static WIFI_OS_PP_QUEUE: AtomicU32 = AtomicU32::new(0);
static WIFI_OS_PENDING_YIELD: AtomicBool = AtomicBool::new(false);

const WIFI_OS_DIAG_LOG_ENABLED: bool = false;
const WIFI_TIMER_DIAG_LOG_ENABLED: bool = false;

fn wifi_os_diag_log_enabled() -> bool {
    WIFI_OS_DIAG_LOG_ENABLED
}

fn wifi_timer_diag_log_enabled() -> bool {
    WIFI_TIMER_DIAG_LOG_ENABLED
}

fn wifi_os_should_log(counter: &AtomicU32) -> Option<u32> {
    if !wifi_os_diag_log_enabled() {
        return None;
    }

    let count = counter.fetch_add(1, Ordering::Relaxed);
    if count < 64 || count.is_power_of_two() {
        Some(count)
    } else {
        None
    }
}

fn wifi_os_next_log_count(counter: &AtomicU32) -> u32 {
    counter.fetch_add(1, Ordering::Relaxed)
}

fn wifi_os_log_count_enabled(count: u32) -> bool {
    wifi_os_diag_log_enabled() && (count < 64 || count.is_power_of_two())
}

fn semaphore_kind_name(kind: &SemaphoreKind) -> &'static str {
    match kind {
        SemaphoreKind::Counting { .. } => "counting",
        SemaphoreKind::Mutex => "mutex",
        SemaphoreKind::RecursiveMutex => "recursive_mutex",
    }
}

#[no_mangle]
unsafe extern "Rust" fn blueos_wifi_timer_diag(event: u32, a: usize, b: u64, c: u64, d: u64) {
    if !wifi_timer_diag_log_enabled() {
        return;
    }

    let count = wifi_os_next_log_count(&WIFI_TIMER_BACKEND_LOG_COUNT);
    let should_log = wifi_os_log_count_enabled(count)
        || matches!(event, 10 | 11 | 12) && b >= 1_000_000
        || matches!(event, 5 | 6 | 7);
    if should_log {
        log::info!(
            "[WIFI_TIMER] diag#{} event={} a=0x{:08x} b={} c={} d={} now_us={}",
            count,
            event,
            a,
            b,
            c,
            d,
            Tick::now().as_micros(),
        );
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn gettimeofday(tv: *mut timeval, _tz: *mut ()) -> i32 {
    if !tv.is_null() {
        unsafe {
            let microseconds = Tick::now().as_micros();
            (*tv).tv_sec = (microseconds / 1_000_000) as u64;
            (*tv).tv_usec = (microseconds % 1_000_000) as u32;
        }
    }

    0
}

#[allow(unused)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn esp_event_post(
    event_base: *const c_char,
    event_id: i32,
    event_data: *mut c_void,
    event_data_size: usize,
    ticks_to_wait: u32,
) -> i32 {
    event_post(
        event_base,
        event_id,
        event_data,
        event_data_size,
        ticks_to_wait,
    )
}

#[no_mangle]
pub unsafe extern "C" fn esp_fill_random(dst: *mut u8, len: u32) {
    unsafe {
        let dst = core::slice::from_raw_parts_mut(dst, len as usize);

        crate::boards::random(dst);
    }
}

#[no_mangle]
pub(crate) unsafe extern "C" fn sleep(seconds: c_uint) -> c_uint {
    let target = Tick::after(Tick::from_millis(seconds as u64 * 1000));
    crate::scheduler::suspend_me_until::<()>(target, None);
    0
}

fn yield_me_now_or_later_irq_safe() {
    if crate::irq::is_in_irq() || arch::local_irq_enabled() {
        scheduler::yield_me_now_or_later();
    } else {
        WIFI_OS_PENDING_YIELD.store(true, Ordering::Release);
    }
}

pub(super) fn flush_pending_yield_if_safe() {
    if arch::local_irq_enabled() && WIFI_OS_PENDING_YIELD.swap(false, Ordering::AcqRel) {
        scheduler::yield_me_now_or_later();
    }
}

struct BkScheduler;

impl SchedulerImplementation for BkScheduler {
    fn initialized(&self) -> bool {
        crate::scheduler::is_schedule_ready()
    }

    fn yield_task(&self) {
        crate::scheduler::yield_me();
    }

    fn yield_task_from_isr(&self) {
        yield_me_now_or_later_irq_safe();
    }

    fn max_task_priority(&self) -> u32 {
        crate::config::MAX_THREAD_PRIORITY as u32
    }

    fn task_create(
        &self,
        _name: &str,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        priority: u32,
        _pin_to_core: Option<u32>,
        task_stack_size: usize,
    ) -> ThreadPtr {
        let entry = Entry::Posix(task, param);
        let stack = match Stack::from_size(task_stack_size) {
            Some(s) => s,
            None => return NonNull::dangling(),
        };
        // FreeRTOS priority is inverted relative to BlueOS: higher numeric value
        // means higher priority in FreeRTOS, but lower numeric value means higher
        // priority in BlueOS. Invert: blueos_prio = MAX - freertos_prio.
        let mut blueos_prio = crate::config::MAX_THREAD_PRIORITY
            .saturating_sub(priority.min(crate::config::MAX_THREAD_PRIORITY));
        if _name == "timer" {
            // The ESP timer service drives Wi-Fi scan dwell timeouts. If it stays
            // below the Wi-Fi task, a scan timer wake can leave the timer task READY
            // but unscheduled, stalling scan completion before association starts.
            blueos_prio = blueos_prio.min(1);
        }
        let thread = crate::thread::Builder::new(entry)
            .set_stack(stack)
            .set_priority(blueos_prio as ThreadPriority)
            .start();
        if let Some(count) = wifi_os_should_log(&WIFI_OS_TASK_LOG_COUNT) {
            log::info!(
                "[WIFI_OS] task_create#{} name={} task=0x{:08x} param={:p} freertos_prio={} blueos_prio={} stack={} thread=0x{:08x} now_us={}",
                count,
                _name,
                task as usize,
                param,
                priority,
                blueos_prio,
                task_stack_size,
                Arc::as_ptr(&thread) as usize,
                Tick::now().as_micros(),
            );
        }
        // into_raw consumes the Arc without decrementing refcount,
        // so the pointer retains ownership of one reference until
        // schedule_task_deletion reclaims it via Arc::from_raw.
        let ptr = Arc::into_raw(thread) as *mut ();
        NonNull::new(ptr).unwrap()
    }

    fn current_task(&self) -> ThreadPtr {
        let thread = crate::scheduler::current_thread();
        // as_ptr only borrows — the local Arc drops and refcount decrements,
        // but the thread is guaranteed alive (currently running).
        let ptr = Arc::as_ptr(&thread) as *mut ();
        NonNull::new(ptr).unwrap()
    }

    fn schedule_task_deletion(&self, task_handle: Option<ThreadPtr>) {
        let Some(handle) = task_handle else {
            // Delete self
            crate::scheduler::retire_me();
            return;
        };
        let current = crate::scheduler::current_thread_id();
        let target = handle.as_ptr() as usize;
        if current == target {
            crate::scheduler::retire_me();
        } else {
            let mut thread: Arc<crate::thread::Thread> =
                unsafe { Arc::from_raw(handle.as_ptr() as *const crate::thread::Thread) };
            if thread.state() == crate::thread::READY {
                crate::scheduler::remove_from_ready_queue(&thread);
            }
            crate::thread::GlobalQueueVisitor::remove(&mut thread);
        }
    }

    fn current_task_thread_semaphore(&self) -> SemaphorePtr {
        let thread = crate::scheduler::current_thread();
        let mut w = thread.lock();
        if let Some(ptr) = w.get_alien_ptr() {
            NonNull::new(ptr.as_ptr() as *mut ()).unwrap()
        } else {
            let sem = SemaphoreHandle::new(SemaphoreKind::Counting { max: 1, initial: 0 }).leak();
            w.set_alien_ptr(NonNull::new(sem.as_ptr().cast()).unwrap());
            sem
        }
    }

    unsafe fn task_priority(&self, task: ThreadPtr) -> u32 {
        let thread = &*(task.as_ptr() as *const crate::thread::Thread);
        let blueos_prio = thread.priority();
        // Invert: freertos_prio = MAX - blueos_prio
        crate::config::MAX_THREAD_PRIORITY.saturating_sub(blueos_prio)
    }

    unsafe fn set_task_priority(&self, task: ThreadPtr, priority: u32) {
        let thread = &mut *(task.as_ptr() as *mut crate::thread::Thread);
        let blueos_prio = crate::config::MAX_THREAD_PRIORITY
            .saturating_sub(priority.min(crate::config::MAX_THREAD_PRIORITY));
        thread.set_priority(blueos_prio as ThreadPriority);
    }

    fn usleep(&self, us: u32) {
        let _ = crate::scheduler::suspend_me_for::<()>(Tick::from_micros(us as u64), None);
    }

    fn usleep_until(&self, target: u64) {
        let deadline = Tick::from_micros(target);
        let _ = crate::scheduler::suspend_me_until::<()>(deadline, None);
    }

    fn now(&self) -> u64 {
        Tick::now().as_micros()
    }
}

/// Wrapper around a spinlocked kernel WaitQueue for use with the esp-radio driver.
///
/// The kernel's `WaitQueue` (`Ilist<WaitEntry, OffsetOfWait>`) is an intrusive linked list.
/// We wrap it in a `SpinLock` for thread safety, heap-allocate it, and expose it as an opaque
/// `WaitQueuePtr` to the esp-radio layer.
struct EspWaitQueue(SpinLock<wait_queue::WaitQueue>);

impl WaitQueueImplementation for EspWaitQueue {
    fn create() -> WaitQueuePtr {
        let wq = Box::new(EspWaitQueue(SpinLock::new(wait_queue::WaitQueue::new())));
        wq.0.irqsave_lock().init();
        let ptr = Box::into_raw(wq);
        NonNull::new(ptr as *mut ()).unwrap()
    }

    unsafe fn delete(queue: WaitQueuePtr) {
        let ptr = queue.as_ptr() as *mut EspWaitQueue;
        let _ = Box::from_raw(ptr);
    }

    unsafe fn wait_until(queue: WaitQueuePtr, deadline_instant: Option<u64>) {
        let this = &mut *(queue.as_ptr() as *mut EspWaitQueue);
        let this_thread = scheduler::current_thread();
        let deadline = deadline_instant.map(Tick::from_micros).unwrap_or(Tick::MAX);
        if let Some(count) = wifi_os_should_log(&WIFI_OS_WAIT_LOG_COUNT) {
            log::info!(
                "[WIFI_OS] wait_enter#{} queue={:p} deadline_us={:?} now_us={}",
                count,
                queue.as_ptr(),
                deadline_instant,
                Tick::now().as_micros(),
            );
        }
        let mut w = this.0.irqsave_lock();
        with_iou!(|borrowed_wait_entry| {
            let mut wait_entry = WaitEntry::new(this_thread.clone());
            borrowed_wait_entry =
                wait_queue::insert(&mut w, &mut wait_entry, InsertToEnd::MODE).unwrap();
            let _ = scheduler::suspend_me_until(deadline, Some(w));
            w = this.0.irqsave_lock();
            borrowed_wait_entry = w.pop(borrowed_wait_entry).unwrap();
        });
        if let Some(count) = wifi_os_should_log(&WIFI_OS_WAIT_LOG_COUNT) {
            log::info!(
                "[WIFI_OS] wait_exit#{} queue={:p} deadline_us={:?} now_us={}",
                count,
                queue.as_ptr(),
                deadline_instant,
                Tick::now().as_micros(),
            );
        }
    }

    unsafe fn notify(queue: WaitQueuePtr) {
        let this = &*(queue.as_ptr() as *const EspWaitQueue);
        let mut w = this.0.irqsave_lock();
        let mut woke = false;
        let mut waiters = 0usize;
        let mut first_thread = 0usize;
        let mut first_state_before = u8::MAX;
        let mut first_state_after = u8::MAX;
        for entry in w.iter() {
            waiters += 1;
            if waiters == 1 {
                let t = entry.thread.clone();
                first_thread = Arc::as_ptr(&t) as usize;
                first_state_before = t.state();
                woke = scheduler::queue_ready_thread(SUSPENDED, t.clone()).is_ok();
                first_state_after = t.state();
            }
        }
        drop(w);
        let count = wifi_os_next_log_count(&WIFI_OS_NOTIFY_LOG_COUNT);
        let schedule_ready = scheduler::is_schedule_ready();
        let current = scheduler::current_thread();
        let current_thread = Arc::as_ptr(&current) as usize;
        let current_prio = current.priority();
        let current_preempt = current.preempt_count();
        let current_state = current.state();
        if wifi_os_diag_log_enabled() && (wifi_os_log_count_enabled(count) || waiters > 0) {
            log::info!(
                "[WIFI_OS] notify#{} queue={:p} waiters={} first=0x{:08x} state_before={} state_after={} woke={} sched={} cur=0x{:08x} cur_state={} cur_prio={} cur_preempt={} irq={} now_us={}",
                count,
                queue.as_ptr(),
                waiters,
                first_thread,
                first_state_before,
                first_state_after,
                woke,
                schedule_ready,
                current_thread,
                current_state,
                current_prio,
                current_preempt,
                arch::local_irq_enabled(),
                Tick::now().as_micros(),
            );
        }
        if woke {
            yield_me_now_or_later_irq_safe();
            if wifi_os_diag_log_enabled() {
                log::info!(
                    "[WIFI_OS] notify_yield_done#{} queue={:p} first=0x{:08x} first_state_after_yield={} cur=0x{:08x} cur_preempt={} irq={} now_us={}",
                    count,
                    queue.as_ptr(),
                    first_thread,
                    if first_thread != 0 {
                        (&*(first_thread as *const crate::thread::Thread)).state()
                    } else {
                        u8::MAX
                    },
                    scheduler::current_thread_id(),
                    scheduler::current_thread_ref().preempt_count(),
                    arch::local_irq_enabled(),
                    Tick::now().as_micros(),
                );
            }
        }
    }

    unsafe fn notify_from_isr(queue: WaitQueuePtr, mut higher_prio_task_waken: Option<&mut bool>) {
        let this = &*(queue.as_ptr() as *const EspWaitQueue);
        let mut w = this.0.irqsave_lock();
        let mut woke = false;
        let has_hptw = higher_prio_task_waken.is_some();
        let mut waiters = 0usize;
        let mut first_thread = 0usize;
        let mut first_state_before = u8::MAX;
        let mut first_state_after = u8::MAX;
        for entry in w.iter() {
            waiters += 1;
            if waiters == 1 {
                let t = entry.thread.clone();
                first_thread = Arc::as_ptr(&t) as usize;
                first_state_before = t.state();
                if scheduler::queue_ready_thread(SUSPENDED, t.clone()).is_ok() {
                    woke = true;
                    if let Some(hptw) = higher_prio_task_waken.as_mut() {
                        **hptw = true;
                    }
                }
                first_state_after = t.state();
            }
        }
        drop(w);
        let count = wifi_os_next_log_count(&WIFI_OS_NOTIFY_LOG_COUNT);
        if wifi_os_diag_log_enabled() && (wifi_os_log_count_enabled(count) || waiters > 0) {
            log::info!(
                "[WIFI_OS] notify_from_isr#{} queue={:p} waiters={} first=0x{:08x} state_before={} state_after={} woke={} hptw_ptr={} now_us={}",
                count,
                queue.as_ptr(),
                waiters,
                first_thread,
                first_state_before,
                first_state_after,
                woke,
                has_hptw,
                Tick::now().as_micros(),
            );
        }
    }
}

/// BkSemaphore: a semaphore implementation using short interrupt-disabled critical
/// sections instead of NonReentrantMutex. This avoids the "lock is not reentrant"
/// panic that occurs when CompatSemaphore's NonReentrantMutex is held across a
/// context switch inside EspWaitQueue::wait_until.
///
/// On single-core, saving and disabling local interrupts is sufficient for mutual
/// exclusion. Blocking waits must happen outside these critical sections so the
/// scheduler sees local interrupts enabled when suspending and resuming threads.
struct BkSemaphore {
    data: UnsafeCell<SemaphoreBkData>,
}

struct SemaphoreBkData {
    kind: SemaphoreKind,
    current: u32,
    max: u32,
    waiting: WaitQueuePtr,
    owner: usize,
    owner_thread: Option<ThreadNode>,
    owner_boosted: bool,
    recursion: u32,
}

unsafe impl Sync for BkSemaphore {}

impl BkSemaphore {
    fn new(kind: SemaphoreKind) -> Self {
        let (current, max) = match kind {
            SemaphoreKind::Counting { max, initial } => (initial, max),
            SemaphoreKind::Mutex | SemaphoreKind::RecursiveMutex => (1, 1),
        };
        BkSemaphore {
            data: UnsafeCell::new(SemaphoreBkData {
                kind,
                current,
                max,
                waiting: unsafe { EspWaitQueue::create() },
                owner: 0,
                owner_thread: None,
                owner_boosted: false,
                recursion: 0,
            }),
        }
    }

    /// Enter a short critical section, run `f`, then restore the full saved IRQ state.
    fn with_irq_safe<R>(&self, f: impl FnOnce(&mut SemaphoreBkData) -> R) -> R {
        let irq_level = arch::disable_local_irq_save();
        let r = f(unsafe { &mut *self.data.get() });
        arch::enable_local_irq_restore(irq_level);
        r
    }

    fn is_mutex_kind(kind: &SemaphoreKind) -> bool {
        matches!(kind, SemaphoreKind::Mutex | SemaphoreKind::RecursiveMutex)
    }

    fn promote_owner_for_waiter(
        semaphore: SemaphorePtr,
        kind: &'static str,
        owner_thread: &ThreadNode,
        waiter_thread: &ThreadNode,
    ) -> bool {
        let target_priority = waiter_thread.priority();
        let old_priority = owner_thread.priority();
        let owner_state = owner_thread.state();
        let mut promoted = false;
        let mut rq_updated = false;
        let mut rq_state = owner_state;

        if target_priority < old_priority {
            match scheduler::update_ready_thread_priority(owner_thread, target_priority) {
                Ok(()) => {
                    promoted = true;
                    rq_updated = true;
                    rq_state = SUSPENDED;
                }
                Err(state) => {
                    rq_state = state;
                    promoted = owner_thread.lock().promote_priority_to(target_priority);
                }
            }
        }

        let count = wifi_os_next_log_count(&WIFI_OS_SEM_PI_LOG_COUNT);
        if wifi_os_diag_log_enabled() && (promoted || wifi_os_log_count_enabled(count)) {
            log::info!(
                "[WIFI_OS] sem_pi_promote#{} sem={:p} kind={} owner=0x{:08x} waiter=0x{:08x} owner_state={} rq_state={} old_prio={} target_prio={} new_prio={} promoted={} rq_updated={} now_us={}",
                count,
                semaphore.as_ptr(),
                kind,
                Arc::as_ptr(owner_thread) as usize,
                Arc::as_ptr(waiter_thread) as usize,
                owner_state,
                rq_state,
                old_priority,
                target_priority,
                owner_thread.priority(),
                promoted,
                rq_updated,
                Tick::now().as_micros(),
            );
        }

        promoted
    }

    fn recover_owner_priority(
        semaphore: SemaphorePtr,
        kind: &'static str,
        owner_thread: &ThreadNode,
    ) -> bool {
        let old_priority = owner_thread.priority();
        let origin_priority = owner_thread.origin_priority();
        let owner_state = owner_thread.state();
        let mut recovered = false;
        let mut rq_updated = false;
        let mut rq_state = owner_state;

        if old_priority != origin_priority {
            match scheduler::update_ready_thread_priority(owner_thread, origin_priority) {
                Ok(()) => {
                    recovered = true;
                    rq_updated = true;
                    rq_state = SUSPENDED;
                }
                Err(state) => {
                    rq_state = state;
                    owner_thread.lock().recover_priority();
                    recovered = owner_thread.priority() != old_priority;
                }
            }
        }

        let count = wifi_os_next_log_count(&WIFI_OS_SEM_PI_LOG_COUNT);
        if wifi_os_diag_log_enabled() && (recovered || wifi_os_log_count_enabled(count)) {
            log::info!(
                "[WIFI_OS] sem_pi_recover#{} sem={:p} kind={} owner=0x{:08x} owner_state={} rq_state={} old_prio={} origin_prio={} new_prio={} recovered={} rq_updated={} now_us={}",
                count,
                semaphore.as_ptr(),
                kind,
                Arc::as_ptr(owner_thread) as usize,
                owner_state,
                rq_state,
                old_priority,
                origin_priority,
                owner_thread.priority(),
                recovered,
                rq_updated,
                Tick::now().as_micros(),
            );
        }

        recovered
    }
}

impl SemaphoreImplementation for BkSemaphore {
    fn create(kind: SemaphoreKind) -> SemaphorePtr {
        let sem = Box::new(Self::new(kind));
        NonNull::from(Box::leak(sem)).cast()
    }

    unsafe fn delete(semaphore: SemaphorePtr) {
        let sem = Box::from_raw(semaphore.cast::<Self>().as_ptr());
        // Delete the wait queue — no lock needed, nobody references this any more.
        EspWaitQueue::delete(sem.with_irq_safe(|d| d.waiting));
        drop(sem);
    }

    unsafe fn take(semaphore: SemaphorePtr, timeout_us: Option<u32>) -> bool {
        <Self as SemaphoreImplementation>::take_with_deadline(
            semaphore,
            timeout_us.map(|us| Tick::now().as_micros() + us as u64),
        )
    }

    unsafe fn take_with_deadline(semaphore: SemaphorePtr, _deadline_instant: Option<u64>) -> bool {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        loop {
            // Quick path: try to decrement current under a short critical section.
            // If semaphore is available, take it and return immediately.
            // If we need to block, the wait happens after with_irq_safe restores the
            // saved IRQ state.
            let current_thread = scheduler::current_thread();
            let current_owner = Arc::as_ptr(&current_thread) as usize;
            let (available, waiting, current, max, kind, is_mutex, owner, recursion, owner_thread) =
                sem.with_irq_safe(|data| {
                    let available = match &data.kind {
                        SemaphoreKind::RecursiveMutex if data.owner == current_owner => {
                            data.recursion = data.recursion.saturating_add(1);
                            true
                        }
                        SemaphoreKind::Mutex | SemaphoreKind::RecursiveMutex if data.current > 0 => {
                            data.current -= 1;
                            data.owner = current_owner;
                            data.owner_thread = Some(current_thread.clone());
                            data.owner_boosted = false;
                            data.recursion = 1;
                            true
                        }
                        _ if data.current > 0 => {
                            data.current -= 1;
                            true
                        }
                        _ => false,
                    };
                    (
                        available,
                        data.waiting,
                        data.current,
                        data.max,
                        semaphore_kind_name(&data.kind),
                        Self::is_mutex_kind(&data.kind),
                        data.owner,
                        data.recursion,
                        data.owner_thread.clone(),
                    )
                });

            if available {
                return true;
            }

            if is_mutex && owner != 0 && owner != current_owner {
                if let Some(owner_thread) = owner_thread.as_ref() {
                    if Self::promote_owner_for_waiter(semaphore, kind, owner_thread, &current_thread) {
                        sem.with_irq_safe(|data| data.owner_boosted = true);
                    }
                }
            }

            // Semaphore not available — need to block.
            // wait_until handles insert + suspend + pop atomically (via with_iou!).
            // ESP Wi-Fi may call this while its upstream critical section has local
            // IRQ disabled. BlueOS cannot context-switch with local IRQ disabled, so
            // bridge the two contracts: save the caller's IRQ state, enable IRQ only
            // for the blocking wait, then restore the saved state after wait returns.
            if _deadline_instant.is_some_and(|d| Tick::now().as_micros() >= d) {
                return false;
            }
            let irq_level = arch::disable_local_irq_save();
            arch::enable_local_irq();
            // SAFETY: `waiting` pointer is immutable after new() — it is only
            // read here and in give().  `waiting` was already captured under the
            // guard above, so we use the local copy.
            unsafe {
                EspWaitQueue::wait_until(waiting, _deadline_instant);
            }
            arch::enable_local_irq_restore(irq_level);
        }
    }

    unsafe fn give(semaphore: SemaphorePtr) -> bool {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        let current_thread = scheduler::current_thread();
        let current_owner = Arc::as_ptr(&current_thread) as usize;
        let (ok, released, waiting, current, max, kind, owner, recursion, owner_thread, owner_boosted) =
            sem.with_irq_safe(|data| {
                let mut released_owner_thread = None;
                let mut released_owner_boosted = false;
                let (ok, released) = match &data.kind {
                    SemaphoreKind::Mutex | SemaphoreKind::RecursiveMutex
                        if data.owner == current_owner =>
                    {
                        if matches!(&data.kind, SemaphoreKind::RecursiveMutex) && data.recursion > 1 {
                            data.recursion -= 1;
                            (true, false)
                        } else {
                            data.recursion = 0;
                            data.owner = 0;
                            released_owner_thread = data.owner_thread.take();
                            released_owner_boosted = data.owner_boosted;
                            data.owner_boosted = false;
                            if data.current < data.max {
                                data.current += 1;
                            }
                            (true, true)
                        }
                    }
                    SemaphoreKind::Mutex | SemaphoreKind::RecursiveMutex => (false, false),
                    _ if data.current < data.max => {
                        data.current += 1;
                        (true, true)
                    }
                    _ => (false, false),
                };
                (
                    ok,
                    released,
                    data.waiting,
                    data.current,
                    data.max,
                    semaphore_kind_name(&data.kind),
                    data.owner,
                    data.recursion,
                    released_owner_thread,
                    released_owner_boosted,
                )
            });
        if released && owner_boosted {
            if let Some(owner_thread) = owner_thread.as_ref() {
                Self::recover_owner_priority(semaphore, kind, owner_thread);
            }
        }
        if released {
            unsafe { EspWaitQueue::notify(waiting) };
        }
        ok
    }

    unsafe fn try_give_from_isr(
        semaphore: SemaphorePtr,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        let has_hptw = higher_prio_task_waken.is_some();
        let (ok, waiting, current, max, kind, owner, recursion) = sem.with_irq_safe(|data| {
            let ok = match &data.kind {
                SemaphoreKind::Mutex | SemaphoreKind::RecursiveMutex => false,
                _ if data.current < data.max => {
                    data.current += 1;
                    true
                }
                _ => false,
            };
            (
                ok,
                data.waiting,
                data.current,
                data.max,
                semaphore_kind_name(&data.kind),
                data.owner,
                data.recursion,
            )
        });
        if ok {
            unsafe { EspWaitQueue::notify_from_isr(waiting, higher_prio_task_waken) };
        }
        ok
    }

    unsafe fn current_count(semaphore: SemaphorePtr) -> u32 {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        sem.with_irq_safe(|data| data.current)
    }

    unsafe fn try_take(semaphore: SemaphorePtr) -> bool {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        let current_thread = scheduler::current_thread();
        let current_owner = Arc::as_ptr(&current_thread) as usize;
        sem.with_irq_safe(|data| match &data.kind {
            SemaphoreKind::RecursiveMutex if data.owner == current_owner => {
                data.recursion = data.recursion.saturating_add(1);
                true
            }
            SemaphoreKind::Mutex | SemaphoreKind::RecursiveMutex if data.current > 0 => {
                data.current -= 1;
                data.owner = current_owner;
                data.owner_thread = Some(current_thread.clone());
                data.owner_boosted = false;
                data.recursion = 1;
                true
            }
            _ if data.current > 0 => {
                data.current -= 1;
                true
            }
            _ => false,
        })
    }

    unsafe fn try_take_from_isr(
        semaphore: SemaphorePtr,
        _higher_prio_task_waken: Option<&mut bool>,
    ) -> bool {
        <Self as SemaphoreImplementation>::try_take(semaphore)
    }
}

register_semaphore_implementation!(BkSemaphore);
register_queue_implementation!(CompatQueue);
register_timer_implementation!(CompatTimer);
register_wait_queue_implementation!(EspWaitQueue);
register_scheduler_implementation!(static SCHEDULER: BkScheduler = BkScheduler);

pub static ISR_INTERRUPT_1: Handler = Handler::new();

static WIFI_PWR_INTERRUPT: Interrupt = Interrupt::new(2, 1);
static WIFI_MAC_INTERRUPT: Interrupt = Interrupt::new(0, 1);

pub unsafe extern "C" fn env_is_chip() -> bool {
    true
}

pub unsafe extern "C" fn set_intr(cpu_no: i32, intr_source: u32, intr_num: u32, intr_prio: i32) {
    let intr = Interrupt::new(intr_source as _, intr_num as _);
    get_device!(intc).allocate_irq(intr);
    get_device!(intc).set_priority(intr, intr_prio as _);
}

/// Don't support
pub unsafe extern "C" fn clear_intr(intr_source: u32, intr_num: u32) {}

pub unsafe extern "C" fn set_isr(n: i32, f: *mut c_void, arg: *mut c_void) {
    match n {
        0 | 1 => ISR_INTERRUPT_1.set(f, arg),
        _ => panic!("set_isr - unsupported interrupt number {}", n),
    }

    get_device!(intc).enable_irq(WIFI_PWR_INTERRUPT);
    get_device!(intc).enable_irq(WIFI_MAC_INTERRUPT);
}

pub unsafe extern "C" fn ints_on(mask: u32) {
    let tmp = core::ptr::read_volatile(0x600C2104 as *const u32);
    core::ptr::write_volatile(0x600C2104 as *mut u32, tmp | mask);
}

pub unsafe extern "C" fn ints_off(mask: u32) {
    let tmp = core::ptr::read_volatile(0x600C2104 as *const u32);
    core::ptr::write_volatile(0x600C2104 as *mut u32, tmp & !mask);
}

pub unsafe extern "C" fn is_from_isr() -> bool {
    true
}

pub unsafe extern "C" fn spin_lock_create() -> *mut c_void {
    semphr_create(1, 1)
}

/// INTC `cpu_int_enable_reg` offset for ESP32-C3.
const INTC_CPU_INT_ENABLE: *mut u32 = (0x600c_2000 + 0x104) as *mut u32;

/// Bitmask for the two wifi interrupt sources (interrupts 0 and 1, which
/// correspond to mcause bits 0|1 in `handle_intc_irq`).
const WIFI_IRQ_MASK: u32 = 0b11;

/// Re-implementation of `wifi_int_disable` that masks only the wifi interrupt
/// sources (INTC IRQ 0 and 1) instead of globally disabling MIE.
///
/// This avoids the MIE=0 leak through `take_with_deadline` →
/// `suspend_me_until` → `debug_assert!(arch::local_irq_enabled())` crash when
/// the async handler calls C APIs (e.g. `esp_wifi_scan_get_ap_records`) that
/// internally hold this lock.
pub unsafe extern "C" fn wifi_int_disable(_wifi_int_mux: *mut c_void) -> u32 {
    let old = unsafe { INTC_CPU_INT_ENABLE.read_volatile() };
    unsafe { INTC_CPU_INT_ENABLE.write_volatile(old & !WIFI_IRQ_MASK) };
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    if wifi_os_diag_log_enabled() {
        log::trace!(
            "wifi_int_disable: INTC enable reg {:#x} -> {:#x}",
            old,
            old & !WIFI_IRQ_MASK
        );
    }
    old
}

pub unsafe extern "C" fn wifi_int_restore(_wifi_int_mux: *mut c_void, tmp: u32) {
    let old = unsafe { INTC_CPU_INT_ENABLE.read_volatile() };
    let new = (old & !WIFI_IRQ_MASK) | (tmp & WIFI_IRQ_MASK);
    unsafe { INTC_CPU_INT_ENABLE.write_volatile(new) };
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    if wifi_os_diag_log_enabled() {
        log::trace!("wifi_int_restore: INTC enable reg {:#x} -> {:#x}", old, new);
    }
    flush_pending_yield_if_safe();
}

pub unsafe extern "C" fn task_yield_from_isr() {
    yield_me_now_or_later_irq_safe();
}

pub unsafe extern "C" fn spin_lock_delete(lock: *mut c_void) {
    semphr_delete(lock);
}

pub unsafe extern "C" fn semphr_create(max: u32, init: u32) -> *mut c_void {
    let ptr = SemaphoreHandle::new(SemaphoreKind::Counting { max, initial: init })
        .leak()
        .as_ptr()
        .cast::<c_void>();
    ptr
}

pub unsafe extern "C" fn semphr_delete(semphr: *mut c_void) {
    if !semphr.is_null() {
        let ptr = SemaphorePtr::new(semphr.cast()).expect("invalid semaphore pointer");
        let handle = SemaphoreHandle::from_ptr(ptr);
        core::mem::drop(handle);
    }
}

pub unsafe extern "C" fn semphr_take(semphr: *mut c_void, block_time_tick: u32) -> i32 {
    if !semphr.is_null() {
        let ptr = SemaphorePtr::new(semphr.cast()).expect("invalid semaphore pointer");
        let handle = SemaphoreHandle::ref_from_ptr(&ptr);
        let timeout = if block_time_tick == OSI_FUNCS_TIME_BLOCKING {
            None
        } else {
            Some(block_time_tick)
        };

        handle.take(timeout) as i32
    } else {
        0
    }
}

pub unsafe extern "C" fn semphr_give(semphr: *mut c_void) -> i32 {
    if !semphr.is_null() {
        let ptr = SemaphorePtr::new(semphr.cast()).expect("invalid semaphore pointer");
        let handle = SemaphoreHandle::ref_from_ptr(&ptr);
        handle.give() as i32
    } else {
        0
    }
}

pub unsafe extern "C" fn wifi_thread_semphr_get() -> *mut c_void {
    esp_radio_rtos_driver::current_task_thread_semaphore()
        .as_ptr()
        .cast::<c_void>()
}

pub unsafe extern "C" fn mutex_create() -> *mut c_void {
    let ptr = SemaphoreHandle::new(SemaphoreKind::Mutex)
        .leak()
        .as_ptr()
        .cast();
    ptr
}

pub unsafe extern "C" fn recursive_mutex_create() -> *mut c_void {
    let ptr = SemaphoreHandle::new(SemaphoreKind::RecursiveMutex)
        .leak()
        .as_ptr()
        .cast();
    ptr
}

pub unsafe extern "C" fn mutex_delete(mutex: *mut c_void) {
    if !mutex.is_null() {
        let ptr = SemaphorePtr::new(mutex.cast()).expect("invalid mutex pointer");
        let handle = SemaphoreHandle::from_ptr(ptr);
        core::mem::drop(handle);
    }
}

pub unsafe extern "C" fn mutex_lock(mutex: *mut c_void) -> i32 {
    if !mutex.is_null() {
        let ptr = SemaphorePtr::new(mutex.cast()).expect("invalid mutex pointer");
        let handle = SemaphoreHandle::ref_from_ptr(&ptr);
        handle.take(None) as i32
    } else {
        0
    }
}

pub unsafe extern "C" fn mutex_unlock(mutex: *mut c_void) -> i32 {
    if !mutex.is_null() {
        let ptr = SemaphorePtr::new(mutex.cast()).expect("invalid mutex pointer");
        let handle = SemaphoreHandle::ref_from_ptr(&ptr);
        handle.give() as i32
    } else {
        0
    }
}

pub unsafe extern "C" fn queue_create(queue_len: u32, item_size: u32) -> *mut c_void {
    let queue = QueueHandle::new(queue_len as usize, item_size as usize)
        .leak()
        .as_ptr()
        .cast::<c_void>();
    let is_pp_queue = queue_len == 200 && item_size == 8;
    if is_pp_queue {
        WIFI_OS_PP_QUEUE.store(queue as u32, Ordering::Relaxed);
    }
    if wifi_os_diag_log_enabled() {
        log::info!(
            "[WIFI_OS] queue_create queue={:p} len={} item_size={} pp_queue={} now_us={}",
            queue,
            queue_len,
            item_size,
            is_pp_queue,
            Tick::now().as_micros(),
        );
    }
    queue
}

pub unsafe extern "C" fn queue_delete(queue: *mut c_void) {
    let ptr = QueuePtr::new(queue.cast()).expect("invalid queue pointer");

    let handle = unsafe { QueueHandle::from_ptr(ptr) };
    core::mem::drop(handle);
}

pub unsafe extern "C" fn queue_send(
    queue: *mut c_void,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    queue_send_to_back(queue, item, block_time_tick)
}

fn is_pp_queue(queue: *mut c_void) -> bool {
    let pp_queue = WIFI_OS_PP_QUEUE.load(Ordering::Relaxed) as *mut c_void;
    !pp_queue.is_null() && queue == pp_queue
}

unsafe fn log_pp_queue_item(tag: &str, queue: *mut c_void, item: *mut c_void, ret: i32) {
    if !wifi_os_diag_log_enabled() {
        return;
    }

    if is_pp_queue(queue) && !item.is_null() {
        let msg0 = unsafe { core::ptr::read_unaligned(item.cast::<u8>()) };
        let word0 = unsafe { core::ptr::read_unaligned(item.cast::<u32>()) };
        let word1 = unsafe { core::ptr::read_unaligned(item.cast::<u8>().add(4).cast::<u32>()) };
        if msg0 == 7 && word1 != 0 {
            let nested = word1 as *const u8;
            let nested_msg0 = unsafe { core::ptr::read_unaligned(nested) };
            let nested_word0 = unsafe { core::ptr::read_unaligned(nested.cast::<u32>()) };
            let nested_word1 = unsafe { core::ptr::read_unaligned(nested.add(4).cast::<u32>()) };
            log::info!(
                "[WIFI_OS] pp_queue_{} queue={:p} item={:p} ret={} msg0={} word0=0x{:08x} word1=0x{:08x} nested_msg0={} nested_word0=0x{:08x} nested_word1=0x{:08x} now_us={}",
                tag,
                queue,
                item,
                ret,
                msg0,
                word0,
                word1,
                nested_msg0,
                nested_word0,
                nested_word1,
                Tick::now().as_micros(),
            );
        } else {
            log::info!(
                "[WIFI_OS] pp_queue_{} queue={:p} item={:p} ret={} msg0={} word0=0x{:08x} word1=0x{:08x} now_us={}",
                tag,
                queue,
                item,
                ret,
                msg0,
                word0,
                word1,
                Tick::now().as_micros(),
            );
        }
    }
}

pub unsafe extern "C" fn queue_send_from_isr(
    queue: *mut c_void,
    item: *mut c_void,
    hptw: *mut c_void,
) -> i32 {
    if !queue.is_null() {
        let ptr = QueuePtr::new(queue.cast()).expect("invalid queue pointer");
        let handle = unsafe { QueueHandle::ref_from_ptr(&ptr) };
        let ret = handle.try_send_to_back_from_isr(item.cast(), (hptw as *mut bool).as_mut()) as i32;
        unsafe { log_pp_queue_item("send_isr", queue, item, ret) };
        if let Some(count) = wifi_os_should_log(&WIFI_OS_QUEUE_LOG_COUNT) {
            log::info!(
                "[WIFI_OS] queue_send_from_isr#{} queue={:p} item={:p} hptw={:p} ret={} now_us={}",
                count,
                queue,
                item,
                hptw,
                ret,
                Tick::now().as_micros(),
            );
        }
        ret
    } else {
        0
    }
}

pub unsafe extern "C" fn queue_send_to_back(
    queue: *mut c_void,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    if !queue.is_null() {
        let ptr = QueuePtr::new(queue.cast()).expect("invalid queue pointer");
        let handle = unsafe { QueueHandle::ref_from_ptr(&ptr) };
        let timeout = if block_time_tick == OSI_FUNCS_TIME_BLOCKING {
            None
        } else {
            Some(block_time_tick)
        };

        let before_waiting = if is_pp_queue(queue) {
            Some(handle.messages_waiting())
        } else {
            None
        };
        let ret = handle.send_to_back(item.cast(), timeout) as i32;
        if wifi_os_diag_log_enabled() {
            if let Some(before_waiting) = before_waiting {
                let after_waiting = handle.messages_waiting();
                log::info!(
                    "[WIFI_OS] pp_queue_send_count queue={:p} ret={} waiting_before={} waiting_after={} now_us={}",
                    queue,
                    ret,
                    before_waiting,
                    after_waiting,
                    Tick::now().as_micros(),
                );
            }
        }
        unsafe { log_pp_queue_item("send", queue, item, ret) };
        if let Some(count) = wifi_os_should_log(&WIFI_OS_QUEUE_LOG_COUNT) {
            log::info!(
                "[WIFI_OS] queue_send_to_back#{} queue={:p} item={:p} ticks={} timeout_us={:?} ret={} now_us={}",
                count,
                queue,
                item,
                block_time_tick,
                timeout,
                ret,
                Tick::now().as_micros(),
            );
        }
        ret
    } else {
        0
    }
}

pub unsafe extern "C" fn queue_send_to_front(
    queue: *mut c_void,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    if !queue.is_null() {
        let ptr = QueuePtr::new(queue.cast()).expect("invalid queue pointer");
        let handle = unsafe { QueueHandle::ref_from_ptr(&ptr) };
        let timeout = if block_time_tick == OSI_FUNCS_TIME_BLOCKING {
            None
        } else {
            Some(block_time_tick)
        };

        handle.send_to_front(item.cast(), timeout) as i32
    } else {
        0
    }
}

pub unsafe extern "C" fn queue_recv(
    queue: *mut c_void,
    item: *mut c_void,
    block_time_tick: u32,
) -> i32 {
    if !queue.is_null() {
        let ptr = QueuePtr::new(queue.cast()).expect("invalid queue pointer");
        let handle = unsafe { QueueHandle::ref_from_ptr(&ptr) };
        let timeout = if block_time_tick == OSI_FUNCS_TIME_BLOCKING {
            None
        } else {
            Some(block_time_tick)
        };

        let before_waiting = if is_pp_queue(queue) {
            Some(handle.messages_waiting())
        } else {
            None
        };
        if wifi_os_diag_log_enabled() {
            if let Some(before_waiting) = before_waiting {
                log::info!(
                    "[WIFI_OS] pp_queue_recv_enter queue={:p} item={:p} waiting_before={} timeout_us={:?} now_us={}",
                    queue,
                    item,
                    before_waiting,
                    timeout,
                    Tick::now().as_micros(),
                );
            }
        }
        let ret = handle.receive(item.cast(), timeout) as i32;
        if wifi_os_diag_log_enabled() {
            if let Some(before_waiting) = before_waiting {
                let after_waiting = handle.messages_waiting();
                log::info!(
                    "[WIFI_OS] pp_queue_recv_count queue={:p} item={:p} ret={} waiting_before={} waiting_after={} now_us={}",
                    queue,
                    item,
                    ret,
                    before_waiting,
                    after_waiting,
                    Tick::now().as_micros(),
                );
            }
        }
        unsafe { log_pp_queue_item("recv", queue, item, ret) };
        if let Some(count) = wifi_os_should_log(&WIFI_OS_QUEUE_LOG_COUNT) {
            log::info!(
                "[WIFI_OS] queue_recv#{} queue={:p} item={:p} ticks={} timeout_us={:?} ret={} now_us={}",
                count,
                queue,
                item,
                block_time_tick,
                timeout,
                ret,
                Tick::now().as_micros(),
            );
        }
        ret
    } else {
        0
    }
}

pub unsafe extern "C" fn queue_msg_waiting(queue: *mut c_void) -> u32 {
    if !queue.is_null() {
        let ptr = QueuePtr::new(queue.cast()).expect("invalid queue pointer");
        let handle = unsafe { QueueHandle::ref_from_ptr(&ptr) };
        handle.messages_waiting() as u32
    } else {
        0
    }
}

pub unsafe extern "C" fn event_group_create() -> *mut c_void {
    log::error!("wifi_os_event_group_create called");
    todo!("event_group_create")
}

pub unsafe extern "C" fn event_group_delete(event: *mut c_void) {
    log::error!("wifi_os_event_group_delete called: event={:p}", event);
    todo!("event_group_delete")
}

pub unsafe extern "C" fn event_group_set_bits(event: *mut c_void, bits: u32) -> u32 {
    log::error!(
        "wifi_os_event_group_set_bits called: event={:p} bits=0x{:08x}",
        event,
        bits,
    );
    todo!("event_group_set_bits")
}

pub unsafe extern "C" fn event_group_clear_bits(event: *mut c_void, bits: u32) -> u32 {
    log::error!(
        "wifi_os_event_group_clear_bits called: event={:p} bits=0x{:08x}",
        event,
        bits,
    );
    todo!("event_group_clear_bits")
}

pub unsafe extern "C" fn event_group_wait_bits(
    event: *mut c_void,
    bits_to_wait_for: u32,
    clear_on_exit: i32,
    wait_for_all_bits: i32,
    block_time_tick: u32,
) -> u32 {
    log::error!(
        "wifi_os_event_group_wait_bits called: event={:p} bits=0x{:08x} clear={} all={} ticks={}",
        event,
        bits_to_wait_for,
        clear_on_exit,
        wait_for_all_bits,
        block_time_tick,
    );
    todo!("event_group_wait_bits")
}

pub unsafe extern "C" fn task_create_pinned_to_core(
    task_func: *mut c_void,
    _task_name: *const core::ffi::c_char,
    stack_depth: u32,
    param: *mut c_void,
    prio: u32,
    task_handle: *mut c_void,
    core_id: u32,
) -> i32 {
    let task_name = "unused";

    let task_func = core::mem::transmute::<*mut c_void, extern "C" fn(*mut c_void)>(task_func);

    let task = esp_radio_rtos_driver::task_create(
        task_name,
        task_func,
        param,
        prio,
        if core_id < 2 { Some(core_id) } else { None },
        stack_depth as usize,
    );
    *(task_handle as *mut usize) = task.as_ptr() as usize;
    if let Some(count) = wifi_os_should_log(&WIFI_OS_TASK_LOG_COUNT) {
        log::info!(
            "[WIFI_OS] task_create_pinned#{} func={:p} param={:p} prio={} stack={} core={} handle={:p} now_us={}",
            count,
            task_func as *mut c_void,
            param,
            prio,
            stack_depth,
            core_id,
            task.as_ptr(),
            Tick::now().as_micros(),
        );
    }

    1
}

pub unsafe extern "C" fn task_create(
    task_func: *mut c_void,
    _task_name: *const core::ffi::c_char,
    stack_depth: u32,
    param: *mut c_void,
    prio: u32,
    task_handle: *mut c_void,
) -> i32 {
    let task_name = "unused";

    let task_func = core::mem::transmute::<*mut c_void, extern "C" fn(*mut c_void)>(task_func);

    let task = esp_radio_rtos_driver::task_create(
        task_name,
        task_func,
        param,
        prio,
        None,
        stack_depth as usize,
    );
    *(task_handle as *mut usize) = task.as_ptr() as usize;
    if let Some(count) = wifi_os_should_log(&WIFI_OS_TASK_LOG_COUNT) {
        log::info!(
            "[WIFI_OS] task_create#{} func={:p} param={:p} prio={} stack={} handle={:p} now_us={}",
            count,
            task_func as *mut c_void,
            param,
            prio,
            stack_depth,
            task.as_ptr(),
            Tick::now().as_micros(),
        );
    }

    1
}

pub unsafe extern "C" fn task_delete(task_handle: *mut c_void) {
    esp_radio_rtos_driver::schedule_task_deletion(NonNull::new(task_handle.cast::<()>()));
}

pub unsafe extern "C" fn task_delay(tick: u32) {
    if let Some(count) = wifi_os_should_log(&WIFI_OS_TASK_LOG_COUNT) {
        log::info!(
            "[WIFI_OS] task_delay_enter#{} tick={} now_us={}",
            count,
            tick,
            Tick::now().as_micros(),
        );
    }
    crate::scheduler::suspend_me_for::<()>(Tick(tick as usize), None);
    if let Some(count) = wifi_os_should_log(&WIFI_OS_TASK_LOG_COUNT) {
        log::info!(
            "[WIFI_OS] task_delay_exit#{} tick={} now_us={}",
            count,
            tick,
            Tick::now().as_micros(),
        );
    }
}

pub unsafe extern "C" fn task_ms_to_tick(ms: u32) -> i32 {
    Tick::from_millis(ms as u64).0 as i32
}

pub unsafe extern "C" fn task_get_current_task() -> *mut c_void {
    esp_radio_rtos_driver::current_task()
        .cast::<c_void>()
        .as_ptr()
}

pub unsafe extern "C" fn task_get_max_priority() -> i32 {
    esp_radio_rtos_driver::max_task_priority() as i32
}

/// Rust-side log output function called from C bridge (log_bridge.c).
/// Receives the fully formatted message string and prints via kernel log.
#[no_mangle]
pub unsafe extern "C" fn blueos_wifi_log_output(level: c_uint, tag: *const c_char, msg: *const c_char) {
    let level_name = match level {
        0 => "NONE",
        1 => "ERROR",
        2 => "WARN",
        3 => "INFO",
        4 => "DEBUG",
        5 => "VERBOSE",
        _ => "???",
    };
    let tag_str = if tag.is_null() {
        "<null>"
    } else {
        // Safety: tag is a C string from the ESP driver, guaranteed non-null here
        unsafe { core::str::from_utf8_unchecked(core::slice::from_raw_parts(tag as *const u8, unsafe { libc::strlen(tag) } as usize)) }
    };
    let msg_str = if msg.is_null() {
        "<null>"
    } else {
        // Safety: msg is a C string produced by vsnprintf in log_bridge.c
        unsafe { core::str::from_utf8_unchecked(core::slice::from_raw_parts(msg as *const u8, unsafe { libc::strlen(msg) } as usize)) }
    };
    match level {
        1 => log::error!("[ESP_WIFI][{}] {}", tag_str, msg_str),
        2 => log::warn!("[ESP_WIFI][{}] {}", tag_str, msg_str),
        3 => log::info!("[ESP_WIFI][{}] {}", tag_str, msg_str),
        4 if wifi_os_diag_log_enabled() => log::debug!("[ESP_WIFI][{}] {}", tag_str, msg_str),
        _ if wifi_os_diag_log_enabled() => log::trace!("[ESP_WIFI][{}] {}", tag_str, msg_str),
        _ => {}
    }
}

extern "C" {
    /// C bridge for _log_writev — formats va_list via vsnprintf and calls
    /// blueos_wifi_log_output. Defined in log_bridge.c.
    fn wifi_log_writev_bridge(level: c_uint, tag: *const c_char, format: *const c_char, args: *mut c_void);
    /// C bridge for _log_write — formats varargs and calls wifi_log_writev_bridge.
    /// Defined in log_bridge.c.
    fn wifi_log_write_bridge(level: c_uint, tag: *const c_char, format: *const c_char, ...);
}

pub unsafe extern "C" fn log_writev(level: c_uint, tag: *const c_char, format: *const c_char, args: *mut c_void) {
    wifi_log_writev_bridge(level, tag, format, args)
}

pub unsafe extern "C" fn log_write(level: c_uint, tag: *const c_char, format: *const c_char, ...) {
    // We can't forward Rust ... to C ... directly.
    // Instead, just call the C bridge which handles varargs natively.
    // Note: wifi_log in libnet80211.a always calls _log_writev first,
    // then _log_write with the same arguments. Since _log_writev already
    // printed the message, we make _log_write a no-op to avoid duplicate output.
    // The C bridge function exists for completeness but we skip duplicate printing.
}

pub unsafe extern "C" fn malloc(size: usize) -> *mut c_void {
    crate::allocator::malloc(size) as *mut c_void
}

pub unsafe extern "C" fn free(p: *mut c_void) {
    crate::allocator::free(p as *mut u8);
}

pub unsafe extern "C" fn event_post(
    event_base: *const core::ffi::c_char,
    event_id: i32,
    event_data: *mut c_void,
    event_data_size: usize,
    ticks_to_wait: u32,
) -> i32 {
    use num_traits::FromPrimitive;

    let Some(event) = WifiEvent::from_i32(event_id) else {
        log::warn!("Unknown event id: {}", event_id);
        return 0;
    };
    let important_event = matches!(
        &event,
        WifiEvent::ScanDone
            | WifiEvent::StationConnected
            | WifiEvent::StationDisconnected
            | WifiEvent::StationAuthenticationModeChange
            | WifiEvent::StationBeaconTimeout
    );
    if wifi_os_diag_log_enabled() {
        log::debug!("Event: {:?}", event);
    }

    let Some(payload) = super::event::EventInfo::from_wifi_event_raw(event, event_data) else {
        return 0;
    };
    if important_event {
        log::info!(
            "WiFi event_post: base={:p} id={} size={} ticks={} payload={:?}",
            event_base,
            event_id,
            event_data_size,
            ticks_to_wait,
            payload
        );
    } else if wifi_os_diag_log_enabled() {
        log::debug!("Event payload: {:?}", payload);
    }

    // Forward to async handler only; payload processing stays in async context.
    if let Err(e) = unsafe { super::EVENT_SENDER.assume_init_mut() }.try_send(payload) {
        log::warn!("Event channel full, dropping event: {:?}", e.0);
    } else if important_event {
        log::info!("WiFi event_post: queued id={}", event_id);
    }
    0
}

#[no_mangle]
pub unsafe extern "C" fn get_free_internal_heap_size() -> usize {
    let memory_info = crate::allocator::memory_info();
    (memory_info.total - memory_info.used) as usize
}

pub unsafe extern "C" fn get_free_heap_size() -> u32 {
    let memory_info = crate::allocator::memory_info();
    (memory_info.total - memory_info.used) as u32
}

pub unsafe extern "C" fn rand() -> u32 {
    random_u32()
}

pub unsafe extern "C" fn dport_access_stall_other_cpu_start_wrap() {}

pub unsafe extern "C" fn dport_access_stall_other_cpu_end_wrap() {}

/// no-op
pub unsafe extern "C" fn wifi_apb80m_request() {}

/// no-op
pub unsafe extern "C" fn wifi_apb80m_release() {}

const SYSTEM_WIFI_CLK_WIFI_BT_COMMON_M: u32 = 0x78078F;
static PHY_CLK_REF: AtomicU32 = AtomicU32::new(0);
static PHY_CLK_LOCK: SpinLock<()> = SpinLock::new(());
const WIFI_CLK_EN_REG_ADDRESS: usize = 0x60026014;

pub unsafe extern "C" fn phy_disable() {
    esp_phy::disable_phy();
}

pub unsafe extern "C" fn phy_enable() {
    core::mem::forget(esp_phy::enable_phy());
}

// no-support
pub unsafe extern "C" fn phy_update_country_info(_country: *const core::ffi::c_char) -> i32 {
    -1
}

pub unsafe extern "C" fn read_mac(mac_out: *mut u8, type_: u32) -> i32 {
    let mac = read_mac_address();
    match type_ {
        0 => {
            // Station
            let mac_bytes = mac.bytes();
            unsafe {
                core::ptr::copy_nonoverlapping(mac_bytes.as_ptr(), mac_out, 6);
            }
            0
        }
        1 => {
            // Access Point
            let mac_bytes = mac.get_local_mac();
            unsafe {
                core::ptr::copy_nonoverlapping(mac_bytes.as_ptr(), mac_out, 6);
            }
            0
        }
        _ => -1,
    }
}

pub unsafe extern "C" fn ets_timer_arm(timer: *mut c_void, tmout: u32, repeat: bool) {
    let count = wifi_os_next_log_count(&WIFI_OS_TIMER_ARM_LOG_COUNT);
    let should_log = wifi_os_diag_log_enabled() && (wifi_os_log_count_enabled(count) || tmout >= 1000);
    if should_log {
        let priv_ = (timer as *mut ets_timer)
            .as_ref()
            .map(|timer| timer.priv_)
            .unwrap_or(core::ptr::null_mut());
        log::info!(
            "[WIFI_OS] ets_timer_arm#{} timer={:p} wake_null={} priv={:p} ms={} repeat={} now_us={}",
            count,
            timer,
            timer as u32 == wake_null_timer_addr(),
            priv_,
            tmout,
            repeat,
            Tick::now().as_micros(),
        );
    }

    ets_timer_arm_us(timer, tmout.saturating_mul(1000), repeat);

    if should_log {
        log::info!(
            "[WIFI_OS] ets_timer_arm_done#{} timer={:p} ms={} repeat={} now_us={}",
            count,
            timer,
            tmout,
            repeat,
            Tick::now().as_micros(),
        );
    }
}

pub unsafe extern "C" fn ets_timer_disarm(timer: *mut c_void) {
    let ets_timer = timer as *mut ets_timer;
    let ets_timer = ets_timer.as_mut().expect("ets_timer is null");

    if let Some(count) = wifi_os_should_log(&WIFI_OS_TIMER_LOG_COUNT) {
        log::info!(
            "[WIFI_OS] ets_timer_disarm#{} timer={:p} wake_null={} priv={:p} now_us={}",
            count,
            timer,
            timer as u32 == wake_null_timer_addr(),
            ets_timer.priv_,
            Tick::now().as_micros(),
        );
    }

    if let Some(timer) = TimerPtr::new(ets_timer.priv_.cast()) {
        let timer = unsafe { TimerHandle::ref_from_ptr(&timer) };

        timer.disarm();
    }
}

pub unsafe extern "C" fn ets_timer_done(ptimer: *mut c_void) {
    let ets_timer = ptimer as *mut ets_timer;
    let ets_timer = ets_timer.as_mut().expect("ets_timer is null");

    if let Some(count) = wifi_os_should_log(&WIFI_OS_TIMER_LOG_COUNT) {
        log::info!(
            "[WIFI_OS] ets_timer_done#{} timer={:p} wake_null={} priv={:p} now_us={}",
            count,
            ptimer,
            ptimer as u32 == wake_null_timer_addr(),
            ets_timer.priv_,
            Tick::now().as_micros(),
        );
    }

    if let Some(timer) = TimerPtr::new(ets_timer.priv_.cast()) {
        let timer = unsafe { TimerHandle::from_ptr(timer) };

        core::mem::drop(timer);
        ets_timer.priv_ = core::ptr::null_mut();
    }
}

pub unsafe extern "C" fn ets_timer_setfn(
    ptimer: *mut c_void,
    pfunction: *mut c_void,
    parg: *mut c_void,
) {
    // This function is expected to create timers. For the simplicity of the preempt API, we
    // will not update existing timers, but create new ones.
    ets_timer_done(ptimer);

    let ets_timer = ptimer as *mut ets_timer;
    let ets_timer = ets_timer.as_mut().expect("ets_timer is null");
    let timer = unsafe {
        TimerHandle::new(
            core::mem::transmute::<*mut c_void, unsafe extern "C" fn(*mut c_void)>(pfunction),
            parg,
        )
    }
    .leak()
    .cast()
    .as_ptr();

    let count = wifi_os_next_log_count(&WIFI_OS_TIMER_LOG_COUNT);
    if wifi_os_diag_log_enabled() {
        log::info!(
            "[WIFI_OS] ets_timer_setfn#{} timer={:p} wake_null={} func={:p} arg={:p} priv={:p} now_us={}",
            count,
            ptimer,
            ptimer as u32 == wake_null_timer_addr(),
            pfunction,
            parg,
            timer,
            Tick::now().as_micros(),
        );
    }

    ets_timer.next = core::ptr::null_mut();
    ets_timer.period = 0;
    ets_timer.func = None;
    ets_timer.priv_ = timer;
}

pub unsafe extern "C" fn ets_timer_arm_us(ptimer: *mut c_void, us: u32, repeat: bool) {
    let ets_timer = ptimer as *mut ets_timer;
    let ets_timer = ets_timer.as_mut().expect("ets_timer is null");

    let timer = TimerPtr::new(ets_timer.priv_.cast()).expect("timer is null");
    let timer = TimerHandle::ref_from_ptr(&timer);

    let count = wifi_os_next_log_count(&WIFI_OS_TIMER_LOG_COUNT);
    let should_log = wifi_os_diag_log_enabled() && (wifi_os_log_count_enabled(count) || us >= 1_000_000);
    if should_log {
        log::info!(
            "[WIFI_OS] ets_timer_arm_us#{} timer={:p} wake_null={} priv={:p} us={} repeat={} now_us={}",
            count,
            ptimer,
            ptimer as u32 == wake_null_timer_addr(),
            ets_timer.priv_,
            us,
            repeat,
            Tick::now().as_micros(),
        );
    }

    timer.arm(us as u64, repeat);

    if should_log {
        log::info!(
            "[WIFI_OS] ets_timer_arm_us_done#{} timer={:p} priv={:p} us={} repeat={} now_us={}",
            count,
            ptimer,
            ets_timer.priv_,
            us,
            repeat,
            Tick::now().as_micros(),
        );
    }
}

pub unsafe extern "C" fn wifi_reset_mac() {
    const APB_CTRL_BASE: usize = 0x6002_6000;
    const WIFI_RST_EN: *mut u32 = (APB_CTRL_BASE + 0x18) as *mut u32;
    const MAC_RST: u32 = 1 << 2;

    // set_bit()
    let value = core::ptr::read_volatile(WIFI_RST_EN);
    core::ptr::write_volatile(WIFI_RST_EN, value | MAC_RST);

    // clear_bit()
    let value = core::ptr::read_volatile(WIFI_RST_EN);
    core::ptr::write_volatile(WIFI_RST_EN, value & !MAC_RST);
}

/// no-op
pub unsafe extern "C" fn wifi_clock_enable() {}

/// no-op
pub unsafe extern "C" fn wifi_clock_disable() {}

pub unsafe extern "C" fn wifi_rtc_enable_iso() {
    todo!("wifi_rtc_enable_iso")
}

pub unsafe extern "C" fn wifi_rtc_disable_iso() {
    todo!("wifi_rtc_disable_iso")
}

pub unsafe extern "C" fn __esp_radio_esp_timer_get_time() -> i64 {
    Tick::now().as_micros() as i64
}

pub unsafe extern "C" fn nvs_set_i8(
    _handle: u32,
    _key: *const core::ffi::c_char,
    _value: i8,
) -> i32 {
    0
}

pub unsafe extern "C" fn nvs_get_i8(
    _handle: u32,
    _key: *const core::ffi::c_char,
    _out_value: *mut i8,
) -> i32 {
    todo!("nvs_get_i8")
}

pub unsafe extern "C" fn nvs_set_u8(
    _handle: u32,
    _key: *const core::ffi::c_char,
    _value: u8,
) -> i32 {
    todo!("nvs_set_u8")
}

pub unsafe extern "C" fn nvs_get_u8(
    _handle: u32,
    _key: *const core::ffi::c_char,
    _out_value: *mut u8,
) -> i32 {
    todo!("nvs_get_u8")
}

pub unsafe extern "C" fn nvs_set_u16(
    _handle: u32,
    _key: *const core::ffi::c_char,
    _value: u16,
) -> i32 {
    todo!("nvs_set_u16")
}

pub unsafe extern "C" fn nvs_get_u16(
    _handle: u32,
    _key: *const core::ffi::c_char,
    _out_value: *mut u16,
) -> i32 {
    todo!("nvs_get_u16")
}

pub unsafe extern "C" fn nvs_open(
    _name: *const core::ffi::c_char,
    _open_mode: u32,
    _out_handle: *mut u32,
) -> i32 {
    todo!("nvs_open")
}

pub unsafe extern "C" fn nvs_close(_handle: u32) {
    todo!("nvs_close")
}

pub unsafe extern "C" fn nvs_commit(_handle: u32) -> i32 {
    todo!("nvs_commit")
}

pub unsafe extern "C" fn nvs_set_blob(
    _handle: u32,
    _key: *const core::ffi::c_char,
    _value: *const c_void,
    _length: usize,
) -> i32 {
    todo!("nvs_set_blob")
}

pub unsafe extern "C" fn nvs_get_blob(
    _handle: u32,
    _key: *const core::ffi::c_char,
    _out_value: *mut c_void,
    _length: *mut usize,
) -> i32 {
    todo!("nvs_get_blob")
}

pub unsafe extern "C" fn nvs_erase_key(_handle: u32, _key: *const core::ffi::c_char) -> i32 {
    todo!("nvs_erase_key")
}

pub unsafe extern "C" fn get_random(buf: *mut u8, len: usize) -> i32 {
    let slice = core::slice::from_raw_parts_mut(buf, len);
    crate::boards::random(slice);
    0
}

pub unsafe extern "C" fn get_time(_t: *mut c_void) -> i32 {
    todo!("get_time")
}

pub unsafe extern "C" fn random() -> u32 {
    random_u32()
}

pub unsafe extern "C" fn slowclk_cal_get() -> u32 {
    28639
}

pub unsafe extern "C" fn log_timestamp() -> u32 {
    crate::time::Tick::now().as_millis() as u32
}

#[no_mangle]
pub unsafe extern "C" fn malloc_internal(size: usize) -> *mut c_void {
    crate::allocator::malloc(size) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn free_internal(ptr: *mut c_void) {
    crate::allocator::free(ptr as *mut u8);
}

#[no_mangle]
pub unsafe extern "C" fn realloc_internal(ptr: *mut c_void, size: usize) -> *mut c_void {
    crate::allocator::realloc(ptr as *mut u8, size) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn calloc_internal(n: usize, size: usize) -> *mut c_void {
    crate::allocator::calloc(n, size) as *mut c_void
}

pub unsafe extern "C" fn zalloc_internal(size: usize) -> *mut c_void {
    calloc_internal(size, 1)
}

pub unsafe extern "C" fn wifi_malloc(size: usize) -> *mut c_void {
    malloc_internal(size)
}

pub unsafe extern "C" fn wifi_realloc(ptr: *mut c_void, size: usize) -> *mut c_void {
    realloc_internal(ptr, size)
}

pub unsafe extern "C" fn wifi_calloc(n: usize, size: usize) -> *mut c_void {
    calloc_internal(n, size)
}

pub unsafe extern "C" fn wifi_zalloc(size: usize) -> *mut c_void {
    wifi_calloc(size, 1)
}

pub unsafe extern "C" fn wifi_create_queue(queue_len: i32, item_size: i32) -> *mut c_void {
    let queue = queue_create(queue_len as u32, item_size as u32);

    let queue_ptr: *mut *mut c_void = Box::leak(Box::new(queue));

    queue_ptr.cast()
}

pub unsafe extern "C" fn wifi_delete_queue(queue: *mut c_void) {
    let queue_ptr: *mut *mut c_void = queue.cast();

    let boxed = unsafe { Box::from_raw(queue_ptr) };

    queue_delete(*boxed)
}

pub unsafe extern "C" fn coex_init() -> i32 {
    0
}

pub unsafe extern "C" fn coex_deinit() {}

pub unsafe extern "C" fn coex_enable() -> i32 {
    0
}

pub unsafe extern "C" fn coex_disable() {}

pub unsafe extern "C" fn coex_status_get() -> u32 {
    0
}

pub unsafe extern "C" fn coex_wifi_request(_event: u32, _latency: u32, _duration: u32) -> i32 {
    0
}

pub unsafe extern "C" fn coex_wifi_release(_event: u32) -> i32 {
    0
}

pub unsafe extern "C" fn coex_wifi_channel_set(_primary: u8, _secondary: u8) -> i32 {
    if wifi_os_diag_log_enabled() {
        log::info!("[COEX] coex_wifi_channel_set: primary={}, secondary={}", _primary, _secondary);
    }
    0
}

pub unsafe extern "C" fn coex_event_duration_get(_event: u32, _duration: *mut u32) -> i32 {
    0
}

pub unsafe extern "C" fn coex_pti_get(_event: u32, _pti: *mut u8) -> i32 {
    0
}

pub unsafe extern "C" fn coex_schm_status_bit_clear(_type_: u32, _status: u32) {}

pub unsafe extern "C" fn coex_schm_status_bit_set(_type_: u32, _status: u32) {}

pub unsafe extern "C" fn coex_schm_interval_set(_interval: u32) -> i32 {
    0
}

pub unsafe extern "C" fn coex_schm_interval_get() -> u32 {
    0
}

pub unsafe extern "C" fn coex_schm_curr_period_get() -> u8 {
    0
}

pub unsafe extern "C" fn coex_schm_curr_phase_get() -> *mut c_void {
    core::ptr::null_mut()
}

pub unsafe extern "C" fn coex_schm_process_restart() -> i32 {
    0
}

pub unsafe extern "C" fn coex_schm_register_cb(
    _arg1: i32,
    _cb: Option<unsafe extern "C" fn(_arg1: i32) -> i32>,
) -> i32 {
    0
}

pub unsafe extern "C" fn coex_register_start_cb(_cb: Option<unsafe extern "C" fn() -> i32>) -> i32 {
    0
}

pub unsafe extern "C" fn coex_schm_flexible_period_set(_arg1: u8) -> i32 {
    0
}

pub unsafe extern "C" fn coex_schm_flexible_period_get() -> u8 {
    0
}

pub unsafe extern "C" fn coex_schm_get_phase_by_idx(_arg1: i32) -> *mut c_void {
    core::ptr::null_mut()
}

pub unsafe extern "C" fn calloc_internal_wrapper(n: usize, size: usize) -> *mut c_void {
    calloc_internal(n, size)
}

pub unsafe extern "C" fn coex_schm_register_cb_wrapper(
    arg1: i32,
    cb: Option<unsafe extern "C" fn(arg1: i32) -> i32>,
) -> i32 {
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn phy_printf(format: *const c_char, mut __valist: ...) -> core::ffi::c_int {
    if format.is_null() {
        return 0;
    }
    let s = unsafe { core::ffi::CStr::from_ptr(format) };
    if let Ok(s) = s.to_str() {
        crate::kearly_println!("[phy] {}", s);
    }
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn pp_printf(format: *const c_char, mut __valist: ...) -> core::ffi::c_int {
    if format.is_null() {
        return 0;
    }
    let s = unsafe { core::ffi::CStr::from_ptr(format) };
    if let Ok(s) = s.to_str() {
        crate::kearly_println!("[pp] {}", s);
    }
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn net80211_printf(
    format: *const c_char,
    mut __valist: ...
) -> core::ffi::c_int {
    if format.is_null() {
        return 0;
    }
    let s = unsafe { core::ffi::CStr::from_ptr(format) };
    if let Ok(s) = s.to_str() {
        crate::kearly_println!("[net80211] {}", s);
    }
    0
}
