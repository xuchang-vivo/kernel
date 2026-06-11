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
    boards::{efuse::read_mac_address, get_device, random_u32, Handler},
    scheduler::{self, wait_queue, InsertToEnd, WaitEntry},
    sync::{mqueue::MessageQueue, SpinLock},
    thread::{Entry, Stack, SUSPENDED},
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
    sync::atomic::{AtomicU32, Ordering},
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
use esp_sync::{
    raw::{RawLock, SingleCoreInterruptLock},
    RawMutex,
};
use esp_wifi_sys_esp32c3::include::{
    esp_event_base_t, ets_timer, timeval, OSI_FUNCS_TIME_BLOCKING,
};

static WIFI_LOCK: RawMutex = RawMutex::new();

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

struct BkScheduler;

impl SchedulerImplementation for BkScheduler {
    fn initialized(&self) -> bool {
        crate::scheduler::is_schedule_ready()
    }

    fn yield_task(&self) {
        crate::scheduler::yield_me();
    }

    fn yield_task_from_isr(&self) {
        crate::scheduler::yield_me_now_or_later();
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
        let blueos_prio = crate::config::MAX_THREAD_PRIORITY
            .saturating_sub(priority.min(crate::config::MAX_THREAD_PRIORITY));
        let thread = crate::thread::Builder::new(entry)
            .set_stack(stack)
            .set_priority(blueos_prio as ThreadPriority)
            .start();
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
struct EspWaitQueue(wait_queue::WaitQueue);

impl WaitQueueImplementation for EspWaitQueue {
    fn create() -> WaitQueuePtr {
        let mut wq = Box::new(EspWaitQueue(wait_queue::WaitQueue::new()));
        wq.0.init();
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
        with_iou!(|borrowed_wait_entry| {
            let mut wait_entry = WaitEntry::new(this_thread.clone());
            borrowed_wait_entry =
                wait_queue::insert(&mut this.0, &mut wait_entry, InsertToEnd::MODE).unwrap();
            let _ = scheduler::suspend_me_until::<()>(deadline, None);
            borrowed_wait_entry = this.0.pop(borrowed_wait_entry).unwrap();
        });
    }

    unsafe fn notify(queue: WaitQueuePtr) {
        let this = &*(queue.as_ptr() as *const EspWaitQueue);
        // Wake the first waiter (entries are appended to end, so first = oldest waiter).
        if let Some(entry) = this.0.iter().next() {
            let t = entry.thread.clone();
            let _ = scheduler::queue_ready_thread(SUSPENDED, t);
        }
        // scheduler::yield_me();
    }

    unsafe fn notify_from_isr(queue: WaitQueuePtr, higher_prio_task_waken: Option<&mut bool>) {
        let this = &*(queue.as_ptr() as *const EspWaitQueue);
        if let Some(entry) = this.0.iter().next() {
            let t = entry.thread.clone();
            if scheduler::queue_ready_thread(SUSPENDED, t).is_ok() {
                if let Some(hptw) = higher_prio_task_waken {
                    *hptw = true;
                }
            }
        }
        // scheduler::yield_me_now_or_later();
    }
}

/// BkSemaphore: a semaphore implementation using SingleCoreInterruptLock instead of
/// NonReentrantMutex. This avoids the "lock is not reentrant" panic that occurs when
/// CompatSemaphore's NonReentrantMutex is held across a context switch inside
/// EspWaitQueue::wait_until.
///
/// SingleCoreInterruptLock only disables/re-enables interrupts (csrrci mstatus, 8 on RISC-V)
/// with no reentry tracking. On single-core, interrupt disable is sufficient for mutual
/// exclusion, and a context switch inside the critical section won't trigger a panic
/// when another thread tries the same lock.
struct BkSemaphore {
    lock: SingleCoreInterruptLock,
    data: UnsafeCell<SemaphoreBkData>,
}

struct SemaphoreBkData {
    kind: SemaphoreKind,
    current: u32,
    max: u32,
    waiting: WaitQueuePtr,
}

unsafe impl Sync for BkSemaphore {}

impl BkSemaphore {
    fn new(kind: SemaphoreKind) -> Self {
        let (current, max) = match kind {
            SemaphoreKind::Counting { max, initial } => (initial, max),
            SemaphoreKind::Mutex | SemaphoreKind::RecursiveMutex => (1, 1),
        };
        BkSemaphore {
            lock: SingleCoreInterruptLock,
            data: UnsafeCell::new(SemaphoreBkData {
                kind,
                current,
                max,
                waiting: unsafe { EspWaitQueue::create() },
            }),
        }
    }

    /// Enter critical section, run `f`, then exit.  Mirrors how CompatSemaphore
    /// uses `NonReentrantMutex::with()` — except SingleCoreInterruptLock
    /// tracks no locked-state, so context switches inside the closure are safe.
    fn with_irq_safe<R>(&self, f: impl FnOnce(&mut SemaphoreBkData, &Self) -> R) -> R {
        // SAFETY: enter/exit are paired; token is valid only within this call.
        let token = unsafe { self.lock.enter() };
        let r = f(unsafe { &mut *self.data.get() }, self);
        // SAFETY: token from the paired enter above.
        unsafe { self.lock.exit(token) };
        r
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
        EspWaitQueue::delete(sem.with_irq_safe(|d, _| d.waiting));
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
            // Quick path: try to decrement current under interrupt-disable.
            // If semaphore is available, take it and return immediately.
            let available = sem.with_irq_safe(|data, _| {
                if data.current > 0 {
                    data.current -= 1;
                    true
                } else {
                    false
                }
            });
            if available {
                return true;
            }

            // Semaphore not available — need to block.
            // wait_until handles insert + suspend + pop atomically (via with_iou!),
            // and the context switch inside suspend_me_until must happen with
            // interrupts enabled (scheduler asserts local_irq_enabled after resume).
            // So we do NOT hold SingleCoreInterruptLock across the suspend call.
            if _deadline_instant.is_some_and(|d| Tick::now().as_micros() >= d) {
                return false;
            }
            unsafe {
                EspWaitQueue::wait_until(
                    sem.with_irq_safe(|data, _| data.waiting),
                    _deadline_instant,
                );
            }
        }
    }

    unsafe fn give(semaphore: SemaphorePtr) -> bool {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        sem.with_irq_safe(|data, _| {
            if data.current < data.max {
                data.current += 1;
                unsafe { EspWaitQueue::notify(data.waiting) };
                true
            } else {
                false
            }
        })
    }

    unsafe fn try_give_from_isr(
        semaphore: SemaphorePtr,
        higher_prio_task_waken: Option<&mut bool>,
    ) -> bool {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        sem.with_irq_safe(|data, _| {
            if data.current < data.max {
                data.current += 1;
                unsafe {
                    EspWaitQueue::notify_from_isr(data.waiting, higher_prio_task_waken);
                }
                true
            } else {
                false
            }
        })
    }

    unsafe fn current_count(semaphore: SemaphorePtr) -> u32 {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        sem.with_irq_safe(|data, _| data.current)
    }

    unsafe fn try_take(semaphore: SemaphorePtr) -> bool {
        let sem = &*semaphore.cast::<BkSemaphore>().as_ptr();
        sem.with_irq_safe(|data, _| {
            if data.current > 0 {
                data.current -= 1;
                true
            } else {
                false
            }
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

pub unsafe extern "C" fn wifi_int_disable(_wifi_int_mux: *mut c_void) -> u32 {
    // TODO: can we use wifi_int_mux?
    let token = unsafe { WIFI_LOCK.acquire() };
    unsafe { core::mem::transmute::<esp_sync::RestoreState, u32>(token) }
}

pub unsafe extern "C" fn wifi_int_restore(_wifi_int_mux: *mut c_void, tmp: u32) {
    let token = unsafe { core::mem::transmute::<u32, esp_sync::RestoreState>(tmp) };
    unsafe { WIFI_LOCK.release(token) }
}

pub unsafe extern "C" fn task_yield_from_isr() {
    crate::scheduler::yield_me_now_or_later();
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
        let micros = Tick(block_time_tick as usize).as_micros() as u32;
        let ptr = SemaphorePtr::new(semphr.cast()).expect("invalid semaphore pointer");
        let handle = SemaphoreHandle::ref_from_ptr(&ptr);
        let timeout = if micros == OSI_FUNCS_TIME_BLOCKING {
            None
        } else {
            Some(micros)
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
    QueueHandle::new(queue_len as usize, item_size as usize)
        .leak()
        .as_ptr()
        .cast()
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

pub unsafe extern "C" fn queue_send_from_isr(
    queue: *mut c_void,
    item: *mut c_void,
    hptw: *mut c_void,
) -> i32 {
    if !queue.is_null() {
        let ptr = QueuePtr::new(queue.cast()).expect("invalid queue pointer");
        let handle = unsafe { QueueHandle::ref_from_ptr(&ptr) };
        handle.try_send_to_back_from_isr(item.cast(), (hptw as *mut bool).as_mut()) as i32
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
        let micros = Tick(block_time_tick as usize).as_micros() as u32;
        let timeout = if micros == OSI_FUNCS_TIME_BLOCKING {
            None
        } else {
            Some(micros)
        };

        handle.send_to_back(item.cast(), timeout) as i32
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
        let micros = Tick(block_time_tick as usize).as_micros() as u32;
        let timeout = if micros == OSI_FUNCS_TIME_BLOCKING {
            None
        } else {
            Some(micros)
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
        let micros = Tick(block_time_tick as usize).as_micros() as u32;
        let timeout = if micros == OSI_FUNCS_TIME_BLOCKING {
            None
        } else {
            Some(micros)
        };

        handle.receive(item.cast(), timeout) as i32
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
    todo!("event_group_create")
}

pub unsafe extern "C" fn event_group_delete(_event: *mut c_void) {
    todo!("event_group_delete")
}

pub unsafe extern "C" fn event_group_set_bits(_event: *mut c_void, _bits: u32) -> u32 {
    todo!("event_group_set_bits")
}

pub unsafe extern "C" fn event_group_clear_bits(_event: *mut c_void, _bits: u32) -> u32 {
    todo!("event_group_clear_bits")
}

pub unsafe extern "C" fn event_group_wait_bits(
    _event: *mut c_void,
    _bits_to_wait_for: u32,
    _clear_on_exit: i32,
    _wait_for_all_bits: i32,
    _block_time_tick: u32,
) -> u32 {
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

    1
}

pub unsafe extern "C" fn task_delete(task_handle: *mut c_void) {
    esp_radio_rtos_driver::schedule_task_deletion(NonNull::new(task_handle.cast::<()>()));
}

pub unsafe extern "C" fn task_delay(tick: u32) {
    crate::scheduler::suspend_me_for::<()>(Tick(tick as usize), None);
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

    if let Some(event) = crate::boards::wifi::event::WifiEvent::from_i32(event_id) {
        log::debug!("Event: {:?}", event);

        if let Some(payload) = super::event::EventInfo::from_wifi_event_raw(event, event_data) {
            log::debug!("Event payload: {:?}", payload);
        }
    } else {
        log::warn!("Unknown event id: {}", event_id);
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
    ets_timer_arm_us(timer, tmout.saturating_mul(1000), repeat);
}

pub unsafe extern "C" fn ets_timer_disarm(timer: *mut c_void) {
    let ets_timer = timer as *mut ets_timer;
    let ets_timer = ets_timer.as_mut().expect("ets_timer is null");

    if let Some(timer) = TimerPtr::new(ets_timer.priv_.cast()) {
        let timer = unsafe { TimerHandle::ref_from_ptr(&timer) };

        timer.disarm();
    }
}

pub unsafe extern "C" fn ets_timer_done(ptimer: *mut c_void) {
    let ets_timer = ptimer as *mut ets_timer;
    let ets_timer = ets_timer.as_mut().expect("ets_timer is null");
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

    timer.arm(us as u64, repeat);
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
