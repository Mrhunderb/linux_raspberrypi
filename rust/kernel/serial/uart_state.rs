use core::mem::MaybeUninit;

#[repr(transparent)]
pub struct UartState(bindings::uart_state);

impl UartState {
    /// Create a new [`UartState`]
    pub const fn new() -> Self {
        let uart_state = unsafe { MaybeUninit::<bindings::uart_state>::zeroed().assume_init() };
        Self(uart_state)
    }

    /// Returns a raw pointer to the inner C struct.
    #[inline]
    pub const fn as_ptr(&self) -> *mut bindings::uart_state {
        &self.0 as *const _ as *mut _
    }

    /// Creates a reference to a [`UartState`] from a valid pointer.
    /// # Safety
    /// Callers must ensure that `ptr` is valid, non-null, and has a non-zero reference count for
    /// the entire duration when the returned reference exists.
    pub unsafe fn from_raw<'a>(ptr: *mut bindings::uart_state) -> &'a Self {
        // SAFETY: Guaranteed by the safety requirements of the function.
        unsafe { &*ptr.cast() }
    }

    pub fn get_state(&self) -> u32 {
        self.0.pm_state
    }

}
unsafe impl Send for UartState {}
unsafe impl Sync for UartState {}

