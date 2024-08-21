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
}
unsafe impl Send for UartState {}
unsafe impl Sync for UartState {}

