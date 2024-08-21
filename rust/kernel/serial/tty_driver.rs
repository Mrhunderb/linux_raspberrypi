use core::mem::MaybeUninit;

#[repr(transparent)]
pub struct TTYDriver(bindings::tty_driver);

impl TTYDriver {
    /// Create a new [`TTYDriver`]
    pub const fn new() -> Self {
        let tty_driver = unsafe { MaybeUninit::<bindings::tty_driver>::zeroed().assume_init() };
        Self(tty_driver)
    }

    /// Returns a raw pointer to the inner C struct.
    #[inline]
    pub const fn as_ptr(&self) -> *mut bindings::tty_driver {
        &self.0 as *const _ as *mut _
    }
}
unsafe impl Send for TTYDriver {}
unsafe impl Sync for TTYDriver {}
