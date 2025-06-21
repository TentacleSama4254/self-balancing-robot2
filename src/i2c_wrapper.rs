use core::cell::RefCell;
// Use the newer embedded-hal 1.0.0 I2c trait
use embedded_hal::i2c::{I2c, ErrorType, Operation};

/// A wrapper for I2C that implements Clone by using a shared reference
pub struct I2cWrapper<'a, I> {
    inner: &'a RefCell<I>,
}

impl<'a, I> I2cWrapper<'a, I> {
    pub fn new(i2c: &'a RefCell<I>) -> Self {
        Self { inner: i2c }
    }
}

impl<'a, I> Clone for I2cWrapper<'a, I> {
    fn clone(&self) -> Self {
        Self { inner: self.inner }
    }
}

// Implement the ErrorType trait for the wrapper
impl<'a, I> ErrorType for I2cWrapper<'a, I>
where
    I: ErrorType,
{
    type Error = I::Error;
}

// Implement the I2c trait from embedded-hal 1.0.0
impl<'a, I> I2c for I2cWrapper<'a, I>
where
    I: I2c,
{
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.inner.borrow_mut().transaction(address, operations)
    }
}
