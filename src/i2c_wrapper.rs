use core::cell::RefCell;
use core::ops::Deref;
use embedded_hal::i2c::{Error as I2cError, I2c};

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

impl<'a, I, E> I2c for I2cWrapper<'a, I>
where
    I: I2c<Error = E>,
{
    type Error = E;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.inner.borrow_mut().read(address, buffer)
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.inner.borrow_mut().write(address, bytes)
    }

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.inner.borrow_mut().write_read(address, bytes, buffer)
    }
}
