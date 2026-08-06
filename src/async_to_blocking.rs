use embassy_futures::block_on;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal::i2c::{ErrorType, I2c, Operation};

pub struct AsyncToBlockingI2c<'a, T: 'static> {
    mutex: &'a Mutex<CriticalSectionRawMutex, T>,
}

impl<'a, T: 'static> AsyncToBlockingI2c<'a, T> {
    pub fn new(mutex: &'a Mutex<CriticalSectionRawMutex, T>) -> Self {
        Self { mutex }
    }
}

impl<'a, T> ErrorType for AsyncToBlockingI2c<'a, T>
where
    T: embedded_hal_async::i2c::ErrorType,
{
    type Error = T::Error;
}

impl<'a, T> I2c for AsyncToBlockingI2c<'a, T>
where
    T: embedded_hal_async::i2c::I2c,
{
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        // Block on acquiring the async mutex and executing the async transaction
        block_on(async {
            let mut bus = self.mutex.lock().await;
            bus.transaction(address, operations).await
        })
    }
}