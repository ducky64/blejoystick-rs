use bilge::prelude::*;
use embedded_hal_async::i2c::{I2c, Operation};

// TODO separate out into own repo

/// Transport layer that supports both SPI and I2C
trait Transport {
    type Error;
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error>;
    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;
}

pub struct I2cTransport<I2CType> {
    i2c: I2CType,
    address: u8,
}

impl<I2CType> Transport for I2cTransport<I2CType>
where
    I2CType: I2c,
{
    type Error = I2CType::Error;

    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2CType::Error> {
        self.i2c.write(self.address, &[addr]).await?;
        self.i2c.write(self.address, data).await?;
        Ok(())
    }

    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), I2CType::Error> {
        self.i2c.write(self.address, &[addr]).await?;
        self.i2c.read(self.address, buffer).await?;
        Ok(())
    }
}

pub struct Lsm6ds3tr<TransportType> {
    transport: TransportType,
}

impl<I2CType> Lsm6ds3tr<I2cTransport<I2CType>> {
    pub fn new(i2c: I2CType, address: u8) -> Self {
        Self {
            transport: I2cTransport { i2c, address },
        }
    }
}

impl<TransportType> Lsm6ds3tr<TransportType> {}
