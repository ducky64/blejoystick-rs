use bilge::prelude::*;
use embedded_hal_async::i2c::I2c;

// TODO separate out into own repo

/// Transport layer that supports both SPI and I2C
pub trait Transport {
    type Error;
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error>;
    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;
}

pub struct I2cTransport<I2cType> {
    i2c: I2cType,
    address: u8,
}

impl<I2cType> Transport for I2cTransport<I2cType>
where
    I2cType: I2c,
{
    type Error = I2cType::Error;

    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), I2cType::Error> {
        self.i2c.write(self.address, &[addr]).await?;
        self.i2c.write(self.address, data).await?;
        Ok(())
    }

    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), I2cType::Error> {
        self.i2c.write_read(self.address, &[addr], buffer).await?;
        Ok(())
    }
}

pub struct Lsm6ds3tr<TransportType>
where
    TransportType: Transport,
{
    transport: TransportType,
}

impl<I2cType> Lsm6ds3tr<I2cTransport<I2cType>>
where
    I2cType: I2c,
{
    const LSM6DS3TR_ID: u8 = 0x6A; // SA0 = 0

    /// Creates a device with address with SA0=0
    pub fn new(i2c: I2cType) -> Self {
        Self {
            transport: I2cTransport {
                i2c,
                address: Self::LSM6DS3TR_ID,
            },
        }
    }
}

#[repr(u8)]
enum RegisterAddress {
    WhoAmI = 0x0F,
}

impl<TransportType> Lsm6ds3tr<TransportType>
where
    TransportType: Transport,
{
    const WHO_AM_I_ID: u8 = 0b01101010;

    pub async fn read_whoami(&mut self) -> Result<u8, TransportType::Error> {
        let mut buffer = [0u8; 1];
        self.transport
            .read(RegisterAddress::WhoAmI as u8, &mut buffer)
            .await?;
        Ok(buffer[0])
    }

    pub async fn check(&mut self) -> Result<bool, TransportType::Error> {
        Ok(self.read_whoami().await? == Self::WHO_AM_I_ID)
    }
}
