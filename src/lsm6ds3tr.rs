use crate::prelude::*;
use bilge::prelude::*;
use embedded_hal_async::i2c::{I2c, Operation};

// TODO separate out into own repo

/// Transport layer that supports both SPI and I2C
pub trait Transport {
    type Error;
    async fn write_u8(&mut self, addr: u8, data: u8) -> Result<(), Self::Error>;
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

    async fn write_u8(&mut self, addr: u8, data: u8) -> Result<(), I2cType::Error> {
        self.i2c.write(self.address, &[addr, data]).await?;
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

#[allow(dead_code)]
#[repr(u8)]
enum RegisterAddress {
    WhoAmI = 0x0F,
    Ctrl1Xl = 0x10,
    Ctrl2G = 0x11,
    Ctrl3C = 0x12,
    StatusReg = 0x1E,
    OutTempL = 0x20,
    OutTempH = 0x21,
    OutXLG = 0x22,  // gyro pitch rate low
    OutXHG = 0x23,  // gyro pitch rate high
    OutYLG = 0x24,  // gyro roll rate low
    OutYHG = 0x25,  // gyro roll rate high
    OutZLG = 0x26,  // gyro yaw rate low
    OutZHG = 0x27,  // gyro yaw rate high
    OutXLXl = 0x28, // accelerometer x low
    OutXHXl = 0x29, // accelerometer x high
    OutYLXl = 0x2A, // accelerometer y low
    OutYHXl = 0x2B, // accelerometer y high
    OutZLXl = 0x2C, // accelerometer z low
    OutZHXl = 0x2D, // accelerometer z high
}

#[bitsize(4)]
#[derive(FromBits)]
pub enum OdrXl {
    PowerDown = 0b0000,
    Hz12_5 = 0b0001,
    Hz26 = 0b0010,
    Hz52 = 0b0011,
    Hz104 = 0b0100,
    Hz208 = 0b0101,
    Hz416 = 0b0110,
    Hz833 = 0b0111,
    Hz1k66 = 0b1000,
    Hz3k33 = 0b1001,
    Hz6k66 = 0b1010,
    Hz1_6 = 0b1011,
    #[fallback]
    Reserved,
}

#[bitsize(2)]
#[derive(FromBits)]
pub enum FsXl {
    G2 = 0b00,
    G16 = 0b01,
    G4 = 0b10,
    G8 = 0b11,
}

#[bitsize(1)]
#[derive(FromBits)]
pub enum Bw0Xl {
    Hz1k5 = 0b0,
    Hz400 = 0b1,
}

#[bitsize(8)]
#[derive(FromBits)]
struct Ctrl1Struct {
    bw0_xl: Bw0Xl,
    lpf1_bw_sel: bool,
    fs_xl: FsXl,
    odr_xl: OdrXl,
}

#[bitsize(4)]
#[derive(FromBits)]
pub enum OdrG {
    PowerDown = 0b0000,
    Hz12_5 = 0b0001,
    Hz26 = 0b0010,
    Hz52 = 0b0011,
    Hz104 = 0b0100,
    Hz208 = 0b0101,
    Hz416 = 0b0110,
    Hz833 = 0b0111,
    Hz1k66 = 0b1000,
    Hz3k33 = 0b1001,
    Hz6k66 = 0b1010,
    #[fallback]
    Reserved,
}

#[bitsize(3)]
#[derive(FromBits)]
pub enum FsG {
    Dps125 = 0b001,
    Dps245 = 0b000,
    Dps500 = 0b010,
    Dps1000 = 0b100,
    Dps2000 = 0b110,
    #[fallback]
    Reserved,
}

#[bitsize(8)]
#[derive(FromBits)]
struct Ctrl2Struct {
    _reserved: u1,
    fs_xl: FsG,
    odr_xl: OdrG,
}

#[bitsize(8)]
#[derive(FromBits)]
struct StatusRegStruct {
    xlda: bool,
    gda: bool,
    tda: bool,
    _reserved: u5,
}

#[derive(Debug, defmt::Format)] // TODO feature gate
pub struct NewDataAvailable {
    temp: bool,
    gyro: bool,
    accelerometer: bool,
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

    pub async fn config_xl(&mut self, odr: OdrXl, fs: FsXl) -> Result<(), TransportType::Error> {
        let ctrl1 = Ctrl1Struct::new(Bw0Xl::Hz1k5, false, fs, odr);
        self.transport
            .write_u8(RegisterAddress::Ctrl1Xl as u8, ctrl1.value)
            .await?;
        Ok(())
    }

    pub async fn new_data(&mut self) -> Result<NewDataAvailable, TransportType::Error> {
        let mut buffer = [0u8; 1];
        self.transport
            .read(RegisterAddress::StatusReg as u8, &mut buffer)
            .await?;
        let status_reg = StatusRegStruct::from(buffer[0]);
        Ok(NewDataAvailable {
            temp: status_reg.tda(),
            gyro: status_reg.gda(),
            accelerometer: status_reg.xlda(),
        })
    }

    pub async fn read_temp_raw(&mut self) -> Result<u16, TransportType::Error> {
        let mut buffer = [0u8; 2];
        self.transport
            .read(RegisterAddress::OutTempL as u8, &mut buffer)
            .await?;
        Ok(u16::from_le_bytes(buffer))
    }

    pub async fn read_xl_raw(&mut self) -> Result<(i16, i16, i16), TransportType::Error> {
        let mut buffer = [0u8; 6];
        self.transport
            .read(RegisterAddress::OutXLXl as u8, &mut buffer)
            .await?;
        let x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let z = i16::from_le_bytes([buffer[4], buffer[5]]);
        Ok((x, y, z))
    }

    pub async fn read_g_raw(&mut self) -> Result<(i16, i16, i16), TransportType::Error> {
        let mut buffer = [0u8; 6];
        self.transport
            .read(RegisterAddress::OutXLG as u8, &mut buffer)
            .await?;
        let x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let z = i16::from_le_bytes([buffer[4], buffer[5]]);
        Ok((x, y, z))
    }
}
