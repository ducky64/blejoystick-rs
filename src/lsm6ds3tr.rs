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
#[derive(FromBits, Clone, Copy)]
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
#[derive(FromBits, Clone, Copy)]
pub enum FsXl {
    G2 = 0b00,
    G16 = 0b01,
    G4 = 0b10,
    G8 = 0b11,
}

impl FsXl {
    pub fn full_scale_g(&self) -> u8 {
        match self {
            FsXl::G2 => 2,
            FsXl::G4 => 4,
            FsXl::G8 => 8,
            FsXl::G16 => 16,
        }
    }
}

#[bitsize(1)]
#[derive(FromBits, Clone, Copy)]
pub enum Bw0Xl {
    Hz1k5 = 0b0,
    Hz400 = 0b1,
}

#[bitsize(8)]
#[derive(FromBits, Clone, Copy)]
struct Ctrl1Struct {
    bw0_xl: Bw0Xl,
    lpf1_bw_sel: bool,
    fs_xl: FsXl,
    odr_xl: OdrXl,
}

#[bitsize(4)]
#[derive(FromBits, Clone, Copy)]
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
#[derive(FromBits, Clone, Copy)]
pub enum FsG {
    Dps125 = 0b001,
    Dps245 = 0b000,
    Dps500 = 0b010,
    Dps1000 = 0b100,
    Dps2000 = 0b110,
    #[fallback]
    Reserved,
}

impl FsG {
    pub fn full_scale_dps(&self) -> u16 {
        match self {
            FsG::Dps125 => 125,
            FsG::Dps245 => 245,
            FsG::Dps500 => 500,
            FsG::Dps1000 => 1000,
            FsG::Dps2000 => 2000,
            FsG::Reserved => 0, // should not happen
        }
    }
}

#[bitsize(8)]
#[derive(FromBits, Clone, Copy)]
struct Ctrl2Struct {
    _reserved: u1,
    fs_g: FsG,
    odr_g: OdrG,
}

#[bitsize(8)]
#[derive(FromBits, Clone, Copy)]
struct Ctrl3CStruct {
    sw_reset: bool,
    ble: bool,
    if_inc: bool,
    sim: bool,
    pp_od: bool,
    h_lactive: bool,
    bdu: bool,
    boot: bool,
}
impl Ctrl3CStruct {
    pub fn default() -> Self {
        Self::new(false, false, true, false, false, false, false, false)
    }
}

#[bitsize(8)]
#[derive(FromBits, Clone, Copy)]
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

pub struct Lsm6ds3tr<TransportType>
where
    TransportType: Transport,
{
    transport: TransportType,
    xl_config: Option<(OdrXl, FsXl)>,
    g_config: Option<(OdrG, FsG)>,
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
            xl_config: None,
            g_config: None,
        }
    }
}

#[derive(Debug, defmt::Format)] // TODO feature gate
pub enum NormalizationError<TransportError> {
    Transport(TransportError),
    NotConfigured,
}

impl<TransportError> From<TransportError> for NormalizationError<TransportError> {
    fn from(err: TransportError) -> Self {
        NormalizationError::Transport(err)
    }
}

impl<TransportType> Lsm6ds3tr<TransportType>
where
    TransportType: Transport,
{
    const WHO_AM_I_ID: u8 = 0b01101010;

    /// Returns the value of the WHO_AM_I register
    pub async fn read_whoami(&mut self) -> Result<u8, TransportType::Error> {
        let mut buffer = [0u8; 1];
        self.transport
            .read(RegisterAddress::WhoAmI as u8, &mut buffer)
            .await?;
        Ok(buffer[0])
    }

    /// Reads the WHO_AM_I register, checking the value against the expected for this chip
    pub async fn check(&mut self) -> Result<bool, TransportType::Error> {
        Ok(self.read_whoami().await? == Self::WHO_AM_I_ID)
    }

    /// Issues a software reset
    pub async fn reset(&mut self) -> Result<(), TransportType::Error> {
        let mut ctrl3 = Ctrl3CStruct::default();
        ctrl3.set_sw_reset(true);
        self.transport
            .write_u8(RegisterAddress::Ctrl3C as u8, ctrl3.value)
            .await?;
        Ok(())
    }

    /// Sets the accelerometer configuration
    pub async fn config_xl(&mut self, odr: OdrXl, fs: FsXl) -> Result<(), TransportType::Error> {
        let ctrl1 = Ctrl1Struct::new(Bw0Xl::Hz1k5, false, fs, odr);
        self.transport
            .write_u8(RegisterAddress::Ctrl1Xl as u8, ctrl1.value)
            .await?;
        self.xl_config = Some((odr, fs));
        Ok(())
    }

    /// Sets the gyroscope configuration
    pub async fn config_g(&mut self, odr: OdrG, fs: FsG) -> Result<(), TransportType::Error> {
        let ctrl2 = Ctrl2Struct::new(fs, odr);
        self.transport
            .write_u8(RegisterAddress::Ctrl2G as u8, ctrl2.value)
            .await?;
        self.g_config = Some((odr, fs));
        Ok(())
    }

    /// Returns whether there is new data, from the status register
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

    /// Reads the temperature, returning the raw data (0 = 25c, 256 LSB/C)
    pub async fn read_temp_raw(&mut self) -> Result<i16, TransportType::Error> {
        let mut buffer = [0u8; 2];
        self.transport
            .read(RegisterAddress::OutTempL as u8, &mut buffer)
            .await?;
        Ok(i16::from_le_bytes(buffer))
    }

    /// Reads the temperature, converted to Celsius, assuming center temperature offset
    pub async fn read_temp_celsius(&mut self) -> Result<f32, TransportType::Error> {
        let raw = self.read_temp_raw().await?;
        let temp_c = (raw as f32) / 256.0 + 25.0;
        Ok(temp_c)
    }

    /// Reads the accelerometer, returning the raw u16 triple
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

    /// Reads the accelerometer, returning the normalized f32 triple in units of g
    pub async fn read_xl_g(
        &mut self,
    ) -> Result<(f32, f32, f32), NormalizationError<TransportType::Error>> {
        let scale = if let Some((_, fs)) = self.xl_config {
            fs.full_scale_g() as f32 / 32768.0 // 16-bit signed
        } else {
            return Err(NormalizationError::NotConfigured);
        };

        let (x_raw, y_raw, z_raw) = self.read_xl_raw().await?;
        Ok((
            x_raw as f32 * scale,
            y_raw as f32 * scale,
            z_raw as f32 * scale,
        ))
    }

    /// Reads the gyroscope, returning the raw u16 triple
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
