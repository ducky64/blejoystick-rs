use embedded_hal_async::i2c::I2c;
use smart_leds::RGB8;

enum Opcode {
    Resrved = 0x00,
    ReadBtns = 0x01, 
    WriteRgbIndex = 0x12,
    UpdateRgbs = 0x19,
}

pub(crate) struct Expander<I2C> {
    i2c: I2C
}

impl<I2C> Expander<I2C> 
    where I2C: I2c
{
    const I2C_ADDRESS: u8 = 0x42;

    pub fn new(i2c: I2C) -> Self {
        Self {
          i2c
        }
    }

    pub async fn write_rgb(&mut self, index: u8, data: RGB8) -> Result<(), I2C::Error> {
        self.i2c.write(Self::I2C_ADDRESS, &[Opcode::WriteRgbIndex as u8, index, data.r, data.g, data.b]).await?;
        Ok(())
    }

    pub async fn update_rgb(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(Self::I2C_ADDRESS, &[Opcode::UpdateRgbs as u8]).await?;
        Ok(())
    }

    pub async fn read_btns(&mut self) -> Result<u8, I2C::Error> {
        let mut buffer = [0u8; 1];
        self.i2c.write(Self::I2C_ADDRESS, &[Opcode::ReadBtns as u8]).await?;
        self.i2c.read(Self::I2C_ADDRESS, &mut buffer).await?;
        // seems to break with the I2C peripheral HAL on CH32 side
        // self.i2c.write_read(Self::I2C_ADDRESS, &[Opcode::ReadBtns as u8], &mut buffer).await?;
        Ok(buffer[0])
    }
}
