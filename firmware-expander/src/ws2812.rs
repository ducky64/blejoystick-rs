use embedded_hal::spi::{Mode, Phase, Polarity, SpiBus};

use smart_leds_trait::{SmartLedsWriteAsync, RGB8};


struct Ws2812SpiCustom<'a, Word, SPI> {
    spi: SPI,
    zero: Word,
    one: Word,
    bits_per_bit: u8,
    bits_per_word: u8,
    buffer: &'a mut [Word]
}

impl <'a, Word: Copy + 'static, SPI> Ws2812SpiCustom<'a, Word, SPI> 
where
    SPI: SpiBus<Word>
{
    /// Creates a new instance given a SPI bus.
    /// zero: SPI data for a smartled zero bit
    /// one: SPI data for a smartled one bit
    /// bits_per_bit: number of SPI bits per smartled bit
    /// bits_per_word: number of smartled bits per SPI word
    /// buffer: buffer to use for SPI data, large enough for all bits that need to be transferred
    pub fn new(spi: SPI, zero: Word, one: Word, bits_per_bit: u8, bits_per_word: u8, buffer: &'a mut [Word]) -> Self {
        Self { spi, zero, one, bits_per_bit, bits_per_word, buffer }
    }
}

impl <'a, Word: Copy + 'static, SPI> SmartLedsWriteAsync
for Ws2812SpiCustom<'a, Word, SPI> 
where
    SPI: SpiBus<Word>
{
    type Error = SPI::Error;
    type Color = RGB8;

    async fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = I>,
        I: Into<Self::Color> 
    {
        Ok(())
    }
}