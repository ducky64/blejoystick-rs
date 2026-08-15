use core::ops::{BitOr, Shl};

use embedded_hal_async::spi::SpiBus;

use smart_leds_trait::{SmartLedsWriteAsync, RGB8};


pub struct Ws2812SpiCustom<'a, Word, SPI> {
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
    /// zero: SPI data for a smartled zero bit, occupying the LSbits
    /// one: SPI data for a smartled one bit, occupying the LSbits
    /// bits_per_bit: number of SPI bits per smartled bit
    /// bits_per_word: number of smartled bits per SPI word
    /// buffer: buffer to use for SPI data, large enough for all bits that need to be transferred
    pub fn new(spi: SPI, zero: Word, one: Word, bits_per_bit: u8, bits_per_word: u8, buffer: &'a mut [Word]) -> Self {
        Self { spi, zero, one, bits_per_bit, bits_per_word, buffer }
    }
}

impl <'a, Word, SPI> SmartLedsWriteAsync
for Ws2812SpiCustom<'a, Word, SPI> 
where
    SPI: SpiBus<Word>,
    Word: Copy + From<u8> + Shl<u8, Output = Word> + BitOr<Word, Output = Word> + 'static,
{
    type Error = SPI::Error;
    type Color = RGB8;

    async fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = I>,
        I: Into<Self::Color>
    {
        let mut buffer_index = 0;
        let mut bit_index = 0;
        let mut word_buffer: Word = 0.into();  // accumulates SPI bits per word

        for item in iterator {
            let item = item.into();
            for color_byte in [item.g, item.r, item.b] {
                for bit in (0..8).rev() {
                    let bit_value = (color_byte >> bit) & 0x01;
                    let bit_word = if bit_value == 0 { self.zero } else { self.one };
                    word_buffer = (word_buffer << self.bits_per_bit) | bit_word;
                    bit_index += 1;
                    if bit_index >= self.bits_per_word {
                      self.buffer[buffer_index] = word_buffer;
                      word_buffer = 0.into();
                      buffer_index += 1;
                      bit_index = 0;
                    }
                }
            }
        }

        self.spi.write(&self.buffer[0..buffer_index]).await
    }
}
