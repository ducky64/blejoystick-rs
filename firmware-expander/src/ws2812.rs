use core::ops::{BitOr, Shl, Shr};
use core::cmp::min;

use embedded_hal_async::spi::SpiBus;

use smart_leds_trait::{SmartLedsWriteAsync, RGB8};


pub trait NumBits {
    const BITS: u8;
}

impl NumBits for u8 { const BITS: u8 = 8; }
impl NumBits for u16 { const BITS: u8 = 16; }
impl NumBits for u32 { const BITS: u8 = 32; }
impl NumBits for u64 { const BITS: u8 = 64; }

pub trait TruncatedFrom<T> {
    fn truncated_from(value: T) -> Self;
}

impl<T> TruncatedFrom<T> for T {
    fn truncated_from(value: T) -> Self {
        value
    }
}

impl TruncatedFrom<u32> for u8 {
    fn truncated_from(value: u32) -> u8 {
        value as u8
    }
}

impl TruncatedFrom<u16> for u8 {
    fn truncated_from(value: u16) -> u8 {
        value as u8
    }
}

impl TruncatedFrom<u32> for u16 {
    fn truncated_from(value: u32) -> u16 {
        value as u16
    }
}

pub trait Ws2812SpiLookupTable {
    const INDEX_BITS: u8;  // number of bits this table indexes by
    type Output;  // return type

    /// Given BITS of smartled bits, return the corresponding SPI word and number of SPI bits
    fn get(&self, value: u8) -> (Self::Output, u8);
}


/// Define a simple lookup table indexed one bit at a time.
/// zero: SPI data for a smartled zero bit, occupying the LSbits
/// zero_bits: number of SPI bits of the zero value
/// one, one_bits: same for a smartled one bit
/// zero_bits, one_bits do not need to divide cleanly into a Word
///   the SPI buffer generation will build Words across smartled bit boundaries
pub struct OneBitWs2812LookupTable {
    zero: u8,
    zero_bits: u8,
    one: u8,
    one_bits: u8
}

impl OneBitWs2812LookupTable {
    pub fn new(zero: u8, zero_bits: u8, one: u8, one_bits: u8) -> Self {
        Self { zero, zero_bits, one, one_bits }
    }
}

impl Ws2812SpiLookupTable for OneBitWs2812LookupTable {
  const INDEX_BITS: u8 = 1;
  type Output = u8;

  fn get(&self, value: u8) -> (Self::Output, u8) {
      if value & 0x01 == 0 {
          (self.zero, self.zero_bits)
      } else {
          (self.one, self.one_bits)
      }
  }
}


pub struct MultiBitWs2812Lookup<T, const N: usize>
{
    table: [T; N],
    bits: [u8; N],
}

impl <T, const N: usize> MultiBitWs2812Lookup<T, N>
{
    const INDEX_MASK : u8 = (N as u8) - 1;

    pub fn new(zero: u8, zero_bits: u8, one: u8, one_bits: u8) -> Self {
        Self { [] }
    }
}

impl <T, const N: usize> Ws2812SpiLookupTable for MultiBitWs2812Lookup<T, N> 
where T: Copy
{
  const INDEX_BITS: u8 = N.ilog2() as u8;
  type Output = T;

  fn get(&self, value: u8) -> (Self::Output, u8) {
      (self.table[(value & Self::INDEX_MASK) as usize], self.bits[(value & Self::INDEX_MASK) as usize])
  }
}


/// WS2812 LED driver with customizable SPI word size and bit encoding
/// N is the maximum number of LEDs in the chain, for buffer sizing
/// WORDS_PER_COLOR is the maximum number of SPI words used to encode a single color of LED data
///   This encoding is needed for internal array sizing without generic const exprs
pub struct Ws2812SpiCustom<Word, Lut, SPI, const N: usize, const WORDS_PER_COLOR: usize> {
    spi: SPI,
    lut: Lut,
    buffer: [[[Word; WORDS_PER_COLOR]; 3]; N],
}

impl <Word: Copy + 'static, Lut, SPI, const N: usize, const WORDS_PER_COLOR: usize> Ws2812SpiCustom<Word, Lut, SPI, N, WORDS_PER_COLOR> 
where
    SPI: SpiBus<Word>,
    Lut: Ws2812SpiLookupTable,
    Lut::Output: Copy +  Shl<u8, Output = Lut::Output> + Shr<u8, Output = Lut::Output> + NumBits,
    Word: Copy + Default + TruncatedFrom<Lut::Output> + Shl<u8, Output = Word> + Shr<u8, Output = Word> + BitOr<Word, Output = Word> + NumBits + 'static,
{
    /// Creates a new instance given a SPI bus and lookup table defining how smartled bits are encoded into SPI bits.
    /// This requires the SPI driver to continuously transmit data, without inter-word gaps
    pub fn new(spi: SPI, lut: Lut) -> Self {
        Self { spi, lut, buffer: [[[Word::default(); WORDS_PER_COLOR]; 3]; N]}
    }

    fn flat_buffer(buffer: &mut [[[Word; WORDS_PER_COLOR]; 3]; N]) -> &mut [Word] {
        let len = N * 3 * WORDS_PER_COLOR;
        let ptr = buffer.as_mut_ptr() as *mut Word;
        unsafe { core::slice::from_raw_parts_mut(ptr, len) }
    }

    /// Write the colors to SPI bits in a buffer for transmission, returning the number of words.
    fn write_buffer<T, I>(lut: &Lut, iterator: T, buffer: &mut [Word]) -> usize 
    where
        T: IntoIterator<Item = I>,
        I: Into<RGB8>
    {
        const {
            assert!(8 % Lut::INDEX_BITS == 0, "Lut::INDEX_BITS must divide from 8 (RGB8 component)");
            assert!(Lut::INDEX_BITS <= 8, "Lut::INDEX_BITS must be <= 8 (RGB8 component)");
            assert!(Lut::Output::BITS >= Word::BITS, "Lut::Output must larger or equal to than Word");
        };
        
        let mut buffer_index = 0;  // of the current word being written
        let mut word_buffer: Word = Word::default();  // accumulates SPI bits per word, LSbit aligned
        let mut word_bits = 0;  // number of bits written into word_buffer

        for item in iterator {
            let item = item.into();
            for color_byte in [item.g, item.r, item.b] {
                let mut written_bits: u8 = 0;
                while written_bits < 8 {
                  let index_mask = (1 << Lut::INDEX_BITS) - 1;
                  let lut_index = (color_byte >> (8 - written_bits - Lut::INDEX_BITS)) & index_mask;
                  let (spi_data, mut spi_bits) = lut.get(lut_index);
                  written_bits += Lut::INDEX_BITS;

                  // spi_data is shifted into word_buffer MSbit first
                  // spi_bits tracks the most significant valid bit in spi_data
                  while spi_bits > 0 {
                      let shift_bits = min(spi_bits, Word::BITS - word_bits);
                      word_buffer = (word_buffer << shift_bits) | Word::truncated_from(spi_data >> (spi_bits - shift_bits));
                      word_bits += shift_bits;
                      spi_bits -= shift_bits;

                      if word_bits >= Word::BITS {
                          buffer[buffer_index] = word_buffer;
                          buffer_index += 1;
                          word_buffer = Word::default();
                          word_bits = 0;
                      }
                  }
                }
            }
        }

        // shift any leftover bits in the last word
        if word_bits > 0 {
            buffer[buffer_index] = word_buffer << (Word::BITS - word_bits);
            buffer_index += 1;
        }

        buffer_index
    }
}

impl <Word, Lut, SPI, const N: usize, const WORDS_PER_COLOR: usize> SmartLedsWriteAsync
for Ws2812SpiCustom<Word, Lut, SPI, N, WORDS_PER_COLOR> 
where
    SPI: SpiBus<Word>,
    Lut: Ws2812SpiLookupTable,
    Lut::Output: Copy +  Shl<u8, Output = Lut::Output> + Shr<u8, Output = Lut::Output> + NumBits,
    Word: Copy + Default + TruncatedFrom<Lut::Output> + Shl<u8, Output = Word> + Shr<u8, Output = Word> + BitOr<Word, Output = Word> + NumBits + 'static,
{
    type Error = SPI::Error;
    type Color = RGB8;

    async fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = I>,
        I: Into<Self::Color>
    {
        let buffer = Self::flat_buffer(&mut self.buffer);
        let buffer_size = Self::write_buffer(&self.lut, iterator, buffer);
        self.spi.write(&buffer[0..buffer_size]).await
    }
}
