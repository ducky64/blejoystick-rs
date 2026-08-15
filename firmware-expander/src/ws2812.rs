use crate::prelude::*;

use core::ops::{BitOr, Shl, Shr};
use core::cmp::min;

use embedded_hal_async::spi::SpiBus;
use embassy_time::Timer;

use smart_leds_trait::{SmartLedsWriteAsync, RGB8};


pub trait NumBits {
    const BITS: u8;
}

impl NumBits for u8 { const BITS: u8 = 8; }
impl NumBits for u16 { const BITS: u8 = 16; }
impl NumBits for u32 { const BITS: u8 = 32; }
impl NumBits for u64 { const BITS: u8 = 64; }


/// WS2812 LED driver with customizable SPI word size and bit encoding
/// N is the maximum number of LEDs in the chain, for buffer sizing
/// WORDS_PER_COLOR is the maximum number of SPI words used to encode a single color of LED data
///   This encoding is needed for internal array sizing without generic const exprs
pub struct Ws2812SpiCustom<Word, SPI, const N: usize, const WORDS_PER_COLOR: usize> {
    spi: SPI,
    zero: Word,
    zero_bits: u8,
    one: Word,
    one_bits: u8,
    buffer: [[[Word; WORDS_PER_COLOR]; 3]; N],
}

impl <Word: Copy + 'static, SPI, const N: usize, const WORDS_PER_COLOR: usize> Ws2812SpiCustom<Word, SPI, N, WORDS_PER_COLOR> 
where
    SPI: SpiBus<Word>,
    Word: From<u8>
{
    /// Creates a new instance given a SPI bus.
    /// zero: SPI data for a smartled zero bit, occupying the LSbits
    /// zero_bits: number of SPI bits of the zero value
    /// one, one_bits: same for a smartled one bit
    /// zero_bits, one_bits do not need to divide cleanly into a Word
    ///   the SPI buffer generation will build Words across smartled bit boundaries
    /// this requires the SPI driver to continuously transmit data, without inter-word gaps
    pub fn new(spi: SPI, zero: Word, zero_bits: u8, one: Word, one_bits: u8) -> Self {
        Self { spi, zero, zero_bits, one, one_bits, buffer: [[[0.into(); WORDS_PER_COLOR]; 3]; N]}
    }

    fn flat_buffer(buffer: &mut [[[Word; WORDS_PER_COLOR]; 3]; N]) -> &mut [Word] {
        let len = N * 3 * WORDS_PER_COLOR;
        let ptr = buffer.as_mut_ptr() as *mut Word;
        unsafe { core::slice::from_raw_parts_mut(ptr, len) }
    }
}

impl <Word, SPI, const N: usize, const WORDS_PER_COLOR: usize> SmartLedsWriteAsync
for Ws2812SpiCustom<Word, SPI, N, WORDS_PER_COLOR> 
where
    SPI: SpiBus<Word>,
    Word: Copy + From<u8> + Shl<u8, Output = Word> + Shr<u8, Output = Word> + BitOr<Word, Output = Word> + NumBits + 'static,
{
    type Error = SPI::Error;
    type Color = RGB8;

    async fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = I>,
        I: Into<Self::Color>
    {
        let mut buffer_index = 0;  // of the current word being written
        let mut word_buffer: Word = 0.into();  // accumulates SPI bits per word
        let mut word_bits = 0;  // number of bits written into word_buffer

        let buffer = Self::flat_buffer(&mut self.buffer);

        for item in iterator {
            let item = item.into();
            for color_byte in [item.g, item.r, item.b] {
                for bit in (0..8).rev() {
                    let bit_value = (color_byte >> bit) & 0x01;
                    let (bit_data, bit_bits) = if bit_value == 0 { (self.zero, self.zero_bits) } else { (self.one, self.one_bits) };

                    word_bits += bit_bits;
                    // calculate the number of bits to shift the existing word.
                    // either the all the bits of this smartled bit or the remaining bits of the current word
                    let shift_bits = min(bit_bits, word_bits % Word::BITS);
                    word_buffer = (word_buffer << shift_bits) | (bit_data >> (bit_bits - shift_bits));

                    // assume that at most one word can be written per smartled bit, since everything is typed Word
                    if word_bits >= Word::BITS {
                        buffer[buffer_index] = word_buffer;
                        buffer_index += 1;
                        // though all the bits are written to word_buffer, word_bits tracks bit counts
                        // so the extra MSbits will be shifted eventually
                        word_buffer = bit_data;
                        word_bits -= Word::BITS;
                    }
                }
            }
        }
        // shift any leftover bits in the last word
        if word_bits > 0 {
            buffer[buffer_index] = word_buffer << (Word::BITS - word_bits);
            buffer_index += 1;
        }

        self.spi.write(&buffer[0..buffer_index]).await
    }
}
