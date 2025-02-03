#![no_std]
#![no_main]

use core::error::Error;

use u8g2_fonts::{
    fonts::{u8g2_font_6x13_mr, u8g2_font_crox3cb_mr, u8g2_font_inr16_mf, u8g2_font_inr21_mf},
    FontRenderer,
};

pub const FONT0_NORMAL: u8g2_fonts::FontRenderer = FontRenderer::new::<u8g2_font_inr21_mf>();
pub const FONT0_SMALL: u8g2_fonts::FontRenderer = FontRenderer::new::<u8g2_font_inr16_mf>();
pub const FONT0_SMALLER: u8g2_fonts::FontRenderer = FontRenderer::new::<u8g2_font_6x13_mr>();

pub struct SensorData<T, const N: usize = 1> {
    pub sample_time: u64,
    pub samples: [T; N],
}

impl<T, const N: usize> SensorData<T, N>
where
    T: Default + core::marker::Copy,
{
    pub fn new() -> Self {
        SensorData {
            sample_time: 0,
            samples: [T::default(); N],
        }
    }
}

pub fn parse_float<const N: usize>(buffer: &[u8]) -> Result<[f32; N], ParseFloatError> {
    if buffer.len() >= N * 4 {
        // Safety data length is checked and contains atleast N*4 bytes
        // So atleast N floats can be formed
        // This could be done with TryFrom array trait but with this way the data length is only checked once
        let mut res = [0.0; N];
        for (idx, c) in buffer.chunks_exact(4).enumerate() {
            let ptr = c.as_ptr() as *const [u8; 4];
            res[idx] = f32::from_le_bytes(unsafe { *ptr });
        }

        Ok(res)
    } else {
        Err(ParseFloatError::NotEnoughBytes)
    }
}

#[derive(Debug)]
pub enum ParseFloatError {
    NotEnoughBytes,
}
