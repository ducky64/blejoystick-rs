#![cfg_attr(not(test), no_std)]

pub mod input;
pub mod vial;

use fixed::types::{I16F16, I1F15, U0F16};

pub fn scale_bipolar(adc: U0F16, center: U0F16, fullscale: U0F16, deadzone: U0F16) -> I1F15 {
    // takes a adc reading [0, 1) and maps it to a bipolar range [-1, 1), with a configurable center, full-scale, and deadzone
    // full-scale and deadzone are both half-span
    // convert everything to larger working type
    let adc = I16F16::from_num(adc);
    let center = I16F16::from_num(center);
    let fullscale = I16F16::from_num(fullscale);
    let deadzone = I16F16::from_num(deadzone);

    let scale_except_deadzone = fullscale - deadzone;
    let adc_offset = if adc > center + deadzone {
        adc - center - deadzone
    } else if adc < center - deadzone {
        adc - (center - deadzone)
    } else {
        I16F16::ZERO
    };
    I1F15::saturating_from_num(I16F16::saturating_div(adc_offset, scale_except_deadzone))
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)]
    use super::*;

    #[test]
    fn test_scale_bipolar() {
        const CENTER: U0F16 = U0F16::lit("0.5");
        const FULLSCALE: U0F16 = U0F16::lit("0.4");
        const DEADZONE: U0F16 = U0F16::lit("0.0");
        assert_eq!(
            scale_bipolar(U0F16::lit("0.5"), CENTER, FULLSCALE, DEADZONE),
            I1F15::lit("0.0")
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.9"), CENTER, FULLSCALE, DEADZONE),
            I1F15::MAX
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.1"), CENTER, FULLSCALE, DEADZONE),
            I1F15::MIN
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.7"), CENTER, FULLSCALE, DEADZONE),
            I1F15::lit("0.5")
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.3"), CENTER, FULLSCALE, DEADZONE),
            I1F15::lit("-0.5")
        );

        // saturation behavior
        assert_eq!(
            scale_bipolar(U0F16::lit("0.95"), CENTER, FULLSCALE, DEADZONE),
            I1F15::MAX
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.05"), CENTER, FULLSCALE, DEADZONE),
            I1F15::MIN
        );
    }

    #[test]
    fn test_scale_bipolar_deadzone() {
        const CENTER: U0F16 = U0F16::lit("0.5");
        const FULLSCALE: U0F16 = U0F16::lit("0.4");
        const DEADZONE: U0F16 = U0F16::lit("0.1");
        assert_eq!(
            scale_bipolar(U0F16::lit("0.5"), CENTER, FULLSCALE, DEADZONE),
            I1F15::lit("0.0")
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.6"), CENTER, FULLSCALE, DEADZONE),
            I1F15::lit("0.0")
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.4"), CENTER, FULLSCALE, DEADZONE),
            I1F15::lit("0.0")
        );
        assert_eq!(
            scale_bipolar(U0F16::MAX, CENTER, FULLSCALE, DEADZONE),
            I1F15::MAX
        );
        assert_eq!(
            scale_bipolar(U0F16::MIN, CENTER, FULLSCALE, DEADZONE),
            I1F15::MIN
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.75"), CENTER, FULLSCALE, DEADZONE),
            I1F15::lit("0.5")
        );
        assert_eq!(
            scale_bipolar(U0F16::lit("0.25"), CENTER, FULLSCALE, DEADZONE),
            I1F15::lit("-0.5")
        );
    }
}
