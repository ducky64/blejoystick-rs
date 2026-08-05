use defmt::{Format, Formatter};
use heapless::LinearMap;
use serde::{Deserialize, Serialize};

// LinearMap does not have a defmt Format impl, this wrapper provides defmt support
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub(crate) struct FormatLinearMap<K: core::cmp::Eq, V, const N: usize>(pub LinearMap<K, V, N>);
impl<'a, K: Format + core::cmp::Eq, V: Format, const N: usize> Format for FormatLinearMap<K, V, N> {
    fn format(&self, fmt: Formatter) {
        defmt::write!(fmt, "{{");

        for (i, (k, v)) in self.0.iter().enumerate() {
            if i > 0 {
                defmt::write!(fmt, ", ");
            }
            defmt::write!(fmt, "{}: {}", k, v);
        }
        defmt::write!(fmt, "}}");
    }
}

// it turns out a bunch of interp crates are not no_std
use core::ops::{Add, Div, Mul, Sub};
pub(crate) fn interpolate_1d<X, Y>(breakpoints: &[(X, Y)], x: X) -> Y
where
    X: PartialOrd + Copy + Sub<Output = X>,
    Y: Copy + Add<Output = Y> + Sub<Output = Y> + Mul<X, Output = Y> + Div<X, Output = Y>,
{
    assert!(!breakpoints.is_empty(), "breakpoints slice cannot be empty");

    for window in breakpoints.windows(2) {
        let (x0, y0) = window[0];
        let (x1, y1) = window[1];

        if x < x0 {
            return y0; // clamp to lowest
        } else if x >= x0 && x <= x1 {
            let dx = x1 - x0;
            let dy = y1 - y0;
            let offset = x - x0;
            return y0 + (dy * offset) / dx;
        }
    }
    breakpoints.last().unwrap().1 // clamp to highest
}
