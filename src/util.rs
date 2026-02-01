use defmt::{Format, Formatter};
use heapless::LinearMap;
use serde::{Deserialize, Serialize};

// LinearMap does not have a defmt Format impl, this wrapper provides defmt support
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FormatLinearMap<K: core::cmp::Eq, V, const N: usize>(pub LinearMap<K, V, N>);
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
