use sequential_storage::map::{SerializationError, Value};
use serde::{Deserialize, Serialize};


pub struct StorageSerde<T>(pub T);

impl<'a, T> Value<'a> for StorageSerde<T> 
where 
    T: Serialize + Deserialize<'a> + Sized,
{
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        postcard::to_slice(&self.0, buffer)
            .map(|v| v.len())
            .map_err(|_| SerializationError::Custom(0))
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized {
        postcard::take_from_bytes(buffer)
            .map(|(v, rest)| (StorageSerde(v), buffer.len() - rest.len()))
            .map_err(|_| SerializationError::Custom(0))
    }
}
