use anyhow::anyhow;
use ordered_float::OrderedFloat;

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Value {
    Bool(bool),
    U8(u8),
    U32(u32),
    USize(usize),
    U64(u64),
    I32(i32),
    I64(i64),
    Bytes(Vec<u8>),
    Float32(OrderedFloat<f32>),
    Float64(OrderedFloat<f64>),
    H3Cell(h3o::CellIndex),
    String(String),
}

impl From<bool> for Value {
    fn from(value: bool) -> Self {
        Value::Bool(value)
    }
}

impl From<String> for Value {
    fn from(value: String) -> Self {
        Value::String(value)
    }
}

impl From<&str> for Value {
    fn from(value: &str) -> Self {
        Value::String(value.to_string())
    }
}

impl From<u8> for Value {
    fn from(value: u8) -> Self {
        Value::U8(value)
    }
}

impl From<u32> for Value {
    fn from(value: u32) -> Self {
        Value::U32(value)
    }
}

impl From<usize> for Value {
    fn from(value: usize) -> Self {
        Value::USize(value)
    }
}

impl From<i32> for Value {
    fn from(value: i32) -> Self {
        Value::I32(value)
    }
}

impl From<i64> for Value {
    fn from(value: i64) -> Self {
        Value::I64(value)
    }
}

impl From<f32> for Value {
    fn from(value: f32) -> Self {
        Value::Float32(OrderedFloat(value))
    }
}

impl From<f64> for Value {
    fn from(value: f64) -> Self {
        Value::Float64(OrderedFloat(value))
    }
}

impl From<h3o::CellIndex> for Value {
    fn from(value: h3o::CellIndex) -> Self {
        Value::H3Cell(value)
    }
}

macro_rules! impl_try_from_value {
    ($t:ty, $variant:ident) => {
        impl TryFrom<Value> for $t {
            type Error = anyhow::Error;
            fn try_from(value: Value) -> Result<Self, Self::Error> {
                match value {
                    Value::$variant(val) => Ok(val),
                    _ => Err(anyhow!(
                        "Expected Value::{} - got {:?}",
                        stringify!($variant),
                        value
                    )),
                }
            }
        }
    };
    // Special case for floats wrapped in OrderedFloat
    ($t:ty, $variant:ident, ordered_float) => {
        impl TryFrom<Value> for $t {
            type Error = anyhow::Error;
            fn try_from(value: Value) -> Result<Self, Self::Error> {
                match value {
                    Value::$variant(val) => Ok(val.into_inner()),
                    _ => Err(anyhow!(
                        "Expected Value::{} - got {:?}",
                        stringify!($variant),
                        value
                    )),
                }
            }
        }
    };
}

impl_try_from_value!(bool, Bool);
impl_try_from_value!(u8, U8);
impl_try_from_value!(u32, U32);
impl_try_from_value!(usize, USize);
impl_try_from_value!(u64, U64);
impl_try_from_value!(i32, I32);
impl_try_from_value!(i64, I64);
impl_try_from_value!(String, String);
impl_try_from_value!(Vec<u8>, Bytes);
impl_try_from_value!(f32, Float32, ordered_float);
impl_try_from_value!(f64, Float64, ordered_float);
impl_try_from_value!(h3o::CellIndex, H3Cell);

#[cfg(test)]
mod tests {
    use crate::goap::Value;

    #[test]
    fn can_try_from_bool_value() {
        let value = Value::Bool(true);
        let value_as_bool: bool = value.try_into().unwrap();
        assert_eq!(value_as_bool, true);
    }

    #[test]
    fn can_try_from_u8_value() {
        let value = Value::U8(42);
        let value_as_u8: u8 = value.try_into().unwrap();
        assert_eq!(value_as_u8, 42);
    }

    #[test]
    fn can_try_from_u32_value() {
        let value = Value::U32(123456);
        let value_as_u32: u32 = value.try_into().unwrap();
        assert_eq!(value_as_u32, 123456);
    }

    #[test]
    fn can_try_from_usize_value() {
        let value = Value::USize(98765);
        let value_as_usize: usize = value.try_into().unwrap();
        assert_eq!(value_as_usize, 98765);
    }

    #[test]
    fn can_try_from_u64_value() {
        let value = Value::U64(1234567890123456789);
        let value_as_u64: u64 = value.try_into().unwrap();
        assert_eq!(value_as_u64, 1234567890123456789);
    }

    #[test]
    fn can_try_from_i32_value() {
        let value = Value::I32(-123456);
        let value_as_i32: i32 = value.try_into().unwrap();
        assert_eq!(value_as_i32, -123456);
    }

    #[test]
    fn can_try_from_i64_value() {
        let value = Value::I64(-9876543210);
        let value_as_i64: i64 = value.try_into().unwrap();
        assert_eq!(value_as_i64, -9876543210);
    }

    #[test]
    fn can_try_from_string_value() {
        let value = Value::String("hello world".to_string());
        let value_as_string: String = value.try_into().unwrap();
        assert_eq!(value_as_string, "hello world");
    }

    #[test]
    fn can_try_from_bytes_value() {
        let bytes = vec![1, 2, 3, 4];
        let value = Value::Bytes(bytes.clone());
        let value_as_bytes: Vec<u8> = value.try_into().unwrap();
        assert_eq!(value_as_bytes, bytes);
    }

    #[test]
    fn can_try_from_f32_value() {
        let float_value = 3.14_f32;
        let value = Value::Float32(ordered_float::OrderedFloat(float_value));
        let value_as_f32: f32 = value.try_into().unwrap();
        assert_eq!(value_as_f32, float_value);
    }

    #[test]
    fn can_try_from_f64_value() {
        let float_value = -2.718281828459045;
        let value = Value::Float64(ordered_float::OrderedFloat(float_value));
        let value_as_f64: f64 = value.try_into().unwrap();
        assert_eq!(value_as_f64, float_value);
    }

    #[test]
    fn can_try_from_h3cell_value() {
        let cell = h3o::CellIndex::first(h3o::Resolution::Nine);
        let value = Value::H3Cell(cell);
        let value_as_cell: h3o::CellIndex = value.try_into().unwrap();
        assert_eq!(value_as_cell, cell);
    }
}
