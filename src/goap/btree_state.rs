use std::collections::BTreeMap;

use super::{State, Value};

/// A straightforward, albeit low-performance, state for GOAP planning.
#[derive(Clone, Debug, Default, Hash, PartialEq, Eq)]
pub struct BTreeState(BTreeMap<String, Value>);

impl BTreeState {
    /// Gets an iterator over the keys of the map, in sorted order.
    pub fn keys<'a>(&'a self) -> impl Iterator<Item = &'a str> + 'a {
        self.0.keys().map(|k| k.as_str())
    }

    /// Iterate over the key-value pairs in the map, in sorted order.
    pub fn items<'a>(&'a self) -> impl Iterator<Item = (String, &'a Value)> + 'a {
        self.0.iter().map(|(k, v)| (k.to_string(), v))
    }

    /// Iterate over the key-value pairs in the map, in sorted order.
    pub fn items_with_prefix<'a>(
        &'a self,
        prefix: &'a str,
    ) -> impl Iterator<Item = (&'a String, &'a Value)> {
        self.0
            .range(prefix.to_string()..)
            .take_while(move |(k, _)| k.starts_with(prefix))
            .map(|(k, v)| (k, v))
    }

    /// Return whether all of the values in the target state match the values in this state.
    pub fn matches(&self, target_state: &BTreeState) -> bool {
        for (k, v_other) in target_state.items() {
            if self.get(&k).map_or(true, |v_self| v_self != v_other) {
                return false;
            };
        }
        true
    }

    /// Set a key-value pair in the state, and return the new state.
    pub fn with(&self, key: &str, value: Value) -> Self {
        let mut new_state = self.clone();
        new_state.0.insert(key.to_string(), value);
        new_state
    }

    /// Inserts a key-value pair into the map.
    ///
    /// If the map did not have this key present, None is returned.
    ///
    ///If the map did have this key present, the value is updated, and the old value is returned.
    pub fn insert<S: ToString>(&mut self, key: S, value: Value) -> Option<Value> {
        self.0.insert(key.to_string(), value)
    }

    pub fn update<S, F>(&mut self, key: S, f: F) -> Option<Value>
    where
        S: ToString,
        F: FnOnce(&mut Value) -> Option<Value>,
    {
        self.0.get_mut(&key.to_string()).and_then(|v| f(v))
    }

    /// Returns a reference to the value corresponding to the key.
    ///
    /// The key may be any borrowed form of the map’s key type, but the ordering on the borrowed form must match the ordering on the key type.
    pub fn get(&self, key: &str) -> Option<&Value> {
        self.0.get(key)
    }

    /// Remove all entries in the map that start with the given prefix.
    pub fn clear_prefix(&mut self, prefix: &str) {
        if prefix.is_empty() {
            self.0.clear();
            return;
        }
        let prefix_range = prefix.to_string();
        let to_delete: Vec<_> = self
            .0
            .range(prefix_range..)
            .take_while(|(v, _)| v.starts_with(prefix))
            .map(|(k, _)| k)
            .cloned()
            .collect();

        for key in to_delete {
            self.0.remove(&key);
        }
    }

    pub fn manhattan_distance(&self, other: &Self) -> u32 {
        self.0.iter().fold(0, |acc, (k, v)| {
            let is_match = match other.get(k) {
                Some(other_value) => v.eq(other_value),
                None => v.eq(&Value::Bool(false)),
            };
            if is_match { acc } else { acc + 1 }
        })
    }
}

impl State for BTreeState {}

#[cfg(test)]
mod tests {
    use super::{BTreeState, Value};

    #[test]
    fn state_mutation_and_comparision_0() {
        let s0 = BTreeState::default().with("key0", 0u8.into());
        let s1 = s0.with("key1", Value::U8(1));
        assert!(!s0.matches(&s1));
        // However, since s1 still has key0, this passes
        assert!(s1.matches(&s0));
    }

    #[test]
    fn state_mutation_and_comparision_1() {
        let mut s0 = BTreeState::default().with("key0", 0u8.into());
        let new_val = s0.update("key0", |val| {
            let Value::U8(v) = val else {
                panic!("Unexpected value type")
            };
            *v += 1;
            Some(Value::U8(*v))
        });
        let Some(Value::U8(v)) = new_val else {
            panic!("Unexpected value type");
        };
        assert_eq!(v, 1);
        let Some(Value::U8(v)) = s0.get("key0") else {
            panic!("Unexpected value type");
        };
        assert_eq!(*v, 1);
    }
}
