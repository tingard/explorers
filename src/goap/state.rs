use std::fmt::Debug;
use std::hash::Hash;

/// State represents a world state which we navigate using actions. An action will modify the state.
/// Ideal state types are cheap to clone - consider using a struct which supports structural sharing.
pub trait State: Clone + Debug + Hash + Eq {}
