pub trait Action: Clone {
    type Key: Clone + std::hash::Hash + Eq;

    /// Each action must have a key, which is used to uniquely determine a game.
    /// We allow this key to be a series of bytes of a known length.
    fn key(&self) -> Self::Key;
}
