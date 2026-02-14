pub trait State: Clone {
    type Key: std::hash::Hash + Eq + Clone;

    fn key(&self) -> Self::Key;

    fn is_terminal(&self) -> bool;
}
