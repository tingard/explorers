use std::{collections::BinaryHeap, hash::Hash};

#[derive(PartialEq, Eq)]
pub(crate) struct QueueEntry<P: Eq + Ord, I: Eq + Hash>(P, I);

impl<P: Eq + Ord, I: Eq + Hash> PartialOrd for QueueEntry<P, I> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<P: Eq + Ord, I: Eq + Hash> Ord for QueueEntry<P, I> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.cmp(&other.0)
    }
}

pub(crate) struct PriorityQueue<P: Eq + Ord, I: Eq + Hash>(BinaryHeap<QueueEntry<P, I>>);

impl<P: Eq + Ord, I: Eq + Hash> PriorityQueue<P, I> {
    pub fn new() -> Self {
        let q: BinaryHeap<QueueEntry<P, I>> = BinaryHeap::new();
        Self(q)
    }

    pub fn push(&mut self, item: I, priority: P) {
        self.0.push(QueueEntry(priority, item));
    }

    pub fn pop(&mut self) -> Option<(I, P)> {
        self.0.pop().map(|i| (i.1, i.0))
    }
}

#[cfg(test)]
mod tests {
    use super::PriorityQueue;

    #[test]
    fn test_simple_sorting() {
        let mut queue = PriorityQueue::new();
        queue.push("a".to_string(), 0u32);
        queue.push("b".to_string(), 2u32);
        queue.push("c".to_string(), 1u32);
        assert_eq!(queue.pop(), Some(("b".to_string(), 2u32)));
        assert_eq!(queue.pop(), Some(("c".to_string(), 1u32)));
        assert_eq!(queue.pop(), Some(("a".to_string(), 0u32)));
    }
}
