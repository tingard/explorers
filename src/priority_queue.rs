use std::{cmp::Reverse, collections::BinaryHeap};

/// A single entry in the queue's underlying heap.
///
/// `Eq`/`Ord` are implemented manually (rather than derived) so that they only ever consider
/// the priority. Deriving them over both fields would make `Eq` inconsistent with `Ord`, which
/// violates the contract `BinaryHeap` relies on.
pub(crate) struct QueueEntry<P: Ord, I>(Reverse<P>, I);

impl<P: Ord, I> PartialEq for QueueEntry<P, I> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<P: Ord, I> Eq for QueueEntry<P, I> {}

impl<P: Ord, I> PartialOrd for QueueEntry<P, I> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<P: Ord, I> Ord for QueueEntry<P, I> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.cmp(&other.0)
    }
}

/// A min-priority queue: `pop` always returns the item with the *lowest* priority.
///
/// Internally this wraps a `BinaryHeap` (a max-heap) with priorities stored as `Reverse<P>`,
/// so callers never need to negate their priorities to get min-heap behaviour. This also means
/// `P` no longer needs to be negatable, so unsigned cost types (`u32`, `usize`, ...) work fine.
pub(crate) struct PriorityQueue<P: Ord, I>(BinaryHeap<QueueEntry<P, I>>);

impl<P: Ord, I> PriorityQueue<P, I> {
    pub fn new() -> Self {
        Self(BinaryHeap::new())
    }

    pub fn push(&mut self, item: I, priority: P) {
        self.0.push(QueueEntry(Reverse(priority), item));
    }

    pub fn pop(&mut self) -> Option<(I, P)> {
        self.0.pop().map(|entry| (entry.1, entry.0.0))
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
        // Lowest priority pops first.
        assert_eq!(queue.pop(), Some(("a".to_string(), 0u32)));
        assert_eq!(queue.pop(), Some(("c".to_string(), 1u32)));
        assert_eq!(queue.pop(), Some(("b".to_string(), 2u32)));
    }
}
