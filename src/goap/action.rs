use super::State;
use num_traits::{Num, Zero};
use std::fmt::Debug;

/// An action represents some operation that can be performed by an agent given some world state.
///
/// An example could be "arming", "navigating" etc...
pub trait Action<S: State>: Debug {
    type Cost: Num + Zero;
    /// Executing an action mutates the state and returns the cost of the action.
    ///
    /// We couple execution and cost calculation as often a cost requires the same
    /// work as state mutation (e.g. pathfinding).
    fn forward(&self, state: &mut S) -> anyhow::Result<Self::Cost>;
}

/// A boxed action is itself an action.
impl<T, S, C> Action<S> for Box<T>
where
    T: Action<S, Cost = C>,
    S: State,
    C: Num + Zero,
{
    type Cost = C;
    fn forward(&self, state: &mut S) -> anyhow::Result<Self::Cost> {
        self.as_ref().forward(state)
    }
}

/// A bundle of actions is itself an action, where the actions are executed sequentially.
impl<T, S, C> Action<S> for Vec<T>
where
    T: Action<S, Cost = C>,
    S: State,
    C: Num + Zero,
{
    type Cost = C;
    fn forward(&self, state: &mut S) -> anyhow::Result<Self::Cost> {
        let mut cumulative_cost = Self::Cost::zero();
        for action in self.iter() {
            cumulative_cost = cumulative_cost + action.forward(state)?;
        }
        Ok(cumulative_cost)
    }
}
