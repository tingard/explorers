use std::time::Duration;
use thiserror::Error;

#[derive(Clone, Debug, Error)]
pub enum WadooErr {
    #[error("At least one child is required.")]
    EmptyChildren,
}

/// Errors returned by [`crate::astar::astar`].
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum AstarError {
    /// The frontier was exhausted before reaching the goal: there is genuinely no path from
    /// `start` to `goal` in the graph as defined by `get_neighbors`. Retrying with a larger
    /// `max_nodes_searched` or `max_search_time` will not help.
    #[error("No path exists between the start and goal nodes")]
    NoPathExists,
    /// The search stopped because `max_nodes_searched` was reached before the goal was found.
    /// Unlike [`AstarError::NoPathExists`], a path may still exist - retrying with a larger
    /// budget might succeed.
    #[error("No path found within {searched} searched nodes")]
    NodeBudgetExhausted { searched: usize },
    /// The search stopped because `max_search_time` elapsed before the goal was found. As with
    /// [`AstarError::NodeBudgetExhausted`], a path may still exist for a larger time budget.
    #[error("Search timed out after {elapsed:?}")]
    TimedOut { elapsed: Duration },
}
