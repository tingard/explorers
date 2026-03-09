# Explore.re

This package is a collection of various algorithms and utilities for pathing and planning.

## Astar

The `explorers::astar` module provides an implementation of the A* pathfinding algorithm.

```rust
use explorers::astar::{astar, EdgeToNodeWithCost};
let goal = 10i32;
let result = astar()
    .is_goal(|n: &i32, goal: &i32| n == goal)
    // We can increment or decrement by 1 or 2
    .get_neighbors(|i: &i32| {
        vec![
            EdgeToNodeWithCost::new((), i + 1, 1),
            EdgeToNodeWithCost::new((), i - 1, 1),
            EdgeToNodeWithCost::new((), i - 2, 1),
            EdgeToNodeWithCost::new((), i + 2, 1),
        ]
    })
    // Distance heuristic
    .heuristic(|i: &i32, g: &i32| (g - i).abs())
    .max_search_depth(1_000_000)
    .plan_path(&0i32, &goal);
// Check we found a path
assert!(result.is_ok_and(|r| r.path[r.path.len() - 1].1 == goal));
```



## GOAP
