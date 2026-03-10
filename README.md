# Explore.re

This package is a collection of various algorithms and utilities for pathing and planning.

## Astar

The `explorers::astar` module provides an implementation of the A* pathfinding algorithm.

```rust
use explorers::astar::{astar, EdgeToNodeWithCost};

#[derive(Debug, Clone, Copy)]
enum Edges {
    AddOne,
    SubtractOne,
    SubtractTwo,
    AddTwo,
}

// Set up the search using the builder pattern
let result = astar()
    // We provide a function to check if we've reached the goal
    .is_goal(|n: &i32, goal: &i32| n == goal)
    // We define the traversibility of the graph
    .get_neighbors(|i: &i32| {
        vec![
            EdgeToNodeWithCost::new(Edges::AddOne, i + 1, 1),
            EdgeToNodeWithCost::new(Edges::SubtractOne, i - 1, 1),
            EdgeToNodeWithCost::new(Edges::SubtractTwo, i - 2, 1),
            EdgeToNodeWithCost::new(Edges::AddTwo, i + 2, 1),
        ]
    })
    // Provide the distance heuristic to guide the search
    .heuristic(|i: &i32, g: &i32| (g - i).abs())
    // Limit the search depth to prevent infinite searching
    .max_nodes_searched(1_000_000)
    .plan_path(&0i32, &10i32);
// Check we found a path
assert!(result.is_ok_and(|r| r.path[r.path.len() - 1].1 == goal));
```

## Goal-Oriented Action Planning

We can leverage the above a-star implementation as the backbone for action planning. Note that our implementation is highly flexible, and therefore it is up to you to adopt good practises. We suggest reading the [Nyx](https://icaps24.icaps-conference.org/program/workshops/keps-papers/KEPS-24_paper_18.pdf) planner paper alongside other documentation (such as [pyperplan](https://github.com/aibasel/pyperplan/blob/main/doc/documentation.md)).

To view an example of action planning using this library, see the [simple_goap](examples/goap_simple.rs) example.

To view an example of incorporating time in action planning, see the [goap_](examples/goap_time.rs) example.
