use tracing_subscriber::{EnvFilter, fmt, prelude::*};

use explorers::astar::{EdgeToNodeWithCost, astar};

#[derive(Debug, Clone, Copy)]
enum Edges {
    SubtractTwo,
    SubtractOne,
    DoNothing,
    AddOne,
    AddTwo,
}

fn main() {
    tracing_subscriber::registry()
        .with(fmt::layer())
        .with(EnvFilter::from_default_env())
        .init();
    let goal = 100_000i32;
    let result = astar()
        .is_goal(|n: &i32, goal: &i32| n == goal)
        // We can increment or decrement by 1 or 2
        .get_neighbors(|i: &i32| {
            vec![
                EdgeToNodeWithCost::new(Edges::SubtractTwo, i - 2, 3),
                EdgeToNodeWithCost::new(Edges::SubtractOne, i - 1, 2),
                EdgeToNodeWithCost::new(Edges::DoNothing, *i, 1),
                EdgeToNodeWithCost::new(Edges::AddOne, i + 1, 2),
                EdgeToNodeWithCost::new(Edges::AddTwo, i + 2, 3),
            ]
        })
        // Distance heuristic
        .heuristic(|i: &i32, g: &i32| (g - i).abs())
        .plan_path(&0i32, &goal);

    println!(
        "Path has len {:?}",
        result.expect("Found a path").path.len()
    );
}
