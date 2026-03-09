use explorers::astar::{EdgeToNodeWithCost, astar};

#[derive(Debug, Clone, Copy)]
enum Edges {
    AddOne,
    SubtractOne,
    SubtractTwo,
    AddTwo,
}

fn main() {
    let goal = 10i32;
    let result = astar()
        .is_goal(|n: &i32, goal: &i32| n == goal)
        // We can increment or decrement by 1 or 2
        .get_neighbors(|i: &i32| {
            vec![
                EdgeToNodeWithCost::new(Edges::AddOne, i + 1, 1),
                EdgeToNodeWithCost::new(Edges::SubtractOne, i - 1, 1),
                EdgeToNodeWithCost::new(Edges::SubtractTwo, i - 2, 1),
                EdgeToNodeWithCost::new(Edges::AddTwo, i + 2, 1),
            ]
        })
        // Distance heuristic
        .heuristic(|i: &i32, g: &i32| (g - i).abs())
        .max_search_depth(1_000_000)
        .plan_path(&0i32, &goal);

    println!("Path is {:?}", result.expect("Found a path").path);
}
