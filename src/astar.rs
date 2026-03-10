use anyhow::anyhow;
use num_traits::Num;

use bon::builder;
use hashbrown::HashMap;
use priority_queue::PriorityQueue;
use std::{
    fmt::Debug,
    hash::Hash,
    ops::Neg,
    time::{Duration, Instant},
};
use tracing::debug;

const MAX_NODES_SEARCHED_DEFAULT: usize = 1_000_000;

pub struct AstarResult<E, N, C: Num> {
    pub path: Vec<(Option<E>, N)>,
    pub cost: C,
    pub n_nodes_searched: usize,
}

impl<E, N, C> std::fmt::Debug for AstarResult<E, N, C>
where
    E: Debug,
    N: Debug,
    C: Num + Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AstarResult")
            .field("path", &self.path)
            .field("cost", &self.cost)
            .field("nodes_searched", &self.n_nodes_searched)
            .finish()
    }
}

#[derive(Debug, Clone)]
pub struct EdgeToNodeWithCost<E: Debug + Clone, N: Debug + Clone, C: Debug + Num> {
    pub edge: E,
    pub cost: C,
    pub resulting_node: N,
}

impl<E, N, C> From<(E, N, C)> for EdgeToNodeWithCost<E, N, C>
where
    E: Debug + Clone,
    N: Debug + Clone,
    C: Debug + Num,
{
    fn from((edge, resulting_node, cost): (E, N, C)) -> Self {
        Self {
            edge,
            resulting_node,
            cost,
        }
    }
}

impl<E, N, C> EdgeToNodeWithCost<E, N, C>
where
    E: Debug + Clone,
    N: Debug + Clone,
    C: Debug + Num,
{
    pub fn new(edge: E, resulting_node: N, cost: C) -> Self {
        Self {
            edge,
            resulting_node,
            cost,
        }
    }
}

/// A* algorithm implementation.
///
/// Returns an `AstarResult` object containing a path of `(edge: E, node: N)` pairs and resulting cost.
///
/// # Examples
///
/// ```
/// # use wadoo_rs::astar::{astar, EdgeToNodeWithCost};
/// let goal = 10i32;
/// let result = astar()
///     // How do we determine if we are done?
///     .is_goal(|n: &i32, goal: &i32| n == goal)
///     // We can increment or decrement by 1
///     .get_neighbors(|i: &i32| {
///         vec![
///             EdgeToNodeWithCost::new((), i + 1, 1),
///             EdgeToNodeWithCost::new((), i - 1, 1),
///         ]
///     })
///     // Distance heuristic
///     .heuristic(|i: &i32, g: &i32| (g - i).abs())
///     // Trigger the plan
///     .plan_path(&0i32, &goal)
///     .unwrap();
/// // Check we found a path
/// assert!(result.path[result.path.len() - 1].1 == goal);
/// ```
///
/// Generic types:
/// - `N`: Node type.
/// - `E`: Edge type - use () for no edges.
/// - `C`: Cost type - required to be negatable (not unsigned), sortable (i32, OrderedFloat<f32>) and clonable number.
#[builder(derive(Clone), finish_fn = plan_path)]
pub fn astar<N, E, C, G>(
    #[builder(finish_fn)] start: &N,
    #[builder(finish_fn)] goal: &G,
    is_goal: impl Fn(&N, &G) -> bool,
    get_neighbors: impl Fn(&N) -> Vec<EdgeToNodeWithCost<E, N, C>>,
    heuristic: impl Fn(&N, &G) -> C,
    max_nodes_searched: Option<usize>,
    max_search_time: Option<Duration>,
) -> anyhow::Result<AstarResult<E, N, C>>
where
    N: Hash + Eq + Clone + Debug,
    E: Clone + Debug,
    C: Num + Ord + Neg<Output = C> + Clone + Debug,
{
    let max_nodes_search = max_nodes_searched.unwrap_or(MAX_NODES_SEARCHED_DEFAULT);
    // Queue of the most promising nodes to explore next
    let mut frontier = PriorityQueue::<N, C>::new();
    frontier.push(start.clone(), C::zero());

    // Mapping from end node to an option node + edge combo
    let mut came_from: HashMap<N, Option<(N, E)>> = HashMap::new();
    came_from.insert(start.clone(), None);
    // Mapping to the cumulative cost of reaching a node
    let mut cost_so_far: HashMap<N, C> = HashMap::new();
    cost_so_far.insert(start.clone(), C::zero());

    let search_start = &max_search_time.map(|_| Instant::now());
    let max_search_time = max_search_time.unwrap_or(Duration::MAX);

    for n_nodes_searched in 0..max_nodes_search {
        if search_start.is_some_and(|start| start.elapsed() > max_search_time) {
            return Err(anyhow!("Search timed out"));
        }
        let Some((current, priority)) = frontier.pop() else {
            // TODO: Move to thiserror and setup custom error type
            return Err(anyhow!("No path found"));
        };

        debug!("Checking {current:?} (had priority {priority:?})");
        let cost = cost_so_far.get(&current).expect("Current has cost").clone();
        if is_goal(&current, goal) {
            debug!("Reached goal");
            let mut cursor = current.clone();
            let mut path = vec![current.clone()];
            let mut edges = vec![];
            loop {
                let Some(Some(prev)) = came_from.get(&cursor) else {
                    break;
                };
                debug!("{prev:?} -> {cursor:?}");
                cursor = prev.0.clone();
                path.push(prev.0.clone());
                edges.push(Some(prev.1.clone()));
            }
            path.reverse();
            edges.push(None);
            edges.reverse();
            return Ok(AstarResult {
                path: edges.into_iter().zip(path.into_iter()).collect(),
                cost: cost.clone(),
                n_nodes_searched,
            });
        }
        for neighbor in get_neighbors(&current) {
            debug!("\t{current:?} has neighbor {neighbor:?}");
            let new_cost = cost.clone() + neighbor.cost;
            if cost_so_far
                .get(&neighbor.resulting_node)
                .is_none_or(|v| &new_cost < v)
            {
                let priority = -(new_cost.clone() + heuristic(&neighbor.resulting_node, goal));
                debug!(
                    "\t\tAdding {current:?}->{:?} with priority {priority:?}",
                    neighbor.resulting_node
                );
                cost_so_far.insert(neighbor.resulting_node.clone(), new_cost);
                frontier.push(neighbor.resulting_node.clone(), priority);
                came_from.insert(
                    neighbor.resulting_node,
                    Some((current.clone(), neighbor.edge.clone())),
                );
            }
        }
    }
    // Exited early
    Err(anyhow::anyhow!(
        "No path found within {} steps - reached {} nodes",
        max_nodes_search,
        cost_so_far.len()
    ))
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{EdgeToNodeWithCost, astar};

    #[test]
    fn simple_counting_graph() {
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
            .max_nodes_searched(1_000_000)
            .plan_path(&0i32, &goal);
        // Check we found a path
        assert!(result.is_ok_and(|r| r.path[r.path.len() - 1].1 == goal));
    }

    #[test]
    fn simple_counting_graph_partial_build() {
        // Set up the graph
        let builder = astar()
            // We can increment or decrement by 1 or 2
            .get_neighbors(|i: &i32| {
                vec![
                    EdgeToNodeWithCost::new((), i + 1, 1),
                    EdgeToNodeWithCost::new((), i - 1, 1),
                    EdgeToNodeWithCost::new((), i - 2, 1),
                    EdgeToNodeWithCost::new((), i + 2, 1),
                ]
            })
            .is_goal(|n, g| n == g)
            .heuristic(|i, g| (g - i).abs());

        let result_from_0_to_10 = builder.clone().plan_path(&0, &10);
        // Check we found a path
        assert!(result_from_0_to_10.is_ok_and(|r| r.path[r.path.len() - 1].1 == 10));

        // We can reuse the builder to plan another path with the same graph, goal and heuristic
        let result_from_5_to_0 = builder.clone().plan_path(&5, &0);
        // Check we found a path
        assert!(result_from_5_to_0.is_ok_and(|r| r.path[r.path.len() - 1].1 == 0));
    }

    #[test]
    fn will_fail_if_out_of_steps() {
        let goal = 10i32;
        let result = astar()
            // We can increment or decrement by 1 or 2
            .get_neighbors(|i: &i32| {
                vec![
                    EdgeToNodeWithCost::new((), i + 1, 1),
                    EdgeToNodeWithCost::new((), i - 1, 1),
                    EdgeToNodeWithCost::new((), i - 2, 1),
                    EdgeToNodeWithCost::new((), i + 2, 1),
                ]
            })
            .is_goal(|n, g| n == g)
            // Distance heuristic
            .heuristic(|i, g| (g - i).abs())
            .max_nodes_searched(1)
            .plan_path(&0i32, &goal);
        // Check we could not find a path
        println!("{:?}", result);
        assert!(result.is_err_and(|e| e.to_string().starts_with("No path found")));
    }

    #[test]
    fn will_fail_if_no_valid_path() {
        let goal = -5i32;
        let result = astar()
            // We can increment or decrement by 1 or 2
            .get_neighbors(|i: &i32| {
                vec![
                    EdgeToNodeWithCost::new((), i + 1, 1),
                    EdgeToNodeWithCost::new((), i + 2, 1),
                ]
            })
            .is_goal(|n, g| n == g)
            // Distance heuristic
            .heuristic(|i, g| (g - i).abs())
            .max_nodes_searched(100)
            .plan_path(&0i32, &goal);
        // Check we could not find a path
        assert!(result.is_err_and(|e| e.to_string().starts_with("No path found")));
    }

    #[test]
    fn will_time_out() {
        let goal = -5i32;
        let result = astar()
            // We can increment or decrement by 1 or 2
            .get_neighbors(|i: &i32| {
                vec![
                    EdgeToNodeWithCost::new((), i + 1, 1),
                    EdgeToNodeWithCost::new((), i + 2, 1),
                ]
            })
            // Distance heuristic
            .is_goal(|n, g| n == g)
            .heuristic(|i, g| (g - i).abs())
            .max_nodes_searched(1_000_000)
            .max_search_time(Duration::from_nanos(1))
            .plan_path(&0i32, &goal);
        // Check we could not find a path
        assert!(result.is_err_and(|e| e.to_string().eq("Search timed out")));
    }

    #[test]
    fn adjacency_graph_prefer_small_steps() {
        let adj = vec![
            vec![1, 2],       // 0
            vec![0, 2, 3],    // 1
            vec![0, 1, 3, 4], // 2
            vec![1, 2, 4, 5], // 3
            vec![2, 3, 5],    // 4
            vec![3, 4],       // 5
        ];

        fn cost_from_diff(i: usize, j: usize) -> i32 {
            let delta = (j as i64 - (i as i64)).abs();
            delta.pow(2) as i32
        }
        let result = astar()
            // We can increment or decrement by 1 or 2, for a cost of 1
            .get_neighbors(|i: &usize| {
                adj[*i]
                    .iter()
                    .map(|j| EdgeToNodeWithCost::new((), *j, cost_from_diff(*i, *j)))
                    .collect()
            })
            .is_goal(|n, g| n == g)
            .heuristic(|i, g| (g - i) as i32)
            .plan_path(&0usize, &5);
        // Check we found a path
        let result = result.expect("Is ok");
        println!("{result:?}");
        assert!(result.path[result.path.len() - 1].1 == 5);
        // We should have walked through every possible node due to the better cost of small steps
        // This gives a cost of 5
        assert!(result.cost == 5);
    }
    #[test]
    fn adjacency_graph_prefer_big_steps() {
        let adj = vec![
            vec![1, 2],       // 0
            vec![0, 2, 3],    // 1
            vec![0, 1, 3, 4], // 2
            vec![1, 2, 4, 5], // 3
            vec![2, 3, 5],    // 4
            vec![3, 4],       // 5
        ];
        fn cost_from_diff(i: usize, j: usize) -> i32 {
            let delta = (j as i64 - (i as i64)).abs();
            delta.isqrt() as i32
        }
        let result = astar()
            // We can increment or decrement by 1 or 2, for a cost of 1
            .get_neighbors(|i: &usize| {
                adj[*i]
                    .iter()
                    .map(|j| EdgeToNodeWithCost::new((), *j, cost_from_diff(*i, *j)))
                    .collect()
            })
            .is_goal(|n, g| n == g)
            .heuristic(|_, _| 0)
            .plan_path(&0, &5);
        // Check we found a path
        let result = result.expect("Is ok");
        println!("{result:?}");
        assert!(result.path[result.path.len() - 1].1 == 5);
        // We should have walked through as few nodes as possible node due to the better cost of large steps
        // This gives a cost of 3 (due to i32 cost values)
        assert!(result.cost == 3);
    }

    #[test]
    fn adjacency_graph_with_ordered_float_costs() {
        use ordered_float::OrderedFloat;

        let adj = vec![
            vec![1, 2],       // 0
            vec![0, 2, 3],    // 1
            vec![0, 1, 3, 4], // 2
            vec![1, 2, 4, 5], // 3
            vec![2, 3, 5],    // 4
            vec![3, 4],       // 5
        ];

        fn cost_from_diff(i: usize, j: usize) -> OrderedFloat<f32> {
            let delta = (j as i64 - (i as i64)).abs();
            OrderedFloat((delta as f32).sqrt())
        }
        let result = astar()
            .get_neighbors(|i: &usize| {
                adj[*i]
                    .iter()
                    .map(|j| EdgeToNodeWithCost::new((), *j, cost_from_diff(*i, *j)))
                    .collect()
            })
            .is_goal(|n, g| n == g)
            .heuristic(|i, g| OrderedFloat((g - *i) as f32))
            .plan_path(&0, &5);
        let result = result.expect("Is ok");
        println!("{result:?}");
        assert!(result.path[result.path.len() - 1].1 == 5usize);
        // We expect the cost to reflect the float sqrt values, which for this setup
        // should be approximately 3.414 (1 + sqrt(2) + sqrt(2) or similar).
        // We check that the cost is greater than 3.0 (minimal integer paths) but less than 4.0
        assert!((OrderedFloat(3.0) < result.cost) && (result.cost < OrderedFloat(4.0)));
    }
}
