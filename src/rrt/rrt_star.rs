use super::node::RRTNode;
use std::{
    collections::{BTreeSet, HashSet},
    ops::Index,
};

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct SortableCost(f32);

impl SortableCost {
    pub fn new(cost: f32) -> anyhow::Result<Self> {
        if cost.is_infinite() || cost.is_nan() {
            return Err(anyhow::anyhow!("Cannot have infinite or NaN cost"));
        }
        Ok(SortableCost(cost))
    }
}

impl Eq for SortableCost {}

impl Ord for SortableCost {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match self.0.partial_cmp(&other.0) {
            Some(ordering) => ordering,
            // The check in `new` must prevent this
            None => panic!("Cannot compare NaN costs"),
        }
    }
}

pub struct RRTStar<N> {
    // Collection of nodes used in the search
    nodes: Vec<RRTNode<N>>,
    end_nodes: HashSet<usize>,
    // Function to determine if the search is complete
    is_complete: Box<dyn Fn(&N) -> anyhow::Result<bool>>,
    // Function to determine if the first node is allowed to be extended
    // to the second (is this a valid new node? Are we allowed to make this
    // connection?)
    is_valid: Box<dyn Fn(&N, &N) -> anyhow::Result<bool>>,
    // Distance function between two nodes
    cost_function: Box<dyn Fn(&N, &N) -> anyhow::Result<f32>>,
    // How to generate a node, given "from"
    sampling_function: Box<dyn Fn(&[RRTNode<N>]) -> anyhow::Result<N>>,
    // How to step from one node towards another?
    step_function: Box<dyn Fn(&N, &N) -> anyhow::Result<N>>,
}

impl<N> RRTStar<N> {
    pub fn new(
        from: N,
        is_complete: impl Fn(&N) -> anyhow::Result<bool> + 'static,
        is_valid: impl Fn(&N, &N) -> anyhow::Result<bool> + 'static,
        cost_function: impl Fn(&N, &N) -> anyhow::Result<f32> + 'static,
        sampling_function: impl Fn(&[RRTNode<N>]) -> anyhow::Result<N> + 'static,
        step_function: impl Fn(&N, &N) -> anyhow::Result<N> + 'static,
    ) -> Self {
        Self {
            nodes: vec![RRTNode {
                node: from,
                cumulative_cost: 0.0,
                parent: None,
            }],
            end_nodes: HashSet::new(),
            is_complete: Box::new(is_complete),
            is_valid: Box::new(is_valid),
            cost_function: Box::new(cost_function),
            sampling_function: Box::new(sampling_function),
            step_function: Box::new(step_function),
        }
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn sample(&mut self, n: usize) -> anyhow::Result<Option<Vec<usize>>> {
        // Expand the nodes vector to accommodate at least n new items, avoids growing the vector multiple
        // times during the search.
        self.nodes.reserve(n);
        for _ in 0..n {
            // Sample from the tail
            let candidate = (self.sampling_function)(&self.nodes)?;
            // What is the nearest node to the candidate? Find the 20 nearest nodes in the graph.
            // Include all parents of the nearest node in the chain
            let neighborhood = self
                .find_neighborhood(&candidate, 20)?
                .into_iter()
                .flat_map(|n| self.get_parent_chain(n))
                .collect::<Vec<_>>();
            let nearest_node_index = neighborhood[0];
            // Generate a new node which is some interpolation between the candidate and the nearest node
            let new_node = (self.step_function)(&self.nodes[nearest_node_index].node, &candidate)?;
            let new_index = self.nodes.len();
            if !(self.is_valid)(&self.nodes[nearest_node_index].node, &new_node)? {
                // This newly proposed node is not a valid child
                continue;
            }
            let cumulative_cost = self.nodes[nearest_node_index].cumulative_cost
                + self.cost_function(&self.nodes[nearest_node_index].node, &new_node)?;

            // Check what the optimal parent is for this node
            let Some(best_parent_index) = self.select_parent(&neighborhood, &new_node)? else {
                // We could not connect this node to the tree
                continue;
            };
            // We might also want to update the parents of existing nodes in the graph
            for node in neighborhood {
                // Don't make a cycle... (could do better here)
                if node == best_parent_index {
                    continue;
                }
                // What would the new cost be if the new node was this node's parent?
                let updated_cost =
                    cumulative_cost + self.cost_function(&new_node, &self.nodes[node].node)?;
                if updated_cost < self.nodes[node].cumulative_cost {
                    self.nodes[node].parent = Some(new_index);
                    self.nodes[node].cumulative_cost = updated_cost;
                }
            }

            let path_has_been_found = (self.is_complete)(&new_node)?;
            self.nodes.push(RRTNode {
                node: new_node,
                cumulative_cost,
                parent: Some(best_parent_index),
            });

            // Are we complete?
            if path_has_been_found {
                // Record that an end node was found
                self.end_nodes.insert(new_index);
            }
        }
        // What was the best end node?
        let mut best_end_node: Option<usize> = None;
        let mut best_path_cost = f32::MAX;
        self.end_nodes.iter().copied().for_each(|n| {
            let this_node_cost = self.nodes[n].cumulative_cost;
            if this_node_cost < best_path_cost {
                best_end_node = Some(n);
                best_path_cost = this_node_cost;
            }
        });
        let Some(best_end_node) = best_end_node else {
            // We didn't find a path - return None to indicate more samples needed
            return Ok(None);
        };
        let mut out = self.get_parent_chain(best_end_node);
        out.reverse();
        Ok(Some(out))
    }

    pub fn find_neighborhood(
        &self,
        node: &N,
        neighborhood_size: usize,
    ) -> anyhow::Result<Vec<usize>> {
        if self.nodes.is_empty() {
            return Err(anyhow::anyhow!("No nodes in the tree"));
        }
        let mut buf = BTreeSet::<(SortableCost, usize)>::new();
        for (i, n) in self.nodes.iter().enumerate() {
            let distance = SortableCost::new((self.cost_function)(node, &n.node)?)?;
            if buf.len() < neighborhood_size {
                buf.insert((distance, i));
            } else {
                match buf.last() {
                    Some(last_entry) if distance < last_entry.0 => {
                        buf.insert((distance, i));
                        _ = buf.pop_last();
                    }
                    _ => {}
                }
            }
        }
        Ok(buf.into_iter().map(|(_, i)| i).collect())
    }

    fn get_parent_chain(&self, node: usize) -> Vec<usize> {
        let mut chain = vec![node];
        let mut current = node;
        while let Some(parent) = self.nodes[current].parent {
            chain.push(parent);
            current = parent;
        }
        chain
    }

    fn select_parent(
        &mut self,
        neighborhood: &[usize],
        new_node: &N,
    ) -> anyhow::Result<Option<usize>> {
        // Check if any nearby nodes should update their parent to be this node
        let mut best_parent_cost: f32 = f32::MAX;
        let mut best_parent_index: Option<usize> = None;
        for neighbor_node_index in neighborhood {
            let candidate_node = &self.nodes[*neighbor_node_index];
            // Is this candidate a valid connection?
            let is_valid_edge = (self.is_valid)(&candidate_node.node, new_node)?;
            if !is_valid_edge {
                continue;
            }
            let candidate_cost = candidate_node.cumulative_cost
                + self.cost_function(&new_node, &candidate_node.node)?;
            if candidate_cost < best_parent_cost {
                best_parent_cost = candidate_cost;
                best_parent_index = Some(*neighbor_node_index);
            }
        }
        Ok(best_parent_index)
    }

    pub fn cost_function(&self, from: &N, to: &N) -> anyhow::Result<f32> {
        (self.cost_function)(from, to)
    }
    pub fn is_valid(&self, from: &N, to: &N) -> anyhow::Result<bool> {
        (self.is_valid)(from, to)
    }
}

impl<N> Index<usize> for RRTStar<N> {
    type Output = N;

    fn index(&self, index: usize) -> &Self::Output {
        &self.nodes[index].node
    }
}

#[cfg(test)]
mod tests {

    use std::f32;

    use rand::prelude::*;
    use rand_distr::Uniform;

    use super::{RRTNode, RRTStar};

    #[test]
    fn simple_2d() {
        type Node = (f32, f32);

        let from = (0.0, 0.0);
        let to = (10.0, 0.0);
        println!("Pathing from {:?}", from);
        println!("Pathing to {:?}", to);

        // Euclidean distance cost function
        let cost_function =
            |p0: &Node, p1: &Node| Ok(((p1.0 - p0.0).powi(2) + (p1.1 - p0.1).powi(2)).sqrt());
        let is_complete = move |s: &Node| {
            let dist = cost_function(s, &to)?;
            Ok(dist < 1.0)
        };
        // Scatter some blobs on the map - note that no blob may intersect the start or end points
        let circles = vec![(5.0, 0.0, 2.0)];
        println!("Circles: {:?}", circles);
        let is_valid = move |_: &Node, b: &Node| {
            // b must be outside of all circles
            // don't test for line intersection yet (totally doable I'm just lazy)
            Ok(circles.iter().all(|&(cx, cy, r)| {
                let dx = cx - b.0;
                let dy = cy - b.1;
                dx.powi(2) + dy.powi(2) > r * r
            }))
        };
        let sampling_funtion = |nodes: &[RRTNode<Node>]| {
            let p0 = &nodes[0].node;
            let mut rng = rand::rng();
            let x_distr = Uniform::new(-20.0, 20.0).expect("Can create x distr");
            let y_distr = Uniform::new(-20.0, 20.0).expect("Can create y distr");
            let dx: f32 = rng.sample(x_distr);
            let dy: f32 = rng.sample(y_distr);
            Ok((p0.0 + dx, p0.1 + dy))
        };
        let step_size = 0.2;
        let interpolation_function = move |p0: &Node, p1: &Node| {
            let direction = (p1.0 - p0.0, p1.1 - p0.1);
            // scale to desired norm;
            let norm = (direction.0.powi(2) + direction.1.powi(2)).sqrt();
            let s = step_size / norm;
            Ok((p0.0 + s * direction.0, p0.1 + s * direction.1))
        };
        let mut search = RRTStar::<Node>::new(
            from,
            is_complete,
            is_valid,
            cost_function,
            sampling_funtion,
            interpolation_function,
        );
        let mut path = None;
        let mut best_cost = f32::INFINITY;
        for _ in 0..10000 {
            path = search.sample(10).expect("Can search");
            if let Some(path) = &path {
                let cost = path
                    .iter()
                    .zip(path.iter().skip(1))
                    .map(|(p0, p1)| cost_function(&search[*p0], &search[*p1]).unwrap())
                    .sum();
                if cost < best_cost {
                    best_cost = cost;
                    let path_vec: Vec<_> = path.iter().map(|index| search[*index]).collect();
                    println!("{path_vec:?}");
                }
            }
        }
        let Some(path) = path else {
            println!("Path not found");
            return;
        };
        // for index in path.iter() {
        //     let node = search[*index];
        //     println!("{:?}", node);
        // }

        println!(
            "Path with n={} found after {} nodes searched",
            path.len(),
            search.len()
        );
        let optimal_path_steps = cost_function(&from, &to).unwrap() / step_size;
        println!("Optimal path would have n={}", optimal_path_steps);
    }
}
