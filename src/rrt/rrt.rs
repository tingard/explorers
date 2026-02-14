use super::node::RRTNode;
use std::ops::Index;

pub struct RRT<N> {
    // Collection of nodes used in the search
    nodes: Vec<RRTNode<N>>,
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

impl<N> RRT<N> {
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
            // What is the nearest node to the candidate?
            let nearest_node_index = self.nearest_node_index(&candidate)?;
            // Generate a new node which is some interpolation between the candidate and the nearest node
            let new_node = (self.step_function)(&self.nodes[nearest_node_index].node, &candidate)?;
            if !(self.is_valid)(&self.nodes[nearest_node_index].node, &new_node)? {
                // This newly proposed node is
                continue;
            }
            let cumulative_cost = self.nodes[nearest_node_index].cumulative_cost
                + (self.cost_function)(&self.nodes[nearest_node_index].node, &new_node)?;
            let path_has_been_found = (self.is_complete)(&new_node)?;
            self.nodes.push(RRTNode {
                node: new_node,
                cumulative_cost,
                parent: Some(nearest_node_index),
            });
            // Are we complete?
            if path_has_been_found {
                // Build vec of nodes to path and return
                let mut current_index = self.nodes.len() - 1;
                let mut out = vec![];
                loop {
                    out.push(current_index);
                    match self.nodes[current_index].parent {
                        Some(parent_index) => current_index = parent_index,
                        None => break,
                    }
                }
                out.reverse();
                return Ok(Some(out));
            }
        }
        Ok(None)
    }

    fn nearest_node_index(&self, node: &N) -> anyhow::Result<usize> {
        if self.nodes.is_empty() {
            return Err(anyhow::anyhow!("No nodes in the tree"));
        }
        let mut min_distance = f32::INFINITY;
        let mut min_index = 0;
        for (i, n) in self.nodes.iter().enumerate() {
            let distance = (self.cost_function)(node, &n.node)?;
            if distance < min_distance {
                min_distance = distance;
                min_index = i;
            }
        }
        Ok(min_index)
    }
}

impl<N> Index<usize> for RRT<N> {
    type Output = N;

    fn index(&self, index: usize) -> &Self::Output {
        &self.nodes[index].node
    }
}

#[cfg(test)]
mod tests {

    use rand::prelude::*;
    use rand_distr::Uniform;

    use super::{RRT, RRTNode};

    #[test]
    fn simple_2d() {
        type Node = (f32, f32);

        let from = (0.0, 0.0);
        let to = (10.0, 0.0);

        // Euclidean distance cost function
        let cost_function =
            |p0: &Node, p1: &Node| Ok(((p1.0 - p0.0).powi(2) + (p1.1 - p0.1).powi(2)).sqrt());
        let is_complete = move |s: &Node| {
            let dist = cost_function(s, &to)?;
            Ok(dist < 1.0)
        };
        let is_valid = |_: &Node, _: &Node| Ok(true);
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
        let mut search = RRT::<Node>::new(
            from,
            is_complete,
            is_valid,
            cost_function,
            sampling_funtion,
            interpolation_function,
        );
        let path = search.sample(10000).expect("Can search");
        let Some(path) = path else {
            println!("Path not found");
            return;
        };
        for index in path.iter() {
            let node = search[*index];
            println!("{:?}", node);
        }

        println!(
            "Path with n={} found after {} nodes searched",
            path.len(),
            search.len()
        );
        let optimal_path_steps = cost_function(&from, &to).unwrap() / step_size;
        println!("Optimal path would have n={}", optimal_path_steps);
    }
}
