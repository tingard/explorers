use std::{collections::hash_map::Keys, marker::PhantomData};

use hashbrown::HashMap;
use tracing_subscriber::filter::targets::IntoIter;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TickResult {
    Running,
    Success,
    Failure,
}

#[derive(Default)]
pub struct TreeNode<B> {
    tick_fn: Option<Box<dyn FnMut(&mut B) -> TickResult>>,
}

impl<F, B> From<F> for TreeNode<B>
where
    F: FnMut(&mut B) -> TickResult + 'static,
{
    fn from(value: F) -> Self {
        TreeNode {
            tick_fn: Some(Box::new(value)),
        }
    }
}

pub struct BehaviorTreeState<B> {
    _b: PhantomData<B>,
    active_nodes: Vec<NodeId>,
}

pub type NodeId = usize;

pub struct BehaviorTree<B> {
    nodes: Vec<TreeNode<B>>,
    entry_point: Option<usize>,
    graph: HashMap<(NodeId, TickResult), Vec<NodeId>>,
}

impl<B> BehaviorTree<B> {
    pub fn new() -> Self {
        Self {
            nodes: vec![],
            graph: HashMap::new(),
            entry_point: None,
        }
    }
    pub fn tick(
        &self,
        blackboard: &mut B,
        state: BehaviorTreeState<B>,
    ) -> anyhow::Result<BehaviorTreeState<B>> {
        todo!();
    }

    pub fn initial_state(&self) -> anyhow::Result<BehaviorTreeState<B>> {
        let Some(entry_point) = self.entry_point else {
            return Err(anyhow::anyhow!("No entry point specified."));
        };
        Ok(BehaviorTreeState {
            _b: PhantomData,
            active_nodes: vec![entry_point],
        })
    }

    pub fn insert(
        &mut self,
        parent_node: NodeId,
        parent_state: TickResult,
        node: impl Into<TreeNode<B>>,
    ) -> NodeId {
        // If
        let node_id = self.nodes.len();
        self.nodes.push(node.into());
        self.graph
            .entry((parent_node, parent_state))
            .or_default()
            .push(node_id);
        node_id
    }

    fn iter_exit_points<'a>(&'a self) -> impl Iterator<Item = (usize, TickResult)> + 'a {
        (0..self.nodes.len())
            .into_iter()
            .flat_map(|i| {
                [
                    TickResult::Running,
                    TickResult::Success,
                    TickResult::Failure,
                ]
                .iter()
                .copied()
                .map(move |s| (i, s))
            })
            .filter(|k| !self.graph.contains_key(k))
    }
}

macro_rules! sequential {
    ($($x:ident),+) => {
        // {
        //     // let seq_start =
        // };
        println!("Hello!")
    };
}

macro_rules! fallback {
    ($($x:ident),+) => {
        // {
        //     // let seq_start =
        // };
        println!("Hello!")
    };
}

macro_rules! parallel {
    ($($x:ident),+) => {
        // {
        //     // let seq_start =
        // };
        println!("Hello!")
    };
}

#[cfg(test)]
mod tests {
    use crate::bt::TickResult;

    #[test]
    fn single_node_tree() {
        fn do_thing_a(_: ()) -> TickResult {
            TickResult::Success
        }
        fn do_thing_b(_: ()) -> TickResult {
            TickResult::Success
        }
        fn do_thing_c(_: ()) -> TickResult {
            TickResult::Failure
        }
        fn my_fallback(_: ()) -> TickResult {
            TickResult::Success
        }
        fn monitor_status(_: ()) -> TickResult {
            TickResult::Running
        }
        let main_tree = sequential![do_thing_a, do_thing_b, do_thing_c];
        let fallback_tree = fallback![main_tree, my_fallback];
        let tree = parallel![fallback_tree, monitor_status];

        let mut blackboard = ();
        let state = tree.initial_state().expect("Can create initial state");
        let new_state = tree.tick(&mut blackboard, state);

        let mut i: usize = 0;
        let count = move || {
            i = i + 1;
            return i;
        }

    }
}
