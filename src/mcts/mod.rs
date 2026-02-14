mod action;
mod environment;
pub mod policy;
mod state;
pub mod uct;

use std::collections::{HashMap, HashSet};

use anyhow::Context;

pub use self::action::Action;
pub use self::environment::{Environment, EnvironmentTransition, TransitionType};
use self::policy::ActionPolicy;
pub use self::state::State;

#[derive(Clone)]
pub struct MCTSNode<E: Environment> {
    // TODO: f64 feature flag
    pub playouts: u32,
    pub value_sum: f32,
    /// Child state keys
    /// TODO: Action information?
    pub simulated_actions: HashSet<<<E as Environment>::Action as Action>::Key>,
    //
    // @CR: suggestion (non-blocking): The nested associated type `HashSet<<<E as Environment>::Action as Action>::Key>`
    // is quite verbose because of the trait bounds. Consider introducing a type alias in this
    // module (e.g. `type ActionKey<E> = <E as Environment>::Action::Key;`) to improve readability
    // because long types reduce clarity when scanning the code.
}

impl<E: Environment> Default for MCTSNode<E> {
    fn default() -> Self {
        Self {
            playouts: 0,
            value_sum: 0.0,
            simulated_actions: HashSet::default(),
        }
    }
}

#[derive(Default, Clone)]
pub struct MCTSStore<E: Environment> {
    /// Mapping from state ID to node information
    pub store: HashMap<<<E as Environment>::State as State>::Key, MCTSNode<E>>,
}

impl<E> MCTSStore<E>
where
    E: Environment,
{
    pub fn new(root: <<E as Environment>::State as State>::Key) -> Self {
        let mut store = HashMap::new();
        store.insert(root, MCTSNode::default());
        MCTSStore { store }
    }

    pub fn get_node(
        &self,
        key: &<<E as Environment>::State as State>::Key,
    ) -> Option<&MCTSNode<E>> {
        self.store.get(key)
    }

    pub fn find_leaf(
        &self,
        env: &E,
        root: &<E as Environment>::State,
        selection_policy: &dyn ActionPolicy<E>,
    ) -> anyhow::Result<(
        Vec<<E as Environment>::State>,
        Vec<<E as Environment>::Action>,
    )> {
        let mut action_chain = vec![];
        let mut state_chain = vec![root.clone()];

        let mut expected_state = root.clone();

        loop {
            // Is the state a leaf node?
            // This node must already be in the history - if it wasn't we'd have identified its parent as a leaf node.
            let node = self.store.get(&expected_state.key()).context(
                "Node {expected_state:?} not found in history while searching for leaf node",
            )?;
            // What actions can we take from this state?
            let allowed_actions = env.action_space(&expected_state);
            // Have we simulated all of these actions?
            let all_actions_simulated = allowed_actions
                .iter()
                .all(|a| node.simulated_actions.contains(&a.key()));
            if !all_actions_simulated {
                // If not - this is a leaf node!
                break Ok((state_chain, action_chain));
            }
            // Choose the most promising action to explore, given my policy
            let candidate_action = selection_policy
                .select_action(&expected_state, &allowed_actions)
                .context("Could not select action from state {expected_state:?} and allowed actions {allowed_actions:?}")?
                .clone();

            // What does the model suggest will happen if I take this action?
            let transition = env.step(&expected_state, &candidate_action).context(
                "Could not take action {candidate_action:?} from state {expected_state:?}",
            )?;
            // We now expect to be in a new state
            expected_state = transition.new_state;
            // Add this action to the list of actions taken to reach a leaf node
            action_chain.push(candidate_action.clone());
            // Add the resulting state to our state chain
            state_chain.push(expected_state.clone());
            let TransitionType::Running = &transition.transition_type else {
                // If we're now in a terminal state, return the chain leading to it
                break Ok((state_chain, action_chain));
            };
        }
    }

    /// Record that we have expanded the graph from a given action
    pub fn record_simulated_action(
        &mut self,
        state: <<E as Environment>::State as State>::Key,
        action: <<E as Environment>::Action as Action>::Key,
    ) {
        // TODO: Return a result if not found
        //
        // @CR: todo: Consider returning a `Result` when the provided `state` isn't present
        // in `store` because silently inserting a default node (`or_default`) may hide bugs
        // in callers that expect the state to already exist. Returning an error makes misuse
        // easier to detect and reason about.
        let node = self.store.entry(state).or_default();
        node.simulated_actions.insert(action);
    }

    pub fn record_result(&mut self, state: <<E as Environment>::State as State>::Key, reward: f32) {
        let node = self.store.entry(state).or_default();
        node.playouts += 1;
        node.value_sum += reward;

        // @CR: suggestion (non-blocking): If this store will ever be accessed concurrently,
        // consider making `playouts` and `value_sum` updates atomic or protecting the store
        // with a mutex because simultaneous updates can lead to lost increments and subtle
        // bugs. Also consider whether `f64` precision is desirable over `f32` for aggregated
        // rewards because reducing rounding error can matter over many playouts.
    }
}
