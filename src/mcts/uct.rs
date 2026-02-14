use std::f32;

use anyhow::Context;

use crate::mcts::{Action, MCTSStore, State, policy::ActionPolicy};

use super::environment::Environment;

/// The UCT score is designed to balance exploration and exploitation, taking into account the
/// number of wins and simulations for a node, as well as the total number of parent simulations.
///
/// If `playouts` is zero, the function returns `f32::INFINITY` to encourage exploration.
pub fn uct_score(
    value_sum: f32,
    playouts: u32,
    parent_playouts: u32,
    exploration_parameter: f32,
) -> f32 {
    if playouts == 0 {
        return f32::INFINITY;
    }
    (value_sum / (playouts as f32))
        + exploration_parameter * ((parent_playouts as f32).ln() / (playouts as f32)).sqrt()
}

/// UCT policy. Requires access to an MCTS History object, which is used to query number of playouts and mean reward for a
/// state.
pub struct UCTPolicy<'a, E>
where
    E: Environment,
{
    env: &'a E,
    mcts_history: &'a MCTSStore<E>,
}

impl<'a, E> UCTPolicy<'a, E>
where
    E: Environment,
{
    pub fn new(env: &'a E, mcts_history: &'a MCTSStore<E>) -> Self {
        UCTPolicy { env, mcts_history }
    }
}

impl<'a, E, S, A> ActionPolicy<E> for UCTPolicy<'a, E>
where
    E: Environment<State = S, Action = A>,
    S: State + std::fmt::Debug,
    A: Action + std::fmt::Debug,
{
    fn select_action<'b>(&self, state: &S, actions: &'b [A]) -> Result<&'b A, anyhow::Error> {
        if actions.is_empty() {
            return Err(anyhow::anyhow!("No actions available"));
        }
        let parent_node = self
            .mcts_history
            .get_node(&state.key())
            .context("Can get parent node")?;
        // Determine the resulting state of each action
        let best_action = actions
            .iter()
            .map(|a| {
                // TODO: Return an error if we cannot take the action
                let transition = self.env.step(state, a).expect("Can take action");
                let score = self
                    .mcts_history
                    .get_node(&transition.new_state.key())
                    // If the node is not present, it should have a maximal value
                    .map_or(f32::MAX, |node| {
                        uct_score(
                            node.value_sum,
                            node.playouts,
                            parent_node.playouts,
                            2.0f32.sqrt(),
                        )
                    });
                score
            })
            .enumerate()
            .max_by(|(_, v1), (_, v2)| v1.partial_cmp(v2).unwrap())
            .expect("Has maximum value") // We checked actions was not empty earlier
            .0;
        // It's ok to index directly here so long as best_action is selected as an index from actions
        Ok(&actions[best_action])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uct_score() {
        let score_a = uct_score(10.0, 100, 1000, 1.414);
        let score_b = uct_score(10.0, 900, 1000, 1.414);
        let score_c = uct_score(0.0, 0, 1000, 1.414);
        assert!(score_a > score_b && score_c > score_a);
    }
}
