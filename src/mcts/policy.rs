use crate::mcts::MCTSStore;

use super::environment::Environment;
use super::state::State;
use anyhow::Context;
use rand::prelude::*;

pub trait ActionPolicy<E: Environment> {
    fn select_action<'a>(
        &self,
        state: &<E as Environment>::State,
        actions: &'a [<E as Environment>::Action],
    ) -> Result<&'a <E as Environment>::Action, anyhow::Error>;
}

/// Policy which will randomly sample from a distribution over allowed actions weighted by a state.
pub struct StochasticPolicy<E: Environment> {
    action_weights_fn: Box<
        dyn Fn(
            &<E as Environment>::State,
            &[<E as Environment>::Action],
        ) -> Result<Vec<f32>, anyhow::Error>,
    >,
}

impl<E> StochasticPolicy<E>
where
    E: Environment,
{
    // Policy where actions are provided with weights
    pub fn weighted<F>(action_weights_fn: F) -> Self
    where
        F: Fn(
                &<E as Environment>::State,
                &[<E as Environment>::Action],
            ) -> anyhow::Result<Vec<f32>>
            + 'static,
    {
        StochasticPolicy {
            action_weights_fn: Box::new(action_weights_fn),
        }
    }
    // Policy where actions are uniformly weighted
    pub fn uniform() -> Self {
        StochasticPolicy {
            action_weights_fn: Box::new(|_, a| Ok(a.into_iter().map(|_| 1.0).collect())),
        }
    }
}

impl<E: Environment> ActionPolicy<E> for StochasticPolicy<E> {
    fn select_action<'a>(
        &self,
        state: &<E as Environment>::State,
        actions: &'a [<E as Environment>::Action],
    ) -> Result<&'a <E as Environment>::Action, anyhow::Error> {
        let weights = (self.action_weights_fn)(state, actions)?;
        if actions.is_empty() {
            return Err(anyhow::anyhow!("No actions available"));
        }
        if actions.len() != weights.len() {
            return Err(anyhow::anyhow!(
                "Expected a weight for each action - got {} actions and {} weights",
                actions.len(),
                weights.len()
            ));
        }

        // rand::rng replaces the outdated thread_rng method
        let mut rng = rand::rng();
        let dist = rand::distr::weighted::WeightedIndex::new(&weights)
            .context("Expected a valid weight vector")?;

        let index = dist.sample(&mut rng);
        Ok(&actions[index])
    }
}

pub struct MostVisitedNodePolicy<'a, E>
where
    E: Environment,
{
    env: &'a E,
    mcts_history: &'a MCTSStore<E>,
}

impl<'a, E> MostVisitedNodePolicy<'a, E>
where
    E: Environment,
{
    pub fn new(env: &'a E, mcts_history: &'a MCTSStore<E>) -> Self {
        MostVisitedNodePolicy { env, mcts_history }
    }
}

impl<'a, E> ActionPolicy<E> for MostVisitedNodePolicy<'a, E>
where
    E: Environment,
{
    fn select_action<'b>(
        &self,
        state: &<E as Environment>::State,
        actions: &'b [<E as Environment>::Action],
    ) -> Result<&'b <E as Environment>::Action, anyhow::Error> {
        Ok(actions
            .iter()
            .max_by_key(|a| {
                // TODO: Return an error from the function if this fails
                let transition = self.env.step(state, a).expect("Can step");

                // Unseen nodes are treated as having zero visits.
                self.mcts_history
                    .get_node(&transition.new_state.key())
                    .map(|n| n.playouts)
                    .unwrap_or(0)
            })
            .expect("Can pick action"))
    }
}

#[cfg(test)]
mod tests {
    use crate::mcts::{Action, EnvironmentTransition, TransitionType};

    use super::*;

    #[derive(Clone, Copy, Eq, PartialEq, Debug)]
    enum MyAction {
        A,
        B,
    }

    impl Action for MyAction {
        type Key = [u8; 1];
        fn key(&self) -> Self::Key {
            match self {
                Self::A => [0],
                Self::B => [1],
            }
        }
    }

    #[derive(Clone)]
    struct MyState(u8);

    impl State for MyState {
        type Key = [u8; 1];

        fn key(&self) -> Self::Key {
            [self.0]
        }

        fn is_terminal(&self) -> bool {
            false
        }
    }

    struct MyEnv;
    impl Environment for MyEnv {
        type State = MyState;
        type Action = MyAction;

        fn step(
            &self,
            state: &Self::State,
            action: &Self::Action,
        ) -> Result<EnvironmentTransition<Self::State>, anyhow::Error> {
            match action {
                MyAction::A => Ok(EnvironmentTransition {
                    new_state: MyState(state.0.saturating_add(1)),
                    reward: 0.0,
                    transition_type: TransitionType::Running,
                }),
                MyAction::B => Ok(EnvironmentTransition {
                    new_state: MyState(state.0.saturating_sub(1)),
                    reward: 0.0,
                    transition_type: TransitionType::Running,
                }),
            }
        }

        fn action_space(&self, _: &Self::State) -> Vec<Self::Action> {
            vec![MyAction::A, MyAction::B]
        }
    }

    #[test]
    fn can_draw_actions_from_uniform_policy() {
        let policy = StochasticPolicy::<MyEnv>::uniform();
        let n = 10000;
        let actions = (0..n)
            .map(|_| {
                policy
                    .select_action(&MyState(0), &[MyAction::A, MyAction::B])
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let mut count_a = 0;
        let mut count_b = 0;
        actions.into_iter().for_each(|a| match a {
            MyAction::A => count_a += 1,
            MyAction::B => count_b += 1,
        });
        // Check that the counts are approximately equal - TODO: what is an appropritate
        // bound here to avoid randomly failing tests?
        assert!(((count_a as f32 - count_b as f32) / n as f32).abs() < 0.1)
    }

    #[test]
    fn can_draw_actions_from_weighted_policy_uniform() {
        let policy = StochasticPolicy::<MyEnv>::weighted(|_, a| {
            Ok(a.iter()
                .map(|a| match a {
                    MyAction::A => 1.0,
                    MyAction::B => 1.0,
                })
                .collect())
        });
        let n = 10000;
        let actions = (0..n)
            .map(|_| {
                policy
                    .select_action(&MyState(0), &[MyAction::A, MyAction::B])
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let mut count_a = 0;
        let mut count_b = 0;
        actions.into_iter().for_each(|a| match a {
            MyAction::A => count_a += 1,
            MyAction::B => count_b += 1,
        });
        // Check that the counts are approximately equal - TODO: what is an appropritate
        // bound here to avoid randomly failing tests?
        assert!(((count_a as f32 - count_b as f32) / (n as f32)).abs() < 0.1)
    }

    #[test]
    fn can_draw_actions_from_weighted_policy_only_a() {
        let policy = StochasticPolicy::<MyEnv>::weighted(|_, a| {
            Ok(a.iter()
                .map(|a| match a {
                    MyAction::A => 1.0,
                    MyAction::B => 0.0,
                })
                .collect())
        });
        let n = 10_000;
        let actions = (0..n)
            .map(|_| {
                policy
                    .select_action(&MyState(0), &[MyAction::A, MyAction::B])
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let mut count_a = 0;
        let mut count_b = 0;
        actions.into_iter().for_each(|a| match a {
            MyAction::A => count_a += 1,
            MyAction::B => count_b += 1,
        });
        // Check that we only generated A action
        assert_eq!(count_a, n);
        assert_eq!(count_b, 0);
    }

    #[test]
    fn can_draw_actions_from_weighted_policy_only_b() {
        let policy = StochasticPolicy::<MyEnv>::weighted(|_, a| {
            Ok(a.iter()
                .map(|a| match a {
                    MyAction::A => 0.0,
                    MyAction::B => 1.0,
                })
                .collect())
        });
        let n = 10_000;
        let actions = (0..n)
            .map(|_| {
                policy
                    .select_action(&MyState(0), &[MyAction::A, MyAction::B])
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let mut count_a = 0;
        let mut count_b = 0;
        actions.into_iter().for_each(|a| match a {
            MyAction::A => count_a += 1,
            MyAction::B => count_b += 1,
        });
        // Check that we only generated A action
        assert_eq!(count_a, 0);
        assert_eq!(count_b, n);
    }
}
