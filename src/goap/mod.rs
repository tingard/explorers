mod action;
mod btree_state;
mod goap_state;
mod value;

use bon::bon;
use num_traits::Num;
use std::{fmt::Debug, marker::PhantomData, ops::Neg};
use tracing::debug;

pub use self::action::Action;
pub use self::goap_state::State;
use super::astar;
use crate::astar::EdgeToNodeWithCost;
pub use btree_state::BTreeState;
pub use value::Value;

pub struct GoapPlan<S, A, C>
where
    S: State,
    A: Action<S, Cost = C> + Clone,
{
    pub path: Vec<A>,
    pub cost: C,
    _s: PhantomData<S>,
}

impl<S, A, C> std::fmt::Debug for GoapPlan<S, A, C>
where
    S: State,
    A: Action<S, Cost = C> + Clone + Debug,
    C: Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GoapPlan")
            .field("path", &self.path)
            .field("cost", &self.cost)
            .finish()
    }
}

type ActionFactoryFunction<S, A> = dyn Fn(&S) -> Vec<A>;
type GoalComparatorFunction<S, G> = dyn Fn(&S, &G) -> bool;
type HeuristicFunction<S, G, C> = dyn Fn(&S, &G) -> C;
type EventFactoryFunction<S> = dyn Fn(&S, &mut S);

/// GOAP planner which uses A* search to find a path to a goal state.
pub struct Planner<S, A, G, C>
where
    S: State,
    A: Action<S> + Clone,
{
    action_factory: Box<ActionFactoryFunction<S, A>>,
    goal_comparator: Box<GoalComparatorFunction<S, G>>,
    heuristic: Box<HeuristicFunction<S, G, C>>,
    event_factory: Box<EventFactoryFunction<S>>,
}

#[bon]
impl<S, A, G, C> Planner<S, A, G, C>
where
    S: State,
    A: Action<S, Cost = C> + Clone,
    C: Num + Ord + Neg<Output = C> + Clone + Debug,
{
    /// Create a new planner.
    ///
    /// The planner requires a number of functions to
    pub fn new(
        action_factory: impl Fn(&S) -> Vec<A> + 'static,
        is_goal: impl Fn(&S, &G) -> bool + 'static,
        heuristic: impl Fn(&S, &G) -> C + 'static,
        event_factory: impl Fn(&S, &mut S) + 'static,
    ) -> Self {
        Self {
            action_factory: Box::new(action_factory),
            goal_comparator: Box::new(is_goal),
            heuristic: Box::new(heuristic),
            event_factory: Box::new(event_factory),
        }
    }

    #[builder(finish_fn = plan)]
    pub fn plan_for_goal(
        &self,
        current_state: &S,
        goal_state: &G,
        max_nodes_searched: Option<usize>,
    ) -> anyhow::Result<GoapPlan<S, A, C>> {
        let result = astar::astar()
            .get_neighbors(|state: &S| {
                debug!("Checking available actions for state: {:?}", state);
                let actions = (self.action_factory)(state);
                debug!("All available actions: {:?}", actions);
                actions
                    .into_iter()
                    .map(|action| {
                        debug!("Running action {:?} on state {:?}", action, state);
                        let mut new_state = state.clone();
                        let action_cost = action
                            .forward(&mut new_state)
                            .expect("Can run action on state");
                        // This action might have triggered events, which will mutate the resulting state.
                        // For example, a "Wait" action would update some time information in the state, which
                        // we could use to simulate motion of objects via effects.
                        debug!("Running events on resulting state {:?}", state);
                        (self.event_factory)(state, &mut new_state);
                        debug!("Action {:?} resulted in state {:?}", action, new_state);
                        EdgeToNodeWithCost::new(action, new_state, action_cost)
                    })
                    .collect()
            })
            .maybe_max_nodes_searched(max_nodes_searched)
            .is_goal(&self.goal_comparator)
            .heuristic(&self.heuristic)
            .plan_path(current_state, goal_state)?;
        Ok(GoapPlan {
            path: result
                .path
                .into_iter()
                .filter_map(|(action, _)| action)
                .collect(),
            cost: result.cost,
            _s: PhantomData,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum EatingActions {
        Buy(String),
        Cook(String),
        Eat(String),
    }
    impl Action<BTreeState> for EatingActions {
        type Cost = i32;
        fn forward(&self, state: &mut BTreeState) -> anyhow::Result<i32> {
            match self {
                Self::Buy(f) => {
                    state.insert("has_money".to_string(), false.into());
                    state.insert(format!("has_food<{}>", f), true.into());
                    Ok(2)
                }
                Self::Cook(f) => {
                    state.insert(format!("has_ingredients<{}>", f), false.into());
                    state.insert(format!("has_food<{}>", f), true.into());
                    Ok(1)
                }
                Self::Eat(f) => {
                    state.insert(format!("has_food<{}>", f), false.into());
                    state.insert("hungry".to_string(), false.into());
                    Ok(0)
                }
            }
        }
    }

    /// What actions can I take in a given state?
    fn pizza_action_factory(state: &BTreeState) -> Vec<EatingActions> {
        let mut actions: Vec<EatingActions> = vec![];
        if state.get("has_money") == Some(&Value::Bool(true)) {
            actions.push(EatingActions::Buy("pizza".to_string()));
        }
        for (id, v) in state.items_with_prefix("has_ingredients<") {
            if matches!(v, Value::Bool(true)) {
                let food = id
                    .strip_prefix("has_ingredients<")
                    .expect("Has prefix")
                    .split('>')
                    .next()
                    .expect("Has suffix");
                actions.push(EatingActions::Cook(food.to_string()));
            }
        }
        for (id, v) in state.items_with_prefix("has_food<") {
            if matches!(v, Value::Bool(true)) {
                let food = id
                    .strip_prefix("has_food<")
                    .expect("Has prefix")
                    .split('>')
                    .next()
                    .expect("Has suffix");
                actions.push(EatingActions::Eat(food.to_string()));
            }
        }
        if state.get("has_food<pizza>") == Some(&Value::Bool(true)) {
            actions.push(EatingActions::Eat("pizza".to_string()));
        }
        actions
    }

    #[test]
    fn test_planner_creation() {
        // Set up a builder which can be called with a current state to generate a plan.
        let planner = Planner::new(
            // We need to specify what actions are available in a given state
            |_| vec![EatingActions::Eat("pizza".to_string())],
            // We need to specify whether a goal state has been reached
            |state, goal| state.matches(goal),
            // We can provide a heuristic function to guide search
            |state, goal| goal.manhattan_distance(state) as i32,
            // We also need to allow side-effects to mutate the state, this is handled by an event factory
            |_, _| (),
        );

        // If we have pizz and we are hungry, we can eat pizza
        let current_state = BTreeState::default()
            .with("has_food<pizza>", true.into())
            .with("hungry", true.into());

        let goal_state = BTreeState::default().with("hungry", false.into());

        // Given a current state and a goal state, determine a sequence of actions to reach the goal state
        let actions = planner
            .plan_for_goal()
            .current_state(&current_state)
            .goal_state(&goal_state)
            .plan()
            .expect("Can make plan");

        assert_eq!(actions.path.len(), 1);
        assert_eq!(actions.path[0], EatingActions::Eat("pizza".to_string()));
    }

    #[test]
    fn test_multi_state_plan_creation() {
        // Set up a builder which can be called with a current state to generate a plan.
        let planner = Planner::new(
            // We need to specify what actions are available in a given state
            pizza_action_factory,
            // We need to specify whether a goal state has been reached
            |state, goal| state.matches(goal),
            // We can provide a heuristic function to guide search
            |state, goal| goal.manhattan_distance(state) as i32,
            // We also need to allow side-effects to mutate the state, this is handled by an event factory
            |_, _| (),
        );
        // If we have money and we are hungry, we can buy pizza to eat, then eat it
        let current_state = BTreeState::default()
            .with("has_money", true.into())
            .with("hungry", true.into());

        let goal_state = BTreeState::default().with("hungry", false.into());

        // Given a current state and a goal state, determine a sequence of actions to reach the goal state
        let actions = planner
            .plan_for_goal()
            .current_state(&current_state)
            .goal_state(&goal_state)
            .plan()
            .expect("Can plan actions");

        assert_eq!(actions.path.len(), 2);
        assert_eq!(actions.path[0], EatingActions::Buy("pizza".to_string()));
        assert_eq!(actions.path[1], EatingActions::Eat("pizza".to_string()));
    }

    #[test]
    fn test_multi_state_plan_creation_with_cost() {
        // Set up a builder which can be called with a current state to generate a plan.
        let planner = Planner::new(
            // We need to specify what actions are available in a given state
            pizza_action_factory,
            // We need to specify whether a goal state has been reached
            |state, goal| state.matches(goal),
            // We can provide a heuristic function to guide search
            |state, goal| goal.manhattan_distance(state) as i32,
            // We also need to allow side-effects to mutate the state, this is handled by an event factory
            |_, _| (),
        );

        // If we have money AND ingredients for pizza and we are hungry, we can choose whether to
        // buy or make pizza to eat, then eat it.
        // We should prefer to make pizza than buy it as it is cheaper.
        let current_state = BTreeState::default()
            .with("has_money", true.into())
            .with("has_ingredients<pizza>", true.into());

        let goal_state = BTreeState::default().with("hungry", false.into());

        // Given a current state and a goal state, determine a sequence of actions to reach the goal state
        let actions = planner
            .plan_for_goal()
            .current_state(&current_state)
            .goal_state(&goal_state)
            .plan()
            .expect("Can plan actions");

        assert_eq!(actions.path.len(), 2);
        assert_eq!(actions.path[0], EatingActions::Cook("pizza".to_string()));
        assert_eq!(actions.path[1], EatingActions::Eat("pizza".to_string()));
    }

    /// This test script demonstrates leveraging the events factory to react to changes in state caused by actions.
    /// In this case, the "wait" action moves the clock forwards by one second, which is picked up on by the events factory and used to
    /// move the obstacle forward by one unit.
    ///
    /// We show that the planner is able to move the player out of the way of the oncoming obstacle, preferring the minimum number of moves.
    #[test]
    fn test_multi_state_plan_creation_with_time_and_events() {
        #[derive(Debug, Clone, PartialEq, Eq)]
        enum KeyboardAction {
            Wait1,
            Left,
            Right,
        }
        impl Action<BTreeState> for KeyboardAction {
            type Cost = i32;

            fn forward(&self, state: &mut BTreeState) -> anyhow::Result<Self::Cost> {
                match self {
                    KeyboardAction::Left => state.update("position", |v| {
                        let Value::I32(pos) = v else {
                            panic!("Unexpected position type");
                        };
                        *pos -= 1;
                        None
                    }),
                    KeyboardAction::Right => state.update("position", |v| {
                        let Value::I32(pos) = v else {
                            panic!("Unexpected position type");
                        };
                        *pos += 1;
                        None
                    }),
                    KeyboardAction::Wait1 => state.update("timer", |v| {
                        let Value::U32(time) = v else {
                            panic!("Unexpected time type");
                        };
                        *time += 1;
                        None
                    }),
                };
                Ok(1)
            }
        }
        fn action_factory(_: &BTreeState) -> Vec<KeyboardAction> {
            vec![
                KeyboardAction::Left,
                KeyboardAction::Right,
                KeyboardAction::Wait1,
            ]
        }

        fn event_factory(state_before_action: &BTreeState, state_after_action: &mut BTreeState) {
            let Some(Value::U32(time_before)) = state_before_action.get("timer") else {
                eprintln!("No timer found");
                return;
            };
            let Some(Value::U32(time_after)) = state_after_action.get("timer") else {
                eprintln!("No timer found");
                return;
            };
            let time_delta = time_after - time_before;
            if time_delta == 0 {
                return;
            }
            // Move obstacle down
            let Some(Value::U32(resulting_obstacle_y)) =
                state_after_action.update("obstacle_y", |obstacle_y| {
                    let Value::U32(y) = obstacle_y else {
                        return None;
                    };
                    *y = y.saturating_sub(time_delta);
                    Some(Value::U32(*y))
                })
            else {
                eprintln!("Failed to update obstacle_y");
                return;
            };
            if resulting_obstacle_y > 0 {
                return;
            }
            let Some(Value::I32(obstacle_x)) = state_after_action.get("obstacle_x") else {
                eprintln!("Failed to get obstacle_x");
                return;
            };
            // Check for collision with player
            let Some(Value::I32(pos)) = state_after_action.get("position") else {
                eprintln!("Failed to get player position");
                return;
            };
            if (pos - obstacle_x).abs() < 2 {
                state_after_action.insert("collision", true.into());
            }
        }

        // Set up a builder which can be called with a current state to generate a plan.
        let planner = Planner::new(
            // We need to specify what actions are available in a given state
            action_factory,
            // We need to specify whether a goal state has been reached
            |state, goal| state.matches(goal),
            // We can provide a heuristic function to guide search
            |state, goal| goal.manhattan_distance(state) as i32,
            // We also need to allow side-effects to mutate the state, this is handled by an event factory
            event_factory,
        );

        // If we have money AND ingredients for pizza and we are hungry, we can choose whether to
        // buy or make pizza to eat, then eat it.
        // We should prefer to make pizza than buy it as it is cheaper.
        let mut current_state = BTreeState::default()
            .with("timer", 0u32.into())
            .with("collision", false.into())
            .with("position", 0i32.into())
            // By putting the obstacle off-center, we test the planner's ability to prefer the shorter path (by moving left)
            .with("obstacle_x", 1i32.into())
            .with("obstacle_y", 10u32.into());

        // Goal is the obstacle at y=0 WITHOUT a collision
        let goal_state = BTreeState::default()
            .with("collision", false.into())
            .with("obstacle_y", 0u32.into());

        // Given a current state and a goal state, determine a sequence of actions to reach the goal state
        let actions = planner
            .plan_for_goal()
            .current_state(&current_state)
            .goal_state(&goal_state)
            .plan()
            .expect("Can plan actions");

        // Replay the scanerio, including events, to verify the correctness of the plan
        for action in actions.path.iter() {
            let state_before_action = current_state.clone();
            action
                .forward(&mut current_state)
                .expect("Can apply action to state");
            event_factory(&state_before_action, &mut current_state);
        }

        assert!(current_state.get("collision").unwrap() == &Value::Bool(false));
        assert!(current_state.get("obstacle_y").unwrap() == &Value::U32(0));
        assert!(current_state.get("position").unwrap() == &Value::I32(-1));
    }
}
