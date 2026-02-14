use rerun::{demo_util::grid, external::glam};
use std::hash::{DefaultHasher, Hash, Hasher};
use std::time::{Duration, Instant};

use anyhow::Context;
use explorers::mcts::{
    policy::{ActionPolicy, StochasticPolicy},
    *,
};
use rand::rngs::ThreadRng;

/// Utility function to wait for user input (anything ending in a carriage return)
fn wait_for_input() {
    let mut answer = String::new();
    std::io::stdin()
        .read_line(&mut answer)
        .ok()
        .expect("Failed to read line");
}

#[derive(Default, Clone, Debug, PartialEq, Eq)]
struct PositionVelocity {
    x: i32,
    y: i32,
    vx: i32,
    vy: i32,
}

#[derive(Default, Clone, Debug, PartialEq, Eq)]
struct SnowballDefenceState {
    pub visible_snowballs: Vec<PositionVelocity>,
    pub orientation_history: Vec<u8>,
}

impl State for SnowballDefenceState {
    type Key = [u8; 8];

    fn key(&self) -> [u8; 8] {
        let mut hasher = DefaultHasher::new();
        // Where have I been looking?
        self.orientation_history.hash(&mut hasher);
        // What snowballs are currently visible?
        // ...

        let hash = hasher.finish();
        hash.to_be_bytes()
    }

    fn is_terminal(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SnowballDefenceAction {
    RotateLeft,
    RotateRight,
}

impl Action for SnowballDefenceAction {
    type Key = [u8; 1];

    fn key(&self) -> [u8; 1] {
        match self {
            SnowballDefenceAction::RotateLeft => [0],
            SnowballDefenceAction::RotateRight => [1],
        }
    }
}

struct SnowballDefenceEnvironment {
    rng: ThreadRng,
}

impl SnowballDefenceEnvironment {
    pub fn new() -> Self {
        let rng = rand::rng();
        Self { rng }
    }
}

impl Environment for SnowballDefenceEnvironment {
    type Action = SnowballDefenceAction;
    type State = SnowballDefenceState;
    fn action_space(&self, _state: &Self::State) -> Vec<Self::Action> {
        vec![
            SnowballDefenceAction::RotateLeft,
            SnowballDefenceAction::RotateRight,
        ]
    }
    fn step(
        &self,
        state: &Self::State,
        action: &Self::Action,
    ) -> Result<EnvironmentTransition<Self::State>, anyhow::Error> {
        // Did we get hit by any snowballs?
        todo!();

        // Do we pop any snowballs? - compute angle between direction turret
        // is facing and snowball relative position
        todo!();

        // Do we spawn any snowballs?
        todo!();

        // Return the Running transition
        return Ok(EnvironmentTransition {
            transition_type: TransitionType::Running,
            new_state: state.clone(),
            reward: 0.0,
        });
        todo!();
    }
}

fn rollout(
    env: &SnowballDefenceEnvironment,
    root: &SnowballDefenceState,
    policy: &impl ActionPolicy<SnowballDefenceEnvironment>,
) -> Result<(SnowballDefenceState, f32), anyhow::Error> {
    let mut current_state = root.clone();
    loop {
        // What actions _could_ we take?
        let available_actions = env.action_space(&current_state);
        let chosen_action = policy
            .select_action(&current_state, &available_actions)
            .context("Could not select action")?;
        let transition = env.step(&current_state, chosen_action).context(
            "Could not step environment from state\n{current_state}\nwith action {chosen_action}",
        )?;
        current_state = transition.new_state.clone();
        if transition.transition_type != TransitionType::Running {
            break;
        }
    }
    Ok((current_state, 0.0))
}

/// Backpropagate results from a rollout to the mcts history. We must ensure that appropriate rewards are assigned to each
/// state dependent on the actor and the eventual winner.
fn backpropagate<'a>(
    mcts_history: &mut MCTSStore<SnowballDefenceEnvironment>,
    states: impl Iterator<Item = &'a SnowballDefenceState>,
    total_reward: f32,
) {
    // TODO: discount factor?
    for state in states {
        mcts_history.record_result(state.key(), total_reward);
    }
}

fn run_mcts() -> anyhow::Result<()> {
    let rec = rerun::RecordingStreamBuilder::new("rerun_example_minimal").spawn()?;
    // Initialize our environment
    let env = SnowballDefenceEnvironment::new();

    // And a starting state
    let mut state = SnowballDefenceState::default();

    // Initialize MCTS store
    let mut mcts_history = MCTSStore::new(state.key());

    // Initialize a simulation policy - this will be uniformly random over possible actions (for now)
    let simulation_policy: StochasticPolicy<SnowballDefenceEnvironment> =
        StochasticPolicy::uniform();

    // Gameplay loop
    loop {
        println!("State is\n{state:?}");
        // Define a computational budget which we must not exceed
        let run_till = Instant::now() + Duration::from_millis(100);
        while Instant::now() < run_till {
            // Selection
            // Given our available actions at the current node, choose a leaf node.
            // A leaf node is any node that has a potential child from which no simulation
            // has yet been initiated.
            // We need to re-define this policy each loop as mcts_history is mutated
            let selection_policy = explorers::mcts::uct::UCTPolicy::new(&env, &mcts_history);
            let (states_to_leaf, _actions_to_leaf) = mcts_history
                .find_leaf(&env, &state, &selection_policy)
                .expect("Can get leaf node");
            let leaf_node = states_to_leaf.last().expect("States to leaf is not empty");

            // Expansion
            // What actions are available for expansion?
            let expansion_actions = env.action_space(&leaf_node);
            let expansion_action = selection_policy
                .select_action(&leaf_node, &expansion_actions)
                .expect("Can select expansion action");

            let expanded_transition = env.step(&leaf_node, expansion_action).expect("Can step");
            let expanded_state = &expanded_transition.new_state;
            // Each rollout env must have a different rng state
            let rollout_env = SnowballDefenceEnvironment::new();
            let (_rollout_end_state, reward) =
                rollout(&rollout_env, expanded_state, &simulation_policy)?;
            // println!("Rollout winner is {rollout_winner:?}");
            // Record that we ran a simulation from the expanded node + action
            mcts_history.record_simulated_action(leaf_node.key(), expansion_action.key());

            // // Add expanded state to the history - this will not overwrite existing nodes
            // mcts_history.insert_node(expanded_state.key());

            // Update all states up to the expanded node
            backpropagate(
                &mut mcts_history,
                states_to_leaf.iter().chain([expanded_state.clone()].iter()),
                reward,
            );
        }

        // We will play the action which has the highest number of visits during expansion
        let selection_policy =
            explorers::mcts::policy::MostVisitedNodePolicy::new(&env, &mcts_history);
        let actions = env.action_space(&state);
        let chosen_action = selection_policy
            .select_action(&state, &actions)
            .expect("Can select action");
        println!("I will do {:?}", chosen_action);
        let transition = env.step(&state, &chosen_action).expect("Can step");
        state = transition.new_state;
        let TransitionType::Running = &transition.transition_type else {
            break;
        };
        // TODO: Prune mcts_history of states that are no longer reachable
        wait_for_input();
    }
    println!("Game ended");
    println!("{state:?}");
    Ok(())
}

fn main() {
    if let Err(e) = run_mcts() {
        println!("Run errored with {e}");
    };
}
