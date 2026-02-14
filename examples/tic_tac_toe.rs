use std::time::{Duration, Instant};

use anyhow::Context;
use explorers::mcts::{
    policy::{ActionPolicy, StochasticPolicy},
    *,
};

/// Utility function to wait for user input (anything ending in a carriage return)
fn wait_for_input() {
    let mut answer = String::new();
    std::io::stdin()
        .read_line(&mut answer)
        .ok()
        .expect("Failed to read line");
}

#[derive(Eq, PartialEq, Clone, Copy, Debug)]
enum TicTacToePlayer {
    Noughts,
    Crosses,
}

#[derive(Default, Clone, Debug, PartialEq, Eq)]
struct TicTacToeState {
    pub tl: Option<TicTacToePlayer>,
    pub tc: Option<TicTacToePlayer>,
    pub tr: Option<TicTacToePlayer>,
    pub cl: Option<TicTacToePlayer>,
    pub cc: Option<TicTacToePlayer>,
    pub cr: Option<TicTacToePlayer>,
    pub bl: Option<TicTacToePlayer>,
    pub bc: Option<TicTacToePlayer>,
    pub br: Option<TicTacToePlayer>,
}

impl TicTacToeState {
    fn as_bits(val: &Option<TicTacToePlayer>) -> u8 {
        match val {
            None => 0,
            Some(TicTacToePlayer::Noughts) => 1,
            Some(TicTacToePlayer::Crosses) => 2,
        }
    }

    fn player(&self) -> TicTacToePlayer {
        let n_moves_played = (self.tl.is_some() as u32)
            + (self.tc.is_some() as u32)
            + (self.tr.is_some() as u32)
            + (self.cl.is_some() as u32)
            + (self.cc.is_some() as u32)
            + (self.cr.is_some() as u32)
            + (self.bl.is_some() as u32)
            + (self.bc.is_some() as u32)
            + (self.br.is_some() as u32);
        match n_moves_played % 2 == 0 {
            true => TicTacToePlayer::Noughts,
            false => TicTacToePlayer::Crosses,
        }
    }

    fn is_full(&self) -> bool {
        self.tl.is_some()
            && self.tc.is_some()
            && self.tr.is_some()
            && self.cl.is_some()
            && self.cc.is_some()
            && self.cr.is_some()
            && self.bl.is_some()
            && self.bc.is_some()
            && self.br.is_some()
    }
    fn winner(&self) -> Option<TicTacToePlayer> {
        [
            // Rows
            (&self.tl, &self.tc, &self.tr),
            (&self.cl, &self.cc, &self.cr),
            (&self.bl, &self.bc, &self.br),
            // Cols
            (&self.tl, &self.cl, &self.bl),
            (&self.tc, &self.cc, &self.bc),
            (&self.tr, &self.cr, &self.br),
            // Diags
            (&self.tl, &self.cc, &self.br),
            (&self.tr, &self.cc, &self.bl),
        ]
        .into_iter()
        .filter_map(|(i, j, k)| {
            if i.is_some() && i == j && j == k {
                *i
            } else {
                None
            }
        })
        .next()
    }

    fn repr_pos(pos: Option<TicTacToePlayer>) -> &'static str {
        match pos {
            Some(TicTacToePlayer::Noughts) => "O",
            Some(TicTacToePlayer::Crosses) => "X",
            None => " ",
        }
    }
}

impl std::fmt::Display for TicTacToeState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut board = String::new();
        board.push_str(&format!(
            " {} | {} | {} \n",
            Self::repr_pos(self.tl),
            Self::repr_pos(self.tc),
            Self::repr_pos(self.tr)
        ));
        board.push_str(&format!("---+---+---\n"));
        board.push_str(&format!(
            " {} | {} | {} \n",
            Self::repr_pos(self.cl),
            Self::repr_pos(self.cc),
            Self::repr_pos(self.cr)
        ));
        board.push_str(&format!("---+---+---\n"));
        board.push_str(&format!(
            " {} | {} | {} \n",
            Self::repr_pos(self.bl),
            Self::repr_pos(self.bc),
            Self::repr_pos(self.br)
        ));
        write!(f, "{}", board)
    }
}

impl State for TicTacToeState {
    type Key = [u8; 3];

    fn key(&self) -> [u8; 3] {
        let b0 = Self::as_bits(&self.tl) << 6
            | Self::as_bits(&self.tc) << 4
            | Self::as_bits(&self.tr) << 2
            | Self::as_bits(&self.cl);
        let b1 = Self::as_bits(&self.cc) << 6
            | Self::as_bits(&self.cr) << 4
            | Self::as_bits(&self.bl) << 2
            | Self::as_bits(&self.bc);
        let b2 = Self::as_bits(&self.br) << 6;
        return [b0, b1, b2];
    }

    fn is_terminal(&self) -> bool {
        self.winner().is_some() || self.is_full()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum TicTacToePosition {
    TopLeft,
    TopCenter,
    TopRight,
    CenterLeft,
    Center,
    CenterRight,
    BottomLeft,
    BottomCenter,
    BottomRight,
}

impl std::fmt::Display for TicTacToePosition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            TicTacToePosition::TopLeft => "Top Left",
            TicTacToePosition::TopCenter => "Top Center",
            TicTacToePosition::TopRight => "Top Right",
            TicTacToePosition::CenterLeft => "Center Left",
            TicTacToePosition::Center => "Center",
            TicTacToePosition::CenterRight => "Center Right",
            TicTacToePosition::BottomLeft => "Bottom Left",
            TicTacToePosition::BottomCenter => "Bottom Center",
            TicTacToePosition::BottomRight => "Bottom Right",
        };
        write!(f, "{}", name)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct TicTacToeAction {
    position: TicTacToePosition,
    player: TicTacToePlayer,
}

impl TicTacToeAction {
    fn new(player: TicTacToePlayer, position: TicTacToePosition) -> Self {
        Self { player, position }
    }
}

impl Action for TicTacToeAction {
    type Key = [u8; 1];

    fn key(&self) -> [u8; 1] {
        match self.position {
            TicTacToePosition::TopLeft => [0 << 1 | (self.player as u8)],
            TicTacToePosition::TopCenter => [1 << 1 | (self.player as u8)],
            TicTacToePosition::TopRight => [2 << 1 | (self.player as u8)],
            TicTacToePosition::CenterLeft => [3 << 1 | (self.player as u8)],
            TicTacToePosition::Center => [4 << 1 | (self.player as u8)],
            TicTacToePosition::CenterRight => [5 << 1 | (self.player as u8)],
            TicTacToePosition::BottomLeft => [6 << 1 | (self.player as u8)],
            TicTacToePosition::BottomCenter => [7 << 1 | (self.player as u8)],
            TicTacToePosition::BottomRight => [8 << 1 | (self.player as u8)],
        }
    }
}

struct TicTacToeEnvironment {}
impl TicTacToeEnvironment {
    pub fn new() -> Self {
        Self {}
    }
    pub fn initial_state() -> TicTacToeState {
        TicTacToeState::default()
    }
}

impl Environment for TicTacToeEnvironment {
    type Action = TicTacToeAction;
    type State = TicTacToeState;

    fn step(
        &self,
        state: &TicTacToeState,
        action: &TicTacToeAction,
    ) -> Result<EnvironmentTransition<TicTacToeState>, anyhow::Error> {
        if let Some(player) = state.winner() {
            return Err(anyhow::anyhow!("game already terminated - {player:?} won"));
        }
        if state.player() != action.player {
            return Err(anyhow::anyhow!(
                "{:?} tried to play out of turn",
                action.player
            ));
        }
        let mut new_state = state.clone();

        match &action.position {
            TicTacToePosition::TopLeft => {
                if new_state.tl.is_some() {
                    return Err(anyhow::anyhow!("position already filled: TopLeft"));
                }
                new_state.tl = Some(action.player);
            }
            TicTacToePosition::TopCenter => {
                if new_state.tc.is_some() {
                    return Err(anyhow::anyhow!("position already filled: TopCenter"));
                }
                new_state.tc = Some(action.player);
            }
            TicTacToePosition::TopRight => {
                if new_state.tr.is_some() {
                    return Err(anyhow::anyhow!("position already filled: TopRight"));
                }
                new_state.tr = Some(action.player);
            }
            TicTacToePosition::CenterLeft => {
                if new_state.cl.is_some() {
                    return Err(anyhow::anyhow!("position already filled: CenterLeft"));
                }
                new_state.cl = Some(action.player);
            }
            TicTacToePosition::Center => {
                if new_state.cc.is_some() {
                    return Err(anyhow::anyhow!("position already filled: Center"));
                }
                new_state.cc = Some(action.player);
            }
            TicTacToePosition::CenterRight => {
                if new_state.cr.is_some() {
                    return Err(anyhow::anyhow!("position already filled: CenterRight"));
                }
                new_state.cr = Some(action.player);
            }
            TicTacToePosition::BottomLeft => {
                if new_state.bl.is_some() {
                    return Err(anyhow::anyhow!("position already filled: BottomLeft"));
                }
                new_state.bl = Some(action.player);
            }
            TicTacToePosition::BottomCenter => {
                if new_state.bc.is_some() {
                    return Err(anyhow::anyhow!("position already filled: BottomCenter"));
                }
                new_state.bc = Some(action.player);
            }
            TicTacToePosition::BottomRight => {
                if new_state.br.is_some() {
                    return Err(anyhow::anyhow!("position already filled: BottomRight"));
                }
                new_state.br = Some(action.player);
            }
        }
        match (new_state.is_terminal(), new_state.winner()) {
            (false, _) => Ok(EnvironmentTransition {
                new_state,
                reward: 0.0,
                transition_type: TransitionType::Running,
            }),
            (true, None) => Ok(EnvironmentTransition {
                new_state,
                reward: 0.0,
                transition_type: TransitionType::Terminated,
            }),
            (true, Some(winning_player)) => {
                let reward = if winning_player == action.player {
                    1.0
                } else {
                    0.0
                };
                Ok(EnvironmentTransition {
                    new_state,
                    reward,
                    transition_type: TransitionType::Terminated,
                })
            }
        }
    }

    fn action_space(&self, state: &Self::State) -> Vec<Self::Action> {
        if state.winner().is_some() {
            return vec![];
        }
        // There will be at most nine allowed moves
        let mut allowed_actions: Vec<TicTacToeAction> = Vec::with_capacity(9);
        let player = state.player();
        for (slot, position) in [
            (&state.tl, TicTacToePosition::TopLeft),
            (&state.tc, TicTacToePosition::TopCenter),
            (&state.tr, TicTacToePosition::TopRight),
            (&state.cl, TicTacToePosition::CenterLeft),
            (&state.cc, TicTacToePosition::Center),
            (&state.cr, TicTacToePosition::CenterRight),
            (&state.bl, TicTacToePosition::BottomLeft),
            (&state.bc, TicTacToePosition::BottomCenter),
            (&state.br, TicTacToePosition::BottomRight),
        ] {
            if slot.is_none() {
                allowed_actions.push(TicTacToeAction::new(player, position));
            }
        }
        allowed_actions
    }
}

fn rollout(
    env: &TicTacToeEnvironment,
    root: &TicTacToeState,
    policy: &impl ActionPolicy<TicTacToeEnvironment>,
) -> Result<TicTacToeState, anyhow::Error> {
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
    Ok(current_state)
}

/// Backpropagate results from a rollout to the mcts history. We must ensure that appropriate rewards are assigned to each
/// state dependent on the actor and the eventual winner.
fn backpropagate<'a>(
    mcts_history: &mut MCTSStore<TicTacToeEnvironment>,
    states: impl Iterator<Item = &'a TicTacToeState>,
    winner: Option<TicTacToePlayer>,
) {
    // TODO: discount factor?
    for state in states {
        let reward = winner
            .map(|w| if state.player() == w { -1.0 } else { 1.0 })
            .unwrap_or(0.0);
        mcts_history.record_result(state.key(), reward);
    }
}

fn run_mcts() -> anyhow::Result<()> {
    // Initialize our environment
    let env = TicTacToeEnvironment::new();

    // And a starting state
    let mut state = TicTacToeEnvironment::initial_state();

    // Initialize MCTS store
    let mut mcts_history = MCTSStore::new(state.key());

    // Initialize a simulation policy - this will be uniformly random over possible actions (for now)
    let simulation_policy: StochasticPolicy<TicTacToeEnvironment> = StochasticPolicy::uniform();

    // Gameplay loop
    loop {
        println!("State is\n{state}");
        println!("{:?} to play", state.player());
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
            // Unless the game is over, create one or more child nodes and choose an expansion node from them
            if leaf_node.is_terminal() {
                // If the state is terminal, we don't need to expand it further, instead jump straight to
                // Backpropagation
                // For each state we visited - if the player in that state (who acts next) is the
                // eventual winner, they should get a reward
                backpropagate(&mut mcts_history, states_to_leaf.iter(), leaf_node.winner());
                continue;
            }
            // What actions are available for expansion?
            let expansion_actions = env.action_space(&leaf_node);
            let expansion_action = selection_policy
                .select_action(&leaf_node, &expansion_actions)
                .expect("Can select expansion action");

            let expanded_transition = env.step(&leaf_node, expansion_action).expect("Can step");
            let expanded_state = &expanded_transition.new_state;
            let rollout_winner = if expanded_transition.transition_type == TransitionType::Running {
                // Expansion did not result in a terminal state - perform a rollout

                // Simulation
                // Complete a random playout from this expanded state
                let rollout_end_state = rollout(&env, expanded_state, &simulation_policy)?;

                // Backpropagation
                // Which player won the rollout?
                rollout_end_state.winner()
            } else {
                // println!("Expanded state is terminal");
                expanded_state.winner()
            };
            // println!("Rollout winner is {rollout_winner:?}");
            // Record that we ran a simulation from the expanded node + action
            mcts_history.record_simulated_action(leaf_node.key(), expansion_action.key());

            // // Add expanded state to the history - this will not overwrite existing nodes
            // mcts_history.insert_node(expanded_state.key());

            // Update all states up to the expanded node
            backpropagate(
                &mut mcts_history,
                states_to_leaf.iter().chain([expanded_state.clone()].iter()),
                rollout_winner,
            );
        }

        // We will play the action which has the highest number of visits during expansion
        let selection_policy =
            explorers::mcts::policy::MostVisitedNodePolicy::new(&env, &mcts_history);
        let actions = env.action_space(&state);
        let chosen_action = selection_policy
            .select_action(&state, &actions)
            .expect("Can select action");
        println!("{:?} will play {}", state.player(), chosen_action.position);
        let transition = env.step(&state, &chosen_action).expect("Can step");
        state = transition.new_state;
        let TransitionType::Running = &transition.transition_type else {
            break;
        };
        // TODO: Prune mcts_history of states that are no longer reachable
        wait_for_input();
    }
    println!("Game ended");
    println!("{state}");
    Ok(())
}

fn main() {
    if let Err(e) = run_mcts() {
        println!("Run errored with {e}");
    };
}
