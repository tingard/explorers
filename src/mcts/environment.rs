use super::action::Action;
use super::state::State;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TransitionType {
    Running,
    Terminated,
    Truncated,
}

#[derive(Debug)]
pub struct EnvironmentTransition<S>
where
    S: State,
{
    pub new_state: S,
    pub reward: f32,
    pub transition_type: TransitionType,
}

pub trait Environment {
    type Action: Action;
    type State: State;
    fn step(
        &self,
        state: &Self::State,
        action: &Self::Action,
    ) -> Result<EnvironmentTransition<Self::State>, anyhow::Error>;

    fn action_space(&self, state: &Self::State) -> Vec<Self::Action>;
}
