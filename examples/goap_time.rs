use explorers::goap::{Action, BTreeState, Planner, Value};

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

/// The event factory updates the state after an action is applied. In this case, we check if time has progressed
/// and move the obstacle down accordingly.
/// It would be entirely possible to implement this logic in the action itself, and would possibly be more practical
/// in this case. However, we separate it here to illustrate how an event factory can be used.
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
    state_after_action.update("obstacle_y", |obstacle_y| {
        let Value::U32(y) = obstacle_y else {
            return None;
        };
        *y = y.saturating_sub(time_delta);
        Some(Value::U32(*y))
    });

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

fn main() {
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

    println!("Actions: {:?}", actions);

    // Replay the scanerio, including events, to verify the correctness of the plan
    for action in actions.path.iter() {
        let state_before_action = current_state.clone();
        // Run the action, which modifies state accordingly
        action
            .forward(&mut current_state)
            .expect("Can apply action to state");

        // Apply any event factory effects
        event_factory(&state_before_action, &mut current_state);

        // Did a collision happen?
        if current_state.get("collision").unwrap() == &Value::Bool(true) {
            println!("Collision happened! :(");
            return;
        };
    }
    assert!(current_state.get("obstacle_y").unwrap() == &Value::U32(0));
    assert!(current_state.get("position").unwrap() == &Value::I32(-1));
}
