use explorers::goap::{Action, BTreeState, Planner, Value};

/// Represents the possible actions we could take.
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
                // Buying a food item costs money and gives you the food.
                state.insert("has_money".to_string(), false.into());
                state.insert(format!("has_food<{}>", f), true.into());
                Ok(1) // Buying is quick.
            }
            Self::Cook(f) => {
                // Cooking a food item costs ingredients and gives you the food.
                state.insert(format!("has_ingredients<{}>", f), false.into());
                state.insert(format!("has_food<{}>", f), true.into());
                Ok(2) // Cooking takes longer than buying.
            }
            Self::Eat(f) => {
                // Eating a food item costs the food and sates your hunger.
                state.insert(format!("has_food<{}>", f), false.into());
                state.insert("hungry".to_string(), false.into());
                Ok(1) // Eating is quick.
            }
        }
    }
}

/// What actions can I take in a given state?
fn pizza_action_factory(state: &BTreeState) -> Vec<EatingActions> {
    let mut actions: Vec<EatingActions> = vec![];
    // I can only buy pizza if I have money.
    if state.get("has_money") == Some(&Value::Bool(true)) {
        actions.push(EatingActions::Buy("pizza".to_string()));
    }
    // If I have ingredients, I can cook food.
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
    // If I have food, I can eat it.
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
    actions
}

fn main() {
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
    // Since cooking takes longer than buying, we should prefer to buy pizza if we have money.
    let current_state = BTreeState::default()
        // --------------------------------------------------------------------------------------------
        // Edit this line and rerun to see the plan change.
        .with("has_money", true.into())
        // --------------------------------------------------------------------------------------------
        .with("has_ingredients<pizza>", true.into());

    let goal_state = BTreeState::default().with("hungry", false.into());

    // Given a current state and a goal state, determine a sequence of actions to reach the goal state
    let actions = planner
        .plan_for_goal()
        .current_state(&current_state)
        .goal_state(&goal_state)
        .plan()
        .expect("Can plan actions");

    println!("Given starting state: {current_state:?}");
    println!("And goal state: {goal_state:?}");
    println!("Plan is {actions:?}");
}
