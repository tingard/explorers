//! GridWorld is a simple example of a grid-based world. The agent must find the goal, which may require a key to reach through a
//! door. The agent can only see in a small radius around itself, but has perfect memory of its previous moves (Up/Down/Left/Right).
//! It is intended to model a simple, partially observable environment.
use std::{ops::Div, u8};

use explorers::mcts::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Direction {
    Up,
    Down,
    Left,
    Right,
}

impl Direction {
    pub fn apply(&self, location: &mut (usize, usize)) {
        match self {
            // Assume the move is valid. 0, 0 is top left.
            Direction::Up => location.1 -= 1,
            Direction::Down => location.1 += 1,
            Direction::Left => location.0 -= 1,
            Direction::Right => location.0 += 1,
        }
    }
}

/// The ground truth state of the GridWorld environment.
struct GridWorld<const WIDTH: usize, const HEIGHT: usize> {
    // Cols, Rows
    pub occupancy: [[bool; WIDTH]; HEIGHT],
    // Top, Left is 0, 0
    pub key_location: Option<(usize, usize)>,
    pub goal_location: (usize, usize),
    pub player_start_location: (usize, usize),
    pub player_history: Vec<Direction>,
}

impl<const WIDTH: usize, const HEIGHT: usize> GridWorld<WIDTH, HEIGHT> {
    pub fn new(
        occupancy: [[bool; WIDTH]; HEIGHT],
        goal_location: (usize, usize),
        key_location: Option<(usize, usize)>,
        player_start_location: (usize, usize),
    ) -> anyhow::Result<Self> {
        // Check the key is not in a wall
        if let Some((x, y)) = key_location {
            if occupancy[y][x] {
                return Err(anyhow::anyhow!("Key cannot be placed in a wall"));
            }
        }
        // Check the goal is not in a wall
        if occupancy[goal_location.1][goal_location.0] {
            return Err(anyhow::anyhow!("Goal cannot be placed in a wall"));
        }
        // Check the player start location is not in a wall
        if occupancy[player_start_location.1][player_start_location.0] {
            return Err(anyhow::anyhow!("Player start location cannot be in a wall"));
        }
        Ok(Self {
            occupancy,
            goal_location,
            key_location,
            player_start_location,
            player_history: vec![],
        })
    }

    pub fn player_location(&self) -> (usize, usize) {
        let mut location = self.player_start_location;
        self.player_history.iter().for_each(|d| {
            // Assume all previous moves are valid
            d.apply(&mut location);
        });
        todo!();
    }

    pub fn can_move(&self, direction: Direction) -> bool {
        let (x, y) = self.player_location();
        match direction {
            // We can move up only if we're not at the top (y=0), and the space above us is empty
            Direction::Up => y > 0 && !self.occupancy[y - 1][x],
            // We can move down only if we're not at the bottom (y=HEIGHT-1), and the space below us is empty
            Direction::Down => y < HEIGHT - 1 && !self.occupancy[y + 1][x],
            // We can move left only if we're not at the far left (x=0), and the space to our left is empty
            Direction::Left => x > 0 && !self.occupancy[y][x - 1],
            // We can move right only if we're not at the far right (x=WIDTH-1), and the space to our right is empty
            Direction::Right => x < WIDTH - 1 && !self.occupancy[y][x + 1],
        }
    }

    pub fn observation<const DX: usize, const DY: usize>(&self) -> [[u8; DX]; DY] {
        let (x, y) = self.player_location();
        let mut perception = [[0u8; DX]; DY];
        for i in 0..DX {
            for j in 0..DY {
                // Position in the perception grid is (player_pos - SIZE/2 + cursor_pos) but we need to handle overflow correctly
                let px = (x + i).checked_sub(DX.div(2));
                let py = (j + i).checked_sub(DY.div(2));
                match (px, py) {
                    (Some(ii), Some(jj)) => perception[i][j] = self.occupancy[ii][jj] as u8,
                    // It's a wall
                    _ => perception[i][j] = 1,
                }
            }
        }
        // Add the key
        if let Some((kx, ky)) = self.key_location {
            let dx = kx.checked_sub(x).unwrap_or(0);
            let dy = ky.checked_sub(y).unwrap_or(0);
            if dx <= DX.div(2) && dy <= DY.div(2) {
                perception[dx + DX.div(2)][dy + DY.div(2)] = 2;
            }
        }
        perception
    }
}

fn main() {
    todo!();
}
