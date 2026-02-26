use super::bearings::{signed_interior_angle_cartesian, signed_interior_angle_geo};

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeftRight {
    #[default]
    LEFT = 0,
    RIGHT = 1,
}

impl LeftRight {
    pub fn next(&self) -> LeftRight {
        match self {
            LeftRight::LEFT => LeftRight::RIGHT,
            LeftRight::RIGHT => LeftRight::LEFT,
        }
    }
}

#[derive(Default, Debug, Clone)]
struct StringPullState<C> {
    pub points: Vec<C>,
    pub cursor: usize,
    pub funnel: usize,
}

impl<C: Copy> StringPullState<C> {
    fn from_portals(portals: &[(C, C)]) -> (Self, Self) {
        // TODO: Correct mis-oriented portals
        let left_points: Vec<C> = portals.iter().map(|(l, _)| l).copied().collect();
        let right_points: Vec<C> = portals.iter().map(|(_, r)| r).copied().collect();
        (
            StringPullState {
                points: left_points,
                cursor: 0,
                funnel: 0,
            },
            StringPullState {
                points: right_points,
                cursor: 0,
                funnel: 0,
            },
        )
    }
}

fn step_string_pull<C, B>(
    this: &mut StringPullState<C>,
    other: &mut StringPullState<C>,
    edge_to_move: &LeftRight,
    path: &mut Vec<C>,
    signed_interior_angle: B,
) where
    C: Copy,
    B: Fn(C, C, C) -> f64,
{
    // Increment the cursor on this edge
    this.cursor += 1;

    // Get the current points mapping the funnel
    let last_path_point = *path.last().unwrap();
    let this_funnel = this.points[this.funnel];
    let other_funnel = other.points[other.funnel];

    // Get the point to test
    let candidate = this.points[this.cursor];

    let mut relative_bearing_to_candidate_this_side =
        signed_interior_angle(last_path_point, this_funnel, candidate);
    let mut relative_bearing_to_candidate_other_side =
        signed_interior_angle(last_path_point, other_funnel, candidate);
    // Apply a correction if we are currently on the left hand side to make directions match
    if edge_to_move == &LeftRight::RIGHT {
        relative_bearing_to_candidate_this_side *= -1.0;
        relative_bearing_to_candidate_other_side *= -1.0;
    }
    if relative_bearing_to_candidate_this_side < 0.0 {
        // Pass
    } else if relative_bearing_to_candidate_other_side > 0.0 {
        // We have crossed over and need to advance the string pull
        path.push(other.points[other.funnel]);
        // Update search state to the new funnel
        this.cursor = other.funnel + 1;
        this.funnel = other.funnel + 1;
        other.cursor = other.funnel + 1;
        other.funnel = other.funnel + 1;
    } else {
        this.funnel = this.cursor;
    }
}

pub fn string_pull_cartesian(
    mut portals: Vec<((f64, f64), (f64, f64))>,
    start: (f64, f64),
    end: (f64, f64),
) -> anyhow::Result<Vec<(f64, f64)>> {
    portals.push((end, end));
    let (mut left_state, mut right_state) = StringPullState::from_portals(&portals).into();
    let mut path = vec![start];
    let mut edge_to_move = LeftRight::default();

    while (left_state.cursor < left_state.points.len() - 1)
        || (right_state.cursor < right_state.points.len() - 1)
    {
        // Assign "this" and "other" depending on which edge of the funnel we are currently moving
        let (this, other) = match edge_to_move {
            LeftRight::LEFT => (&mut left_state, &mut right_state),
            LeftRight::RIGHT => (&mut right_state, &mut left_state),
        };

        step_string_pull(
            this,
            other,
            &edge_to_move,
            &mut path,
            signed_interior_angle_cartesian,
        );
        edge_to_move = edge_to_move.next();
    }
    path.push(end);
    Ok(path)
}

pub fn string_pull_geo(
    mut portals: Vec<(geo::Coord, geo::Coord)>,
    start: geo::Coord,
    end: geo::Coord,
) -> anyhow::Result<Vec<geo::Coord>> {
    portals.push((end, end));
    let (mut left_state, mut right_state) = StringPullState::from_portals(&portals).into();
    let mut path = vec![start];
    let mut edge_to_move = LeftRight::default();

    while (left_state.cursor < left_state.points.len() - 1)
        || (right_state.cursor < right_state.points.len() - 1)
    {
        // Assign "this" and "other" depending on which edge of the funnel we are currently moving
        let (this, other) = match edge_to_move {
            LeftRight::LEFT => (&mut left_state, &mut right_state),
            LeftRight::RIGHT => (&mut right_state, &mut left_state),
        };

        step_string_pull(
            this,
            other,
            &edge_to_move,
            &mut path,
            signed_interior_angle_geo,
        );
        edge_to_move = edge_to_move.next();
    }
    path.push(end);
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::string_pull_geo;

    /// Simple string pull which we would expect to pass through
    #[test]
    fn does_not_string_pull_for_simple_portals() {
        let portals = vec![
            // N.b these are directed edges - should move clockwise in the expected direction of travel!
            (
                geo::coord! { x: 0.0, y: 0.01 },
                geo::coord! { x: 0.0, y: -0.01 },
            ),
            (
                geo::coord! { x: 0.1, y: 0.01 },
                geo::coord! { x: 0.1, y: -0.01 },
            ),
        ];
        let start = geo::coord! {
            x: -0.1, y: 0.0,
        };
        let end = geo::coord! {
            x: 0.2, y: 0.0,
        };
        let result = string_pull_geo(portals, start, end).expect("Can run string pull");
        assert_eq!(result, vec![start, end]);
    }

    /// Simple string pull which we would expect to pass through
    #[test]
    fn pulls_portal_to_go_round_corners_0() {
        let portals = vec![
            (
                geo::coord! { x: 0.01, y: 0.01 },
                geo::coord! { x: 0.01, y: -0.01 },
            ),
            (
                geo::coord! { x: 0.02, y: 0.01 },
                geo::coord! { x: 0.02, y: -0.01 },
            ),
        ];
        let start = geo::coord! {
            x: 0.0, y: 0.04,
        };
        let end = geo::coord! {
            x: 0.03, y: 0.04,
        };
        let result = string_pull_geo(portals.clone(), start, end).expect("Can run string pull");
        assert_eq!(result, vec![start, portals[0].0, portals[1].0, end]);
    }

    #[test]
    fn pulls_portal_to_go_round_corners_1() {
        let portals = vec![
            (
                geo::coord! { x: 0.0, y: 0.01 },
                geo::coord! { x: 0.0, y: -0.01 },
            ),
            (
                geo::coord! { x: 0.1, y: 0.01 },
                geo::coord! { x: 0.1, y: -0.01 },
            ),
        ];
        let start = geo::coord! {
            x: -0.1, y: 0.0,
        };
        let end = geo::coord! {
            x: 0.2, y: 0.04,
        };
        let result = string_pull_geo(portals.clone(), start, end).expect("Can run string pull");
        assert_eq!(result, vec![start, portals[1].0, end]);
    }
    #[test]
    fn pulls_portal_to_go_round_corners_2() {
        let portals = vec![
            (
                geo::coord! { x: 0.0, y: 0.01 },
                geo::coord! { x: 0.0, y: -0.01 },
            ),
            (
                geo::coord! { x: 0.1, y: 0.01 },
                geo::coord! { x: 0.1, y: -0.01 },
            ),
        ];
        let start = geo::coord! {
            x: -0.1, y: 0.04,
        };
        let end = geo::coord! {
            x: 0.2, y: 0.0,
        };
        let result = string_pull_geo(portals.clone(), start, end).expect("Can run string pull");
        assert_eq!(result, vec![start, portals[0].0, end]);
    }

    #[test]
    fn pulls_portal_to_go_round_corners_3() {
        let portals = vec![
            (
                geo::coord! { x: 0.01, y: 0.01 },
                geo::coord! { x: 0.01, y: -0.01 },
            ),
            (
                geo::coord! { x: 0.02, y: 0.01 },
                geo::coord! { x: 0.02, y: -0.01 },
            ),
        ];
        let start = geo::coord! {
            x: 0.0, y: -0.04,
        };
        let end = geo::coord! {
            x: 0.03, y: -0.04,
        };
        let result = string_pull_geo(portals.clone(), start, end).expect("Can run string pull");
        assert_eq!(result, vec![start, portals[0].1, portals[1].1, end]);
    }

    #[test]
    fn pulls_portal_to_go_round_corners_4() {
        let portals = vec![
            (
                geo::coord! { x: 0.0, y: 0.01 },
                geo::coord! { x: 0.0, y: -0.01 },
            ),
            (
                geo::coord! { x: 0.1, y: 0.01 },
                geo::coord! { x: 0.1, y: -0.01 },
            ),
        ];
        let start = geo::coord! {
            x: -0.1, y: 0.0,
        };
        let end = geo::coord! {
            x: 0.2, y: -0.04,
        };
        let result = string_pull_geo(portals.clone(), start, end).expect("Can run string pull");
        assert_eq!(result, vec![start, portals[1].1, end]);
    }
    #[test]
    fn pulls_portal_to_go_round_corners_5() {
        let portals = vec![
            (
                geo::coord! { x: 0.0, y: 0.01 },
                geo::coord! { x: 0.0, y: -0.01 },
            ),
            (
                geo::coord! { x: 0.1, y: 0.01 },
                geo::coord! { x: 0.1, y: -0.01 },
            ),
        ];
        let start = geo::coord! {
            x: -0.1, y: -0.04,
        };
        let end = geo::coord! {
            x: 0.2, y: 0.0,
        };
        let result = string_pull_geo(portals.clone(), start, end).expect("Can run string pull");
        assert_eq!(result, vec![start, portals[0].1, end]);
    }
}
