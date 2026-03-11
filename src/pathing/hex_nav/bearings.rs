use geo::Bearing;

pub fn signed_interior_angle(bearing_a: f64, bearing_b: f64) -> f64 {
    let bearing_a = bearing_a.rem_euclid(360.0);
    let bearing_b = bearing_b.rem_euclid(360.0);
    let mut difference = bearing_a.max(bearing_b) - bearing_a.min(bearing_b);
    if difference > 180.0 {
        difference = 360.0 - difference;
    }
    if (bearing_a + difference).rem_euclid(360.0) == bearing_b {
        difference
    } else {
        -difference
    }
}

pub fn signed_interior_angle_cartesian(
    origin: (f64, f64),
    point_a: (f64, f64),
    point_b: (f64, f64),
) -> f64 {
    let bearing_a = (point_a.1 - origin.1)
        .atan2(point_a.0 - origin.0)
        .to_degrees();
    let bearing_b = (point_b.1 - origin.1)
        .atan2(point_b.0 - origin.0)
        .to_degrees();
    signed_interior_angle(bearing_a, bearing_b)
}

pub fn signed_interior_angle_geo(
    origin: geo::Coord,
    point_a: geo::Coord,
    point_b: geo::Coord,
) -> f64 {
    let bearing_a = geo::Geodesic.bearing(origin.into(), point_a.into()) % 360.0;
    let bearing_b = geo::Geodesic.bearing(origin.into(), point_b.into()) % 360.0;
    signed_interior_angle(bearing_a, bearing_b)
}

#[cfg(test)]
mod tests {
    use super::{signed_interior_angle, signed_interior_angle_geo};

    #[test]
    fn expected_interior_angles() {
        let items = [
            (0.0, 10.0, 10.0),
            (10.0, 0.0, -10.0),
            (0.0, -10.0, -10.0),
            (-10.0, 0.0, 10.0),
            (-10.0, 10.0, 20.0),
            (10.0, -10.0, -20.0),
            (170.0, 190.0, 20.0),
            (190.0, 170.0, -20.0),
            (-10.0, 190.0, -160.0),
            (-10.0, 90.0, 100.0),
            (190.0, -10.0, 160.0),
        ];
        for (a, b, expected) in items.into_iter() {
            assert_eq!(signed_interior_angle(a, b), expected);
        }
    }

    #[test]
    fn bearing_is_zero_for_matching_vectors() {
        let origin = geo::coord! { x: 0.0, y: 0.0};
        let point = geo::coord! { x: 0.0, y: 1.0};
        assert_eq!(signed_interior_angle_geo(origin, point, point), 0.0);
    }

    #[test]
    fn bearing_is_positive_for_clockwise_vectors_0() {
        let origin = geo::coord! { x: 0.0, y: 0.0};
        let point_a = geo::coord! { x: 0.0, y: 1.0};
        let point_b = geo::coord! { x: 1.0, y: 0.0};
        assert_eq!(signed_interior_angle_geo(origin, point_a, point_b), 90.0);
    }
    #[test]
    fn bearing_is_positive_for_clockwise_vectors_1() {
        let origin = geo::coord! { x: 1.0, y: 0.0};
        let point_a = geo::coord! { x: 1.0, y: -1.0};
        let point_b = geo::coord! { x: 0.0, y: -1.0};
        assert!(signed_interior_angle_geo(origin, point_a, point_b) > 0.0);
    }
    #[test]
    fn bearing_is_positive_for_clockwise_vectors_2() {
        let origin = geo::coord! { x: -1.0, y: 1.0};
        let point_a = geo::coord! { x: -2.0, y: 1.0};
        let point_b = geo::coord! { x: -1.9, y: 1.1};
        assert!(signed_interior_angle_geo(origin, point_a, point_b) > 0.0);
    }
    #[test]
    fn bearing_is_negative_for_counter_clockwise_vectors_0() {
        let origin = geo::coord! { x: 0.0, y: 0.0};
        let point_a = geo::coord! { x: 0.0, y: 1.0};
        let point_b = geo::coord! { x: -1.0, y: 0.0};
        assert_eq!(signed_interior_angle_geo(origin, point_a, point_b), -90.0);
    }
    #[test]
    fn bearing_is_negative_for_counter_clockwise_vectors_1() {
        let origin = geo::coord! { x: 1.0, y: 0.0};
        let point_a = geo::coord! { x: 0.0, y: -1.0};
        let point_b = geo::coord! { x: 1.0, y: -1.0};
        assert!(signed_interior_angle_geo(origin, point_a, point_b) < 0.0);
    }
    #[test]
    fn bearing_is_negative_for_counter_clockwise_vectors_2() {
        let origin = geo::coord! { x: -1.0, y: 1.0};
        let point_a = geo::coord! { x: -1.9, y: 1.1};
        let point_b = geo::coord! { x: -2.0, y: 1.0};
        assert!(signed_interior_angle_geo(origin, point_a, point_b) < 0.0);
    }
}
