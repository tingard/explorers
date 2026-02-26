// TODO: Function using h3 as a navigation mesh
mod bearings;
pub mod string_pull;
use anyhow::anyhow;
use geo::{BoundingRect, Centroid, algorithm::bool_ops::BooleanOps};
use h3o::CellIndex;
use std::collections::{HashSet, VecDeque};

pub use self::string_pull::{string_pull_cartesian, string_pull_geo};
use crate::astar::{EdgeToNodeWithCost, astar};

#[derive(Debug, Clone, Default)]
pub struct HexNav {
    geofence: Option<geo_types::MultiPolygon>,
}

impl HexNav {
    pub fn new(geofence: Option<geo::MultiPolygon>) -> Self {
        Self { geofence }
    }

    pub fn clear_geofence(&mut self) {
        self.geofence = None;
    }

    pub fn forbid(&mut self, area: geo::MultiPolygon) {
        self.geofence = Some(
            self.geofence
                .as_ref()
                .map(|g| g.difference(&area))
                .unwrap_or(area),
        );
    }

    pub fn allow(&mut self, area: geo::MultiPolygon) {
        self.geofence = self.geofence.as_ref().map(|a| a.union(&area));
    }

    /// Decompose a geo multipolygons into a set of hexagons at the hex level
    pub fn hexagons_in_area(
        area: &geo::MultiPolygon,
        resolution: h3o::Resolution,
    ) -> anyhow::Result<HashSet<CellIndex>> {
        // TODO: Rust implementation of https://github.com/uber/h3/blob/68fc3aa1cfa9519137083865013104d9b516786b/src/h3lib/lib/algos.c#L893
        // For now we use a more expensive approach which checks every cell in the polygon area
        // TODO: Provision an appropriately sized hashset given the maximum number of cells that could be contained by area
        let mut out = HashSet::<CellIndex>::new();
        // For each sub-polygon in the MultiPolygon
        for polygon in area.iter() {
            // Pick a point on the exterior as a seed
            let Some(seed) = polygon.centroid() else {
                continue;
            };

            let seed_cell = h3o::LatLng::new(seed.0.y, seed.0.x)
                .unwrap()
                .to_cell(resolution);
            // Set our search area to be within the polygon's bounds
            let Some(bounds) = polygon.bounding_rect() else {
                return Err(anyhow!("Could not get boundary of provided polygon."));
            };
            // TODO: use with_capacity
            let mut seen_hexes = HashSet::<CellIndex>::new();
            let mut search_queue = VecDeque::new();

            seen_hexes.insert(seed_cell);
            search_queue.push_back(seed_cell);

            loop {
                let Some(to_check) = search_queue.pop_front() else {
                    break;
                };
                // Store this as an output cell if it is contained within the polygon
                if crate::hex::is_cell_contained_in_in_geometry(to_check, polygon) {
                    out.insert(to_check);
                }
                // For each new neighboring cell
                for neighbor in to_check.edges().map(|edge| edge.destination()) {
                    if seen_hexes.contains(&neighbor) {
                        continue;
                    }
                    // Log this cell as seen
                    seen_hexes.insert(neighbor);
                    // Add this cell to the search queue so any of its neighbors get searched
                    if crate::hex::is_cell_contained_in_in_geometry(neighbor, &bounds) {
                        search_queue.push_back(neighbor);
                    }
                }
            }
        }
        Ok(out)
    }

    pub fn navigate(
        &self,
        start: geo::Coord,
        end: geo::Coord,
        hex_resolution: h3o::Resolution,
    ) -> anyhow::Result<Vec<geo::Coord>> {
        let Some(geofence) = self.geofence.as_ref() else {
            // If we don't have a geofence, we can move as the crow flies
            return Ok(vec![start, end]);
        };
        let start_cell = h3o::LatLng::new(start.y, start.x)
            .expect("Is valid coordinate")
            .to_cell(hex_resolution);
        let end_cell = h3o::LatLng::new(end.y, end.x)
            .expect("Is valid coordinate")
            .to_cell(hex_resolution);
        // Perform A* search with string pulling to determine the path
        let result = astar()
            .get_neighbors(|c: &CellIndex| {
                c.edges()
                    .filter_map(|e| {
                        let dest = e.destination();
                        if crate::hex::is_cell_contained_in_in_geometry(dest, geofence) {
                            // TODO: String-pulled distance
                            let distance = crate::hex::hex_center_distance(*c, dest) as i32;
                            Some(EdgeToNodeWithCost::new((), dest, distance))
                        } else {
                            None
                        }
                    })
                    .collect()
            })
            .is_goal(|c: &CellIndex, g: &CellIndex| c == g)
            .heuristic(|c, g| crate::hex::hex_center_distance(*c, *g) as i32)
            .plan_path(&start_cell, &end_cell)?;
        let hexes: Vec<_> = result.path.into_iter().map(|(_, hex)| hex).collect();
        let portals: anyhow::Result<Vec<_>> = crate::hex::hexes_to_portals(&hexes).collect();
        string_pull_geo(portals?, start, end)
    }
}

#[cfg(test)]
mod tests {
    use geo::Contains;
    use h3o::Resolution;

    use crate::hex::cell_as_polygon;

    use super::HexNav;

    #[test]
    fn test_polygon_to_hex() {
        let boundary_cell = h3o::LatLng::new(51.3, -0.2)
            .unwrap()
            .to_cell(h3o::Resolution::Eight);
        let hex: geo::MultiPolygon = cell_as_polygon(boundary_cell).into();
        let cells = HexNav::hexagons_in_area(&hex, h3o::Resolution::Eight).expect("Can get cells");
        assert_eq!(cells.len(), 1);
        assert_eq!(cells.iter().next(), Some(&boundary_cell));
    }

    #[test]
    fn test_polygon_to_hex_children() {
        let boundary_cell = h3o::LatLng::new(51.3, -0.2)
            .unwrap()
            .to_cell(h3o::Resolution::Eight);
        let hex: geo::MultiPolygon = cell_as_polygon(boundary_cell).into();
        let cells = HexNav::hexagons_in_area(&hex, h3o::Resolution::Ten).expect("Can get cells");
        for c in cells.into_iter() {
            assert_eq!(c.parent(Resolution::Eight), Some(boundary_cell));
        }
    }

    #[test]
    fn simple_hexnav_no_fence() {
        let nav = HexNav::new(None);
        let start = geo::coord! { x:-0.2, y: 51.3 };
        let end = geo::coord! { x:-0.25, y: 51.1 };
        let path = nav
            .navigate(start, end, h3o::Resolution::Nine)
            .expect("Can navigate");

        assert_eq!(path, vec![start, end])
    }

    #[test]
    fn hexagons_in_rect() {
        let area = geo::Rect::new(
            geo::coord! { x:-1.0, y: 50.0 },
            geo::coord! { x:1.0, y: 51.0 },
        );
        let allowed_hexes =
            HexNav::hexagons_in_area(&area.into(), Resolution::Eight).expect("Is ok");
        assert!(!allowed_hexes.is_empty());
    }

    #[test]
    fn simple_hexnav_large_fence() {
        let area = geo::Rect::new(
            geo::coord! { x:-0.26, y: 51.35 },
            geo::coord! { x:-0.15, y: 51.05 },
        );
        let nav = HexNav::new(Some(area.into()));
        let start = geo::coord! { x:-0.2, y: 51.3 };
        let end = geo::coord! { x:-0.25, y: 51.1 };
        assert!(area.contains(&start));
        assert!(area.contains(&end));
        let path = nav
            .navigate(start, end, h3o::Resolution::Eight)
            .expect("Can navigate");
        assert_eq!(path[0], start);
        assert_eq!(path[path.len() - 1], end);
    }
    #[test]
    fn simple_hexnav_large_fence_reverse() {
        let nav = HexNav::new(Some(
            geo::Rect::new(
                geo::coord! { x:-0.26, y: 51.35 },
                geo::coord! { x:-0.15, y: 51.05 },
            )
            .into(),
        ));
        let start = geo::coord! { x:-0.25, y: 51.1 };
        let end = geo::coord! { x:-0.2, y: 51.3 };
        let path = nav
            .navigate(start, end, h3o::Resolution::Eight)
            .expect("Can navigate");
        assert_eq!(path[0], start);
        assert_eq!(path[path.len() - 1], end);
    }
}
