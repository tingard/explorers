use anyhow::anyhow;
use geo::Distance;

pub fn cell_as_polygon(cell: h3o::CellIndex) -> geo::Polygon {
    let cell_bounds = geo::LineString::new(
        cell.boundary()
            .iter()
            .map(|c| geo::coord! { x: c.lng(), y: c.lat() })
            .collect(),
    );
    geo::Polygon::new(cell_bounds, vec![])
}

/// Check whether a given cell is fully contained within an geospatial area (provided in WGS84)
pub fn is_cell_contained_in_in_geometry<G: geo::Contains<geo::Polygon>>(
    cell: h3o::CellIndex,
    area: &G,
) -> bool {
    area.contains(&cell_as_polygon(cell))
}

/// Return the portal (a pair of geospatial points) connecting two cells.
///
/// Returns an error if the two cells are not adjacent to each other.
pub fn portal_between(
    a: &h3o::CellIndex,
    b: &h3o::CellIndex,
) -> anyhow::Result<(geo::Coord, geo::Coord)> {
    a.edge(*b)
        .map(|v| {
            let bounds = v.boundary();
            let mut coord_gen = bounds
                .into_iter()
                .map(|c| geo::coord! { x: c.lng(), y: c.lat() });
            let right = coord_gen.next().expect("Has left coord");
            let left = coord_gen.next().expect("Has right coord");
            Ok((left, right))
        })
        .unwrap_or(Err(anyhow!("Not all hexagons are adjacent.")))
}

/// Convert a vec of hexes to an iterator of portals (pairs of geospatial points).
///
/// Returns an error if any two cells are not adjacent to each other.
pub fn hexes_to_portals<'a>(
    hexes: &'a [h3o::CellIndex],
) -> impl Iterator<Item = anyhow::Result<(geo::Coord, geo::Coord)>> + 'a {
    hexes
        .iter()
        .zip(hexes.iter().skip(1))
        .map(|(a, b)| portal_between(a, b))
}

pub fn hex_center_distance(a: h3o::CellIndex, b: h3o::CellIndex) -> f64 {
    let center_a = h3o::LatLng::from(a);
    let center_b = h3o::LatLng::from(b);
    geo::Geodesic.distance(
        geo::point! {
            x: center_a.lng(),
            y: center_a.lat()
        },
        geo::point! {
            x: center_b.lng(),
            y: center_b.lat()
        },
    )
}
