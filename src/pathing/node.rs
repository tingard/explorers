pub struct RRTNode<N> {
    pub node: N,
    pub cumulative_cost: f32,
    pub parent: Option<usize>,
}
