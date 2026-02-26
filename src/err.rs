use thiserror::Error;

#[derive(Clone, Debug, Error)]
pub enum WadooErr {
    #[error("At least one child is required.")]
    EmptyChildren,
}
