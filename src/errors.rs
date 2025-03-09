use thiserror::Error;

#[derive(Debug, Error)]
pub enum SimulationError {
    #[error("Physics error: {0}")]
    PhysicsError(String),

    #[error("System error: {0}")]
    SystemError(String),

    #[error("Mission error: {0}")]
    MissionError(String),

    #[error("Initialization error: {0}")]
    InitializationError(String),
}
