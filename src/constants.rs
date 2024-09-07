// Physical Constants
pub const GRAVITY: f64 = 9.81; // m/s²
pub const EARTH_RADIUS: f64 = 6_371_000.0; // meters

// Rocket Constants
pub const ROCKET_THRUST: f64 = 10_000_000.0; // N (4 MN for better realism)
pub const ROCKET_DRAG_COEFFICIENT: f64 = 0.2;
pub const ROCKET_CROSS_SECTIONAL_AREA: f64 = 5.0; // m²

// Environmental Constants
pub const SEA_LEVEL_TEMPERATURE: f64 = 288.15; // K
pub const TROPOSPHERE_TEMP_GRADIENT: f64 = -6.5 / 1_000.0; // °C per meter
pub const TROPOSPHERE_HEIGHT: f64 = 11_000.0; // m

// Simulation Parameters
pub const TIME_STEP: f64 = 1.0; // s
pub const MAX_SIMULATION_TIME: f64 = 3600.0; // s

// Aerodynamic Constants
pub const AIR_DENSITY_SEA_LEVEL: f64 = 1.225; // kg/m³
