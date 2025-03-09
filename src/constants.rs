// Physical Constants
pub const GRAVITY: f64 = 9.81; // m/s²
pub const EARTH_RADIUS: f64 = 6_371_000.0; // meters
pub const GRAVITATIONAL_CONSTANT: f64 = 6.67430e-11; // N⋅m²/kg²

// Rocket Constants
pub const ROCKET_THRUST: f64 = 8_000_000.0; // N (8 MN for better realism)
pub const ROCKET_DRAG_COEFFICIENT: f64 = 0.2;
pub const ROCKET_CROSS_SECTIONAL_AREA: f64 = 5.0; // m²

// Environmental Constants
pub const SEA_LEVEL_TEMPERATURE: f64 = 288.15; // K
pub const TROPOSPHERE_TEMP_GRADIENT: f64 = -6.5 / 1_000.0; // °C per meter
pub const TROPOSPHERE_HEIGHT: f64 = 11_000.0; // m

// Simulation Parameters
pub const TIME_STEP: f64 = 1.0; // s
pub const MAX_SIMULATION_TIME: f64 = 86400.0; // s

// Aerodynamic Constants
pub const AIR_DENSITY_SEA_LEVEL: f64 = 1.225; // kg/m³

// Propulsion Constants
pub const EXHAUST_VELOCITY: f64 = 3000.0; // m/s (typical for liquid fuel rockets)

pub const SPECIFIC_IMPULSE_SEA_LEVEL: f64 = 263.0; // seconds (typical for a first-stage rocket engine)
pub const SPECIFIC_IMPULSE_VACUUM: f64 = 295.0; // seconds (typical for a first-stage rocket engine in vacuum)
pub const SEA_LEVEL_PRESSURE: f64 = 101325.0; // Pa (pascals)
