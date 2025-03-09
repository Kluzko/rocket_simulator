use std::time::Duration;

use crate::{
    constants::ROCKET_THRUST, SEA_LEVEL_PRESSURE, SPECIFIC_IMPULSE_SEA_LEVEL,
    SPECIFIC_IMPULSE_VACUUM,
};

pub struct PropulsionSystem {
    pub current_thrust: f64,
    pub fuel_mass: f64,
    pub fuel_burn_rate: f64,
    pub dry_mass: f64,
    engine_state: EngineState,
    startup_time: Duration,
    current_startup_time: Duration,
}
#[derive(Debug)]
pub enum EngineState {
    Inactive,
    StartingUp,
    Active,
    ShuttingDown,
}
impl PropulsionSystem {
    pub fn new(initial_fuel_mass: f64, burn_rate: f64, dry_mass: f64) -> Self {
        PropulsionSystem {
            current_thrust: 0.0,
            fuel_mass: initial_fuel_mass,
            fuel_burn_rate: burn_rate,
            dry_mass,
            engine_state: EngineState::Inactive,
            startup_time: Duration::from_secs(5), // 5 second startup time
            current_startup_time: Duration::new(0, 0),
        }
    }

    pub fn activate(&mut self) {
        if let EngineState::Inactive = self.engine_state {
            self.engine_state = EngineState::StartingUp;
            println!("Engine startup sequence initiated.");
        }
    }

    pub fn update(&mut self, throttle: f64, delta_time: f64, atmospheric_pressure: f64) -> f64 {
        assert!(
            throttle >= 0.0 && throttle <= 1.3,
            "Throttle must be between 0 and 1"
        );

        match self.engine_state {
            EngineState::Inactive => 0.0,
            EngineState::StartingUp => {
                self.current_startup_time += Duration::from_secs_f64(delta_time);
                if self.current_startup_time >= self.startup_time {
                    self.engine_state = EngineState::Active;
                    println!("Engine startup complete. Now active.");
                    self.calculate_thrust(throttle, atmospheric_pressure)
                } else {
                    // Gradual thrust buildup during startup
                    let startup_progress =
                        self.current_startup_time.as_secs_f64() / self.startup_time.as_secs_f64();
                    self.calculate_thrust(throttle * startup_progress, atmospheric_pressure)
                }
            }
            EngineState::Active => self.calculate_thrust(throttle, atmospheric_pressure),
            EngineState::ShuttingDown => {
                self.current_thrust = 0.0;
                self.engine_state = EngineState::Inactive;
                0.0
            }
        }
    }

    fn calculate_thrust(&mut self, throttle: f64, atmospheric_pressure: f64) -> f64 {
        let thrust_sl = ROCKET_THRUST * throttle;
        let thrust_vacuum = thrust_sl * (SPECIFIC_IMPULSE_VACUUM / SPECIFIC_IMPULSE_SEA_LEVEL);
        self.current_thrust = thrust_vacuum
            - (thrust_vacuum - thrust_sl) * (atmospheric_pressure / SEA_LEVEL_PRESSURE);
        self.current_thrust
    }

    pub fn get_engine_state(&self) -> &EngineState {
        &self.engine_state
    }

    pub fn get_thrust(&self) -> f64 {
        self.current_thrust
    }

    pub fn is_active(&self) -> bool {
        matches!(self.engine_state, EngineState::Active)
    }

    pub fn get_total_mass(&self) -> f64 {
        self.fuel_mass + self.dry_mass
    }

    pub fn consume_fuel(&mut self, amount: f64) {
        if self.fuel_mass > 0.0 {
            self.fuel_mass -= amount;
            if self.fuel_mass <= 0.0 {
                self.fuel_mass = 0.0;
                self.engine_state = EngineState::ShuttingDown;
                println!("Engine shutting down due to fuel depletion.");
            }
        }
    }

    pub fn is_out_of_fuel(&self) -> bool {
        self.fuel_mass <= 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_propulsion_system_initialization() {
        let ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        assert_eq!(ps.fuel_mass, 1000.0);
        assert_eq!(ps.fuel_burn_rate, 10.0);
        assert_eq!(ps.dry_mass, 500.0);
        assert_eq!(ps.current_thrust, 0.0);
        assert!(matches!(ps.engine_state, EngineState::Inactive));
    }

    #[test]
    fn test_propulsion_system_activation() {
        let mut ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        ps.activate();
        assert!(matches!(ps.engine_state, EngineState::StartingUp));
    }

    #[test]
    fn test_propulsion_system_startup_sequence() {
        let mut ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        ps.activate();

        // Simulate startup sequence
        for _ in 0..50 {
            ps.update(1.0, 0.1, SEA_LEVEL_PRESSURE);
        }

        assert!(matches!(ps.engine_state, EngineState::Active));
        assert!(ps.current_thrust > 0.0);
    }

    #[test]
    fn test_propulsion_system_thrust_calculation() {
        let mut ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        ps.activate();

        // Fully start the engine
        for _ in 0..50 {
            ps.update(1.0, 0.1, SEA_LEVEL_PRESSURE);
        }

        let thrust_sea_level = ps.calculate_thrust(1.0, SEA_LEVEL_PRESSURE);
        let thrust_vacuum = ps.calculate_thrust(1.0, 0.0);

        assert!(thrust_vacuum > thrust_sea_level);
        assert_relative_eq!(thrust_sea_level, ROCKET_THRUST, epsilon = 1.0);
    }

    #[test]
    fn test_propulsion_system_get_total_mass() {
        let ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        assert_eq!(ps.get_total_mass(), 1500.0);
    }
}
