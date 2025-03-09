use super::propulsion::PropulsionSystem;
use crate::control::fuel_managment::{FlightPhase, FuelManagementSystem};

pub struct Stage {
    pub propulsion: PropulsionSystem,
    pub burn_time: f64,
    pub is_active: bool,
}

impl Stage {
    pub fn new(fuel_mass: f64, dry_mass: f64, burn_rate: f64, burn_time: f64) -> Self {
        let propulsion_system = PropulsionSystem::new(fuel_mass, burn_rate, dry_mass);
        Stage {
            propulsion: propulsion_system,
            burn_time,
            is_active: false,
        }
    }

    pub fn activate(&mut self) {
        self.is_active = true;
        self.propulsion.activate();
        println!("Stage activated. Engine startup sequence initiated.");
    }

    pub fn update(
        &mut self,
        throttle: f64,
        delta_time: f64,
        atmospheric_pressure: f64,
        fuel_management: &mut FuelManagementSystem,
        flight_phase: FlightPhase,
    ) -> f64 {
        self.burn_time -= delta_time;

        if self.is_active && !self.is_burn_complete() {
            let thrust = self
                .propulsion
                .update(throttle, delta_time, atmospheric_pressure);
            let fuel_consumed =
                FuelManagementSystem::calculate_fuel_consumption(thrust, delta_time);

            fuel_management.consume_fuel(fuel_consumed, flight_phase);
            self.propulsion.consume_fuel(fuel_consumed);

            if self.is_depleted() || self.is_burn_complete() {
                self.is_active = false;
                println!("Stage deactivated");
            }
            thrust
        } else {
            0.0
        }
    }

    pub fn get_thrust(&self) -> f64 {
        if self.is_active {
            self.propulsion.get_thrust()
        } else {
            0.0
        }
    }

    pub fn is_depleted(&self) -> bool {
        self.propulsion.fuel_mass <= 0.0
    }

    pub fn is_burn_complete(&self) -> bool {
        self.burn_time <= 0.0
    }

    pub fn get_total_mass(&self) -> f64 {
        self.propulsion.get_total_mass()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::{
        fuel_managment::{FlightPhase, FuelManagementSystem},
        propulsion::EngineState,
    };

    #[test]
    fn test_stage_activation() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 60.0); // Fuel mass, dry mass, burn rate, burn time
        stage.activate();
        assert!(stage.is_active, "Stage should be active after activation");
        assert!(matches!(
            stage.propulsion.get_engine_state(),
            EngineState::StartingUp
        ));
    }

    #[test]
    fn test_stage_thrust_generation() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 60.0);
        let mut fuel_management = FuelManagementSystem::new(1000.0); // Assume 1000 kg of total fuel

        stage.activate();
        let throttle = 1.0;
        let delta_time = 1.0;
        let atmospheric_pressure = 101325.0; // Sea level pressure
        let thrust = stage.update(
            throttle,
            delta_time,
            atmospheric_pressure,
            &mut fuel_management,
            FlightPhase::Ascent,
        );

        assert!(thrust > 0.0, "Thrust should be generated during stage burn");

        assert!(
            fuel_management.get_available_fuel(FlightPhase::Ascent) < 1000.0,
            "Fuel should be consumed during stage burn"
        );

        assert!(
            fuel_management.get_total_remaining_fuel() > 0.0,
            "There should be remaining fuel after the update"
        );
    }

    #[test]
    fn test_stage_burn_completion() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 5.0); // Set burn time to 5 seconds
        let mut fuel_management = FuelManagementSystem::new(1000.0);

        stage.activate();
        let throttle = 1.0;
        let delta_time = 1.0;
        let atmospheric_pressure = 101325.0;

        for i in 0..5 {
            stage.update(
                throttle,
                delta_time,
                atmospheric_pressure,
                &mut fuel_management,
                FlightPhase::Ascent,
            );
            println!("Iteration {}: burn time {}", i + 1, stage.burn_time);
        }

        assert!(
            stage.is_burn_complete(),
            "Stage should complete its burn after the specified burn time"
        );
        assert!(
            !stage.is_active,
            "Stage should deactivate after burn completion"
        );
    }

    #[test]
    fn test_stage_fuel_depletion() {
        let mut stage = Stage::new(10.0, 500.0, 10.0, 60.0); // Set small fuel mass to test depletion
        let mut fuel_management = FuelManagementSystem::new(10.0);

        stage.activate();
        let throttle = 1.0;
        let delta_time = 1.0;
        let atmospheric_pressure = 101325.0;

        // Simulate fuel depletion
        for _ in 0..2 {
            stage.update(
                throttle,
                delta_time,
                atmospheric_pressure,
                &mut fuel_management,
                FlightPhase::Ascent,
            );
        }

        assert!(
            stage.is_depleted(),
            "Stage should deplete its fuel after consumption"
        );
        assert!(
            !stage.is_active,
            "Stage should deactivate after fuel depletion"
        );
    }
}
