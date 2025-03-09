use crate::constants::EXHAUST_VELOCITY;
use std::f64::EPSILON;

pub struct FuelManagementSystem {
    total_fuel: f64,
    ascent_fuel: f64,
    transfer_fuel: f64,
    descent_fuel: f64,
    reserve_fuel: f64,
    current_phase: FlightPhase,
}

impl FuelManagementSystem {
    // Constructor for initializing the fuel system
    pub fn new(total_fuel: f64) -> Self {
        let reserve_percentage = 0.05; // 5% reserve
        let ascent_percentage = 0.50; // 50% for ascent
        let transfer_percentage = 0.30; // 30% for transfer
        let descent_percentage = 0.15; // 15% for descent

        FuelManagementSystem {
            total_fuel,
            ascent_fuel: total_fuel * ascent_percentage,
            transfer_fuel: total_fuel * transfer_percentage,
            descent_fuel: total_fuel * descent_percentage,
            reserve_fuel: total_fuel * reserve_percentage,
            current_phase: FlightPhase::Ascent,
        }
    }

    // Consumes fuel for a given phase of flight
    pub fn consume_fuel(&mut self, amount: f64, phase: FlightPhase) -> f64 {
        let available_fuel = match phase {
            FlightPhase::Ascent => &mut self.ascent_fuel,
            FlightPhase::Transfer => &mut self.transfer_fuel,
            FlightPhase::Descent => &mut self.descent_fuel,
        };

        let consumed = amount.min(*available_fuel);
        *available_fuel -= consumed;

        // If we've depleted the fuel for this phase, start using the reserve
        if *available_fuel < EPSILON && self.reserve_fuel > 0.0 {
            let additional = (amount - consumed).min(self.reserve_fuel);
            self.reserve_fuel -= additional;
            consumed + additional
        } else {
            consumed
        }
    }

    // Calculates adaptive fuel consumption based on the current and target conditions
    pub fn optimize_fuel_consumption(
        &mut self,
        current_altitude: f64,
        target_altitude: f64,
        current_velocity: f64,
        target_velocity: f64,
    ) -> f64 {
        let adaptive_throttle = self.calculate_adaptive_throttle(
            current_altitude,
            target_altitude,
            current_velocity,
            target_velocity,
        );

        match self.current_phase {
            FlightPhase::Ascent => {
                let gravity_turn_factor = (1.0 - (current_altitude / target_altitude)).max(0.0);
                adaptive_throttle * (1.0 - 0.2 * gravity_turn_factor)
            }
            FlightPhase::Transfer => {
                let transfer_progress = (current_altitude / target_altitude).min(1.0);
                if transfer_progress < 0.1 || transfer_progress > 0.9 {
                    adaptive_throttle
                } else {
                    adaptive_throttle * 0.5 // Coast during middle of transfer
                }
            }
            FlightPhase::Descent => {
                let time_to_impact = 2.0 * current_altitude / current_velocity;
                if time_to_impact < 10.0 {
                    1.0 // Full throttle for final descent
                } else {
                    adaptive_throttle * 0.3 // Minimal burn during initial descent
                }
            }
        }
    }

    pub fn get_current_phase(&self) -> FlightPhase {
        self.current_phase
    }

    // Sets the current flight phase
    pub fn set_current_phase(&mut self, new_phase: FlightPhase) {
        let remaining_fuel = self.get_available_fuel(self.current_phase);
        match new_phase {
            FlightPhase::Transfer => self.transfer_fuel += remaining_fuel,
            FlightPhase::Descent => self.descent_fuel += remaining_fuel,
            _ => {}
        }
        match self.current_phase {
            FlightPhase::Ascent => self.ascent_fuel = 0.0,
            FlightPhase::Transfer => self.transfer_fuel = 0.0,
            _ => {}
        }
        self.current_phase = new_phase;
    }

    // Calculates the fuel consumption based on thrust and time
    pub fn calculate_fuel_consumption(thrust: f64, delta_time: f64) -> f64 {
        (thrust * delta_time) / EXHAUST_VELOCITY
    }

    // Gets available fuel for a specific flight phase
    pub fn get_available_fuel(&self, phase: FlightPhase) -> f64 {
        match phase {
            FlightPhase::Ascent => self.ascent_fuel,
            FlightPhase::Transfer => self.transfer_fuel,
            FlightPhase::Descent => self.descent_fuel,
        }
    }

    pub fn synchronize_fuel(&mut self, actual_remaining_fuel: f64) {
        let total_current_fuel = self.get_total_remaining_fuel();
        if total_current_fuel > 0.0 {
            let remaining_ratio = actual_remaining_fuel / total_current_fuel;
            self.ascent_fuel *= remaining_ratio;
            self.transfer_fuel *= remaining_ratio;
            self.descent_fuel *= remaining_ratio;
            self.reserve_fuel *= remaining_ratio;
        } else {
            // If total_fuel was 0, redistribute evenly
            let phase_count = 4.0; // ascent, transfer, descent, reserve
            self.ascent_fuel = actual_remaining_fuel / phase_count;
            self.transfer_fuel = actual_remaining_fuel / phase_count;
            self.descent_fuel = actual_remaining_fuel / phase_count;
            self.reserve_fuel = actual_remaining_fuel / phase_count;
        }
        self.total_fuel = actual_remaining_fuel;
    }

    // Returns the total remaining fuel across all phases, including reserves
    pub fn get_total_remaining_fuel(&self) -> f64 {
        self.ascent_fuel + self.transfer_fuel + self.descent_fuel + self.reserve_fuel
    }

    // Gets the remaining reserve fuel
    pub fn get_reserve_fuel(&self) -> f64 {
        self.reserve_fuel
    }

    pub fn set_reserve_fuel(&mut self, new_reserve: f64) {
        self.reserve_fuel = new_reserve;
    }

    // Internal method for calculating adaptive throttle based on current and target parameters
    fn calculate_adaptive_throttle(
        &self,
        current_altitude: f64,
        target_altitude: f64,
        current_velocity: f64,
        target_velocity: f64,
    ) -> f64 {
        let altitude_factor = (target_altitude - current_altitude) / target_altitude;
        let velocity_factor = (target_velocity - current_velocity) / target_velocity;

        let phase_factor = match self.current_phase {
            FlightPhase::Ascent => 1.0,
            FlightPhase::Transfer => 0.8,
            FlightPhase::Descent => 0.6,
        };

        let fuel_factor = self.get_total_remaining_fuel() / self.total_fuel;

        let throttle = (altitude_factor + velocity_factor) * phase_factor * fuel_factor;
        throttle.clamp(0.1, 1.0) // Ensure throttle is between 10% and 100%
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum FlightPhase {
    Ascent,
    Transfer,
    Descent,
}

#[cfg(test)]
mod test {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_fuel_initialization() {
        let total_fuel = 100_000.0;
        let fuel_system = FuelManagementSystem::new(total_fuel);

        // Ensure the fuel is correctly allocated to each phase
        assert_abs_diff_eq!(fuel_system.ascent_fuel, 50_000.0, epsilon = 1e-6); // 50% for ascent
        assert_abs_diff_eq!(fuel_system.transfer_fuel, 30_000.0, epsilon = 1e-6); // 30% for transfer
        assert_abs_diff_eq!(fuel_system.descent_fuel, 15_000.0, epsilon = 1e-6); // 15% for descent
        assert_abs_diff_eq!(fuel_system.reserve_fuel, 5_000.0, epsilon = 1e-6); // 5% for reserve
        assert_abs_diff_eq!(
            fuel_system.get_total_remaining_fuel(),
            total_fuel,
            epsilon = 1e-6
        );
    }

    #[test]
    fn test_fuel_consumption_ascent() {
        let total_fuel = 100_000.0;
        let mut fuel_system = FuelManagementSystem::new(total_fuel);

        // Set phase to Ascent and consume 10,000 kg of fuel
        let fuel_consumed = fuel_system.consume_fuel(10_000.0, FlightPhase::Ascent);

        // Ensure 10,000 kg of ascent fuel was consumed
        assert_abs_diff_eq!(fuel_consumed, 10_000.0, epsilon = 1e-6);
        assert_abs_diff_eq!(fuel_system.ascent_fuel, 40_000.0, epsilon = 1e-6);
    }

    #[test]
    fn test_fuel_consumption_transfer() {
        let total_fuel = 100_000.0;
        let mut fuel_system = FuelManagementSystem::new(total_fuel);

        // Set phase to Transfer and consume 5,000 kg of fuel
        let fuel_consumed = fuel_system.consume_fuel(5_000.0, FlightPhase::Transfer);

        // Ensure 5,000 kg of transfer fuel was consumed
        assert_abs_diff_eq!(fuel_consumed, 5_000.0, epsilon = 1e-6);
        assert_abs_diff_eq!(fuel_system.transfer_fuel, 25_000.0, epsilon = 1e-6);
    }

    #[test]
    fn test_fuel_consumption_descent() {
        let total_fuel = 100_000.0;
        let mut fuel_system = FuelManagementSystem::new(total_fuel);

        // Set phase to Descent and consume 3,000 kg of fuel
        let fuel_consumed = fuel_system.consume_fuel(3_000.0, FlightPhase::Descent);

        // Ensure 3,000 kg of descent fuel was consumed
        assert_abs_diff_eq!(fuel_consumed, 3_000.0, epsilon = 1e-6);
        assert_abs_diff_eq!(fuel_system.descent_fuel, 12_000.0, epsilon = 1e-6);
    }

    #[test]
    fn test_reserve_fuel_usage() {
        let total_fuel = 100_000.0;
        let mut fuel_system = FuelManagementSystem::new(total_fuel);

        // Consume all available fuel in Ascent
        fuel_system.consume_fuel(50_000.0, FlightPhase::Ascent);

        // Try to consume an additional 10,000 kg of fuel, triggering reserve usage
        let fuel_consumed = fuel_system.consume_fuel(10_000.0, FlightPhase::Ascent);

        // Ensure that the reserve fuel was used (only 5,000 kg remains)
        assert_abs_diff_eq!(fuel_consumed, 5_000.0, epsilon = 1e-6);
        assert_abs_diff_eq!(fuel_system.reserve_fuel, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_optimize_fuel_consumption_ascent() {
        let total_fuel = 100_000.0;
        let mut fuel_system = FuelManagementSystem::new(total_fuel);

        // Optimize fuel consumption during Ascent phase
        fuel_system.set_current_phase(FlightPhase::Ascent);
        let optimized_throttle =
            fuel_system.optimize_fuel_consumption(50_000.0, 100_000.0, 2_000.0, 7_000.0);

        // Check the throttle value (since gravity turn applies)
        assert!(optimized_throttle < 1.0 && optimized_throttle > 0.0);
    }

    #[test]
    fn test_optimize_fuel_consumption_transfer() {
        let total_fuel = 100_000.0;
        let mut fuel_system = FuelManagementSystem::new(total_fuel);

        // Optimize fuel consumption during Transfer phase
        fuel_system.set_current_phase(FlightPhase::Transfer);
        let optimized_throttle =
            fuel_system.optimize_fuel_consumption(100_000.0, 300_000.0, 5_000.0, 7_500.0);

        // Check the throttle value (coasting behavior expected)
        assert!(optimized_throttle < 1.0 && optimized_throttle > 0.0);
    }

    #[test]
    fn test_optimize_fuel_consumption_descent() {
        let total_fuel = 100_000.0;
        let mut fuel_system = FuelManagementSystem::new(total_fuel);

        // Optimize fuel consumption during Descent phase
        fuel_system.set_current_phase(FlightPhase::Descent);
        let optimized_throttle =
            fuel_system.optimize_fuel_consumption(10_000.0, 100_000.0, 500.0, 1_000.0);

        // Check the throttle value (minimal burn expected)
        assert!(optimized_throttle < 1.0 && optimized_throttle > 0.0);
    }
}
