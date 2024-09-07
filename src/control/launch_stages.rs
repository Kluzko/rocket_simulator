use super::propulsion::PropulsionSystem;

pub struct Stage {
    pub propulsion: PropulsionSystem, // Each stage has its own propulsion system
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
    }

    // Update the stage (handles activation, burning, and deactivation)
    pub fn update(&mut self, throttle: f64, delta_time: f64) {
        if self.is_active {
            self.propulsion.update(throttle, delta_time);
            self.burn_time -= delta_time;

            if self.propulsion.is_out_of_fuel() || self.burn_time <= 0.0 {
                self.is_active = false;
                println!("Debug: Stage deactivated");
            }
        }
    }

    pub fn get_total_mass(&self) -> f64 {
        self.propulsion.get_total_mass()
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_stage() {
        let stage = Stage::new(1000.0, 500.0, 10.0, 100.0);
        assert_eq!(stage.burn_time, 100.0);
        assert_eq!(stage.is_active, false);
    }

    #[test]
    fn test_activate_stage() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 100.0);
        stage.activate();
        assert_eq!(stage.is_active, true);
    }

    #[test]
    fn test_update_inactive_stage() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 100.0);
        stage.update(1.0, 10.0);
        assert_eq!(stage.burn_time, 100.0);
        assert_eq!(stage.is_active, false);
    }

    #[test]
    fn test_update_active_stage() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 100.0);
        stage.activate();
        stage.update(1.0, 10.0);
        assert_eq!(stage.burn_time, 90.0);
        assert_eq!(stage.is_active, true);
    }

    #[test]
    fn test_stage_deactivation_by_burn_time() {
        let mut stage = Stage::new(1000.0, 500.0, 1.0, 100.0);
        stage.activate();
        stage.update(1.0, 100.0);
        assert_eq!(stage.is_active, false);
    }

    #[test]
    fn test_stage_deactivation_by_fuel_depletion() {
        let mut stage = Stage::new(100.0, 500.0, 10.0, 100.0);
        stage.activate();
        stage.update(1.0, 11.0);
        assert_eq!(stage.is_active, false);
    }

    #[test]
    fn test_get_total_mass() {
        let stage = Stage::new(1000.0, 500.0, 10.0, 100.0);
        assert_eq!(stage.get_total_mass(), 1500.0);
    }

    #[test]
    fn test_is_active() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 100.0);
        assert_eq!(stage.is_active, false);
        stage.activate();
        assert_eq!(stage.is_active, true);
    }

    #[test]
    fn test_partial_burn() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 100.0);
        stage.activate();
        stage.update(0.5, 50.0);
        assert_eq!(stage.burn_time, 50.0);
        assert_eq!(stage.is_active, true);
    }

    #[test]
    fn test_zero_throttle() {
        let mut stage = Stage::new(1000.0, 500.0, 10.0, 100.0);
        stage.activate();
        stage.update(0.0, 50.0);
        assert_eq!(stage.burn_time, 50.0);
        assert_eq!(stage.is_active, true);
        // Fuel should not be consumed when throttle is zero
        assert_eq!(stage.propulsion.get_total_mass(), 1500.0);
    }
}
