use crate::constants::ROCKET_THRUST;

pub struct PropulsionSystem {
    pub current_thrust: f64,
    pub fuel_mass: f64,
    pub fuel_burn_rate: f64,
    pub dry_mass: f64,
}

impl PropulsionSystem {
    pub fn new(initial_fuel_mass: f64, burn_rate: f64, dry_mass: f64) -> Self {
        PropulsionSystem {
            current_thrust: 0.0,
            fuel_mass: initial_fuel_mass,
            fuel_burn_rate: burn_rate,
            dry_mass,
        }
    }

    pub fn update(&mut self, throttle: f64, delta_time: f64) {
        assert!(
            throttle >= 0.0 && throttle <= 1.0,
            "Throttle must be between 0 and 1"
        );

        if self.fuel_mass > 0.0 {
            let fuel_consumed = throttle * self.fuel_burn_rate * delta_time;
            if self.fuel_mass >= fuel_consumed {
                self.fuel_mass -= fuel_consumed;
                self.current_thrust = ROCKET_THRUST * throttle;
            } else {
                self.fuel_mass = 0.0;
                self.current_thrust = 0.0;
            }
        } else {
            self.current_thrust = 0.0;
        }

        assert!(self.fuel_mass >= 0.0, "Fuel mass cannot be negative");
        assert!(self.current_thrust >= 0.0, "Thrust cannot be negative");
    }

    pub fn get_thrust(&self) -> f64 {
        self.current_thrust
    }

    pub fn get_total_mass(&self) -> f64 {
        self.fuel_mass + self.dry_mass
    }

    pub fn is_out_of_fuel(&self) -> bool {
        self.fuel_mass <= 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::ROCKET_THRUST;

    #[test]
    fn test_new_propulsion_system() {
        let ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        assert_eq!(ps.fuel_mass, 1000.0);
        assert_eq!(ps.fuel_burn_rate, 10.0);
        assert_eq!(ps.dry_mass, 500.0);
        assert_eq!(ps.current_thrust, 0.0);
    }

    #[test]
    fn test_update_with_fuel() {
        let mut ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        ps.update(0.5, 1.0);
        assert_eq!(ps.fuel_mass, 995.0);
        assert_eq!(ps.current_thrust, 0.5 * ROCKET_THRUST);
    }

    #[test]
    fn test_update_out_of_fuel() {
        let mut ps = PropulsionSystem::new(5.0, 10.0, 500.0);
        ps.update(1.0, 1.0);
        assert_eq!(ps.fuel_mass, 0.0);
        assert_eq!(ps.current_thrust, 0.0);
    }

    #[test]
    fn test_is_out_of_fuel() {
        let mut ps = PropulsionSystem::new(10.0, 10.0, 500.0);
        assert_eq!(ps.is_out_of_fuel(), false);
        ps.update(1.0, 1.0);
        assert_eq!(ps.is_out_of_fuel(), true);
    }

    #[test]
    fn test_get_thrust() {
        let mut ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        ps.update(0.75, 1.0);
        assert_eq!(ps.get_thrust(), 0.75 * ROCKET_THRUST);
    }

    #[test]
    fn test_get_total_mass() {
        let mut ps = PropulsionSystem::new(1000.0, 10.0, 500.0);
        assert_eq!(ps.get_total_mass(), 1500.0);
        ps.update(0.5, 10.0);
        assert_eq!(ps.get_total_mass(), 1450.0);
    }
}
