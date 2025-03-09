use crate::{control::environment::Environment, utils::vector2d::Vector2D};

#[derive(Debug)]
pub struct Aerodynamics {
    pub drag_coefficient: f64,
    pub surface_area: f64,
    pub lift_coefficient: f64,
}

impl Aerodynamics {
    pub fn new(drag_coefficient: f64, surface_area: f64, lift_coefficient: f64) -> Self {
        Aerodynamics {
            drag_coefficient,
            surface_area,
            lift_coefficient,
        }
    }

    pub fn calculate_aerodynamic_force(
        &self,
        velocity: Vector2D,
        angle_of_attack: f64,
        environment: &Environment,
    ) -> Vector2D {
        let drag_vector = self.calculate_drag(velocity, angle_of_attack, environment);
        let lift_vector = self.calculate_lift(velocity, angle_of_attack, environment);

        lift_vector + drag_vector
    }

    pub fn calculate_drag(
        &self,
        velocity: Vector2D,
        angle_of_attack: f64,
        environment: &Environment,
    ) -> Vector2D {
        let dynamic_pressure = self.calculate_dynamic_pressure(velocity, environment);
        let drag_coefficient_adjusted =
            self.drag_coefficient * (1.0 + 0.1 * angle_of_attack.abs().sin());
        let drag_magnitude = dynamic_pressure * self.surface_area * drag_coefficient_adjusted;

        if velocity.magnitude() > 0.0 {
            -velocity.normalize() * drag_magnitude
        } else {
            Vector2D::new(0.0, 0.0)
        }
    }

    pub fn calculate_lift(
        &self,
        velocity: Vector2D,
        angle_of_attack: f64,
        environment: &Environment,
    ) -> Vector2D {
        let dynamic_pressure = self.calculate_dynamic_pressure(velocity, environment);
        let lift_magnitude =
            dynamic_pressure * self.surface_area * self.lift_coefficient * angle_of_attack.sin();
        let velocity_unit = velocity.normalize();


        if velocity.magnitude() > 0.0 {
            Vector2D::new(-velocity_unit.y, velocity_unit.x) * lift_magnitude
        } else {
            Vector2D::new(0.0, 0.0)
        }
    }

    fn calculate_dynamic_pressure(&self, velocity: Vector2D, environment: &Environment) -> f64 {
        let speed = velocity.magnitude();
        let dynamic_pressure = 0.5 * environment.air_density * speed.powi(2);

        dynamic_pressure
    }

    pub fn calculate_re_entry_heating(&self, velocity: f64, air_density: f64) -> f64 {
        let heat_transfer_coefficient = 1.7415e-4; // Approximate value, adjust based on rocket design
        0.5 * heat_transfer_coefficient * air_density * velocity.powi(3)
    }

    pub fn calculate_re_entry_drag(&self, velocity: Vector2D, air_density: f64) -> Vector2D {
        let speed = velocity.magnitude();
        let drag_coefficient = self.calculate_re_entry_drag_coefficient(speed);
        let drag_magnitude =
            0.5 * drag_coefficient * air_density * speed.powi(2) * self.surface_area;
        -velocity.normalize() * drag_magnitude
    }

    fn calculate_re_entry_drag_coefficient(&self, speed: f64) -> f64 {
        // Simplified drag coefficient model for re-entry
        if speed < 1000.0 {
            self.drag_coefficient
        } else if speed < 5000.0 {
            self.drag_coefficient * (1.0 + (speed - 1000.0) / 4000.0)
        } else {
            self.drag_coefficient * 2.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::mission::CelestialBody;
    use crate::utils::vector2d::Vector2D;
    use approx::assert_relative_eq;

    const EPSILON: f64 = 1e-9;

    fn create_earth_environment() -> Environment {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        Environment::new(earth)
    }

    fn create_mars_environment() -> Environment {
        let mars = CelestialBody::new(
            "Mars".to_string(),
            Vector2D::new(0.0, 0.0),
            3_389_500.0,
            6.39e23,
        );
        Environment::new(mars)
    }

    #[test]
    fn test_drag_calculation_at_sea_level() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = Vector2D::new(100.0, 0.0);
        let angle_of_attack = 0.0;
        let mut environment = create_earth_environment();
        environment.update(
            &Vector2D::new(0.0, 6_371_000.0),
            &[environment.current_body.clone()],
        );

        let drag = aero.calculate_drag(velocity, angle_of_attack, &environment);

        assert_relative_eq!(drag.x, -30625.306649767364, epsilon = EPSILON);
        assert_relative_eq!(drag.y, 0.0, epsilon = EPSILON);
    }

    #[test]
    fn test_lift_calculation_at_angle_of_attack() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = Vector2D::new(100.0, 0.0);
        let angle_of_attack = std::f64::consts::PI / 6.0;
        let mut environment = create_earth_environment();
        environment.update(
            &Vector2D::new(0.0, 6_371_000.0),
            &[environment.current_body.clone()],
        );

        let lift = aero.calculate_lift(velocity, angle_of_attack, &environment);

        assert_relative_eq!(lift.x, 0.0, epsilon = EPSILON);
        assert_relative_eq!(lift.y, 9187.591994930208, epsilon = EPSILON);
    }

    #[test]
    fn test_aerodynamic_force_combination() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = Vector2D::new(100.0, 0.0);
        let angle_of_attack = std::f64::consts::PI / 6.0;
        let mut environment = create_earth_environment();
        environment.update(
            &Vector2D::new(0.0, 6_371_000.0),
            &[environment.current_body.clone()],
        );

        let force = aero.calculate_aerodynamic_force(velocity, angle_of_attack, &environment);

        assert_relative_eq!(force.x, -32156.571982255733, epsilon = EPSILON);
        assert_relative_eq!(force.y, 9187.591994930208, epsilon = EPSILON);
    }

    #[test]
    fn test_re_entry_heating() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = 7000.0;
        let air_density = 0.001;

        let heating = aero.calculate_re_entry_heating(velocity, air_density);

        assert_relative_eq!(heating, 29866.725000000002, epsilon = EPSILON);
    }

    #[test]
    fn test_re_entry_drag() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = Vector2D::new(-5000.0, -5000.0);
        let air_density = 0.001;

        let drag = aero.calculate_re_entry_drag(velocity, air_density);

        let speed = velocity.magnitude();
        let drag_coefficient = aero.calculate_re_entry_drag_coefficient(speed);
        let expected_magnitude =
            0.5 * drag_coefficient * air_density * speed.powi(2) * aero.surface_area;
        let expected_drag = -velocity.normalize() * expected_magnitude;

        assert_relative_eq!(drag.x, expected_drag.x, epsilon = EPSILON);
        assert_relative_eq!(drag.y, expected_drag.y, epsilon = EPSILON);
        assert_relative_eq!(drag.magnitude(), expected_magnitude, epsilon = EPSILON);
    }

    #[test]
    fn test_drag_coefficient_variation_with_speed() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);

        assert_relative_eq!(
            aero.calculate_re_entry_drag_coefficient(500.0),
            0.5,
            epsilon = EPSILON
        );
        assert_relative_eq!(
            aero.calculate_re_entry_drag_coefficient(2000.0),
            0.625,
            epsilon = EPSILON
        );
        assert_relative_eq!(
            aero.calculate_re_entry_drag_coefficient(6000.0),
            1.0,
            epsilon = EPSILON
        );
    }

    #[test]
    fn test_zero_velocity_edge_case() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = Vector2D::new(0.0, 0.0);
        let angle_of_attack = std::f64::consts::PI / 4.0;
        let mut environment = create_earth_environment();
        environment.update(
            &Vector2D::new(0.0, 6_371_000.0),
            &[environment.current_body.clone()],
        );

        let force = aero.calculate_aerodynamic_force(velocity, angle_of_attack, &environment);

        assert_relative_eq!(force.x, 0.0, epsilon = EPSILON);
        assert_relative_eq!(force.y, 0.0, epsilon = EPSILON);
    }

    #[test]
    fn test_high_altitude_aerodynamics() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = Vector2D::new(1000.0, 1000.0);
        let angle_of_attack = std::f64::consts::PI / 4.0;
        let mut environment = create_earth_environment();

        // Update environment to simulate conditions at 80 km altitude
        environment.update(
            &Vector2D::new(0.0, 6_371_000.0 + 80_000.0),
            &[environment.current_body.clone()],
        );

        let force = aero.calculate_aerodynamic_force(velocity, angle_of_attack, &environment);

        // Adjust the air density to a more accurate value for 80 km altitude
        let dynamic_pressure = 0.5 * environment.air_density * velocity.magnitude().powi(2);
        let expected_force_magnitude =
            dynamic_pressure * aero.surface_area * (aero.drag_coefficient + aero.lift_coefficient);

        assert_relative_eq!(force.magnitude(), expected_force_magnitude, epsilon = 1e-6);

        // Since the air density is extremely low at 80 km, the force magnitude should be very small
        assert!(
            force.magnitude() < 1.0,
            "Force magnitude at high altitude should be less than 1 N, got {} N",
            force.magnitude()
        );
    }

    #[test]
    fn test_mars_aerodynamics() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = Vector2D::new(100.0, 0.0);
        let angle_of_attack = std::f64::consts::PI / 6.0;
        let mut environment = create_mars_environment();
        environment.update(
            &Vector2D::new(0.0, 3_389_500.0),
            &[environment.current_body.clone()],
        );

        let force = aero.calculate_aerodynamic_force(velocity, angle_of_attack, &environment);

        // The force on Mars should be significantly less than on Earth due to lower atmospheric density
        assert!(
            force.magnitude() < 1000.0,
            "Force on Mars should be much less than on Earth"
        );
    }

    #[test]
    fn test_space_aerodynamics() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let velocity = Vector2D::new(1000.0, 1000.0);
        let angle_of_attack = std::f64::consts::PI / 4.0;
        let mut environment = create_earth_environment();

        // Update environment to space (1,000 km altitude)
        environment.update(
            &Vector2D::new(0.0, 6_371_000.0 + 1_000_000.0),
            &[environment.current_body.clone()],
        );

        let force = aero.calculate_aerodynamic_force(velocity, angle_of_attack, &environment);

        // In space, air density should be effectively zero
        assert!(
            environment.air_density < 1e-15,
            "Space air density should be effectively zero"
        );

        // Aerodynamic forces should be zero in space
        assert!(
            force.magnitude() < 1e-12,
            "Aerodynamic force in space should be effectively zero"
        );
        assert_relative_eq!(force.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(force.y, 0.0, epsilon = 1e-12);

        // Verify that drag and lift are also zero individually
        let drag = aero.calculate_drag(velocity, angle_of_attack, &environment);
        let lift = aero.calculate_lift(velocity, angle_of_attack, &environment);

        assert!(
            drag.magnitude() < 1e-12,
            "Drag force in space should be effectively zero"
        );
        assert!(
            lift.magnitude() < 1e-12,
            "Lift force in space should be effectively zero"
        );
    }
}
