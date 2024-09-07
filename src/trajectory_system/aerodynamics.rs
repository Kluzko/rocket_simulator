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

    /// Calculate aerodynamic force (lift + drag)
    pub fn calculate_aerodynamic_force(
        &self,
        velocity: Vector2D,
        angle_of_attack: f64,
        environment: &Environment,
    ) -> Vector2D {
        let drag_vector = self.calculate_drag(velocity, angle_of_attack, environment);
        let lift_vector = self.calculate_lift(velocity, angle_of_attack, environment);

        // Sum the lift and drag forces
        lift_vector + drag_vector
    }

    /// Calculate drag force based on velocity, angle of attack, and environment
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

    /// Calculate lift force based on velocity, angle of attack, and environment
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
        0.5 * environment.air_density * speed.powi(2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn test_aerodynamic_force() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let env = Environment::new();
        let velocity = Vector2D::new(30.0, 40.0);
        let angle_of_attack = PI / 6.0; // 30 degrees

        let force = aero.calculate_aerodynamic_force(velocity, angle_of_attack, &env);

        // The force should be non-zero
        assert!(force.magnitude() > 0.0);

        // The force should be less than the maximum possible force
        let max_force = velocity.magnitude().powi(2)
            * env.air_density
            * aero.surface_area
            * (aero.lift_coefficient + aero.drag_coefficient);
        assert!(force.magnitude() < max_force);

        // Calculate drag and lift separately
        let drag = aero.calculate_drag(velocity, angle_of_attack, &env);
        let lift = aero.calculate_lift(velocity, angle_of_attack, &env);

        // Verify that the total force is the sum of lift and drag
        assert_relative_eq!(force.x, drag.x + lift.x, epsilon = 1e-6);
        assert_relative_eq!(force.y, drag.y + lift.y, epsilon = 1e-6);

        // Verify that drag is opposite to the velocity direction
        assert!(drag.x * velocity.x < 0.0);
        assert!(drag.y * velocity.y < 0.0);

        // Verify that lift is perpendicular to velocity
        assert_relative_eq!(lift.dot(&velocity), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_drag_calculation() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let env = Environment::new();
        let velocity = Vector2D::new(30.0, 40.0);
        let angle_of_attack = PI / 6.0; // 30 degrees

        let drag = aero.calculate_drag(velocity, angle_of_attack, &env);

        // Drag should be in the opposite direction of velocity
        assert!(drag.x * velocity.x < 0.0);
        assert!(drag.y * velocity.y < 0.0);

        // Calculate expected drag magnitude with angle of attack adjustment
        let speed = velocity.magnitude();
        let dynamic_pressure = 0.5 * env.air_density * speed.powi(2);
        let drag_coefficient_adjusted = 0.5 * (1.0 + 0.1 * (PI / 6.0).abs().sin());
        let expected_magnitude = dynamic_pressure * aero.surface_area * drag_coefficient_adjusted;

        assert_relative_eq!(drag.magnitude(), expected_magnitude, epsilon = 1e-6);
    }

    #[test]
    fn test_lift_calculation() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let env = Environment::new();
        let velocity = Vector2D::new(30.0, 40.0);
        let angle_of_attack = PI / 6.0; // 30 degrees

        let lift = aero.calculate_lift(velocity, angle_of_attack, &env);

        // Lift should be perpendicular to velocity
        assert_relative_eq!(lift.dot(&velocity), 0.0, epsilon = 1e-6);

        let expected_magnitude = 0.5 * 0.3 * 1.225 * 50.0_f64.powi(2) * 10.0 * (PI / 6.0).sin();
        assert_relative_eq!(lift.magnitude(), expected_magnitude, epsilon = 1e-6);
    }

    #[test]
    fn test_zero_velocity_no_forces() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let env = Environment::new();
        let velocity = Vector2D::new(0.0, 0.0);
        let angle_of_attack = PI / 6.0;

        let lift = aero.calculate_lift(velocity, angle_of_attack, &env);
        let drag = aero.calculate_drag(velocity, angle_of_attack, &env);

        // With zero velocity, both lift and drag should be zero
        assert_relative_eq!(lift.magnitude(), 0.0, epsilon = 1e-6);
        assert_relative_eq!(drag.magnitude(), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_high_angle_of_attack_drag_increase() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let env = Environment::new();
        let velocity = Vector2D::new(30.0, 40.0);

        // Compare drag at different angles of attack
        let drag_low_aoa = aero.calculate_drag(velocity, PI / 12.0, &env); // 15 degrees
        let drag_high_aoa = aero.calculate_drag(velocity, PI / 3.0, &env); // 60 degrees

        // Drag should be higher at higher angle of attack
        assert!(drag_high_aoa.magnitude() > drag_low_aoa.magnitude());
    }
}
