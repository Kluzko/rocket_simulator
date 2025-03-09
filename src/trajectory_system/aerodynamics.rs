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

    fn calculate_mach_number(&self, velocity: f64, environment: &Environment) -> f64 {
        // Speed of sound decreases with altitude in a complex way
        // For simplicity, use a basic approximation based on temperature
        let speed_of_sound = 20.05 * environment.temperature.sqrt(); // m/s
        velocity / speed_of_sound
    }

    pub fn calculate_drag(
        &self,
        velocity: Vector2D,
        angle_of_attack: f64,
        environment: &Environment,
    ) -> Vector2D {
        let speed = velocity.magnitude();

        // Early return if no velocity
        if speed < 0.001 {
            return Vector2D::new(0.0, 0.0);
        }

        let mach = self.calculate_mach_number(speed, environment);

        let mach_factor = if mach < 0.8 {
            1.0
        } else if mach < 1.2 {
            1.0 + 5.0 * (mach - 0.8).powi(2)
        } else {
            1.0 + (mach - 1.0).sqrt()
        };

        let angle_effect = 1.0 + angle_of_attack.sin().powi(2);
        let drag_coefficient_adjusted = self.drag_coefficient * angle_effect * mach_factor;

        let dynamic_pressure = 0.5 * environment.air_density * speed.powi(2);

        let drag_magnitude = dynamic_pressure * self.surface_area * drag_coefficient_adjusted;
        -velocity.normalize() * drag_magnitude
    }

    pub fn calculate_lift(
        &self,
        velocity: Vector2D,
        angle_of_attack: f64,
        environment: &Environment,
    ) -> Vector2D {
        let speed = velocity.magnitude();

        if speed < 0.001 {
            return Vector2D::new(0.0, 0.0);
        }

        let max_lift_angle = 0.26;
        let stall_angle = 0.35;

        let effective_lift_coefficient = if angle_of_attack.abs() < stall_angle {
            self.lift_coefficient * (angle_of_attack / max_lift_angle)
        } else {
            self.lift_coefficient
                * (1.0
                    - 0.8
                        * ((angle_of_attack.abs() - stall_angle)
                            / (std::f64::consts::PI / 2.0 - stall_angle)))
                * angle_of_attack.signum()
        };

        let effective_lift_coefficient = effective_lift_coefficient.clamp(-1.5, 1.5);

        let dynamic_pressure = 0.5 * environment.air_density * speed.powi(2);

        let lift_magnitude =
            dynamic_pressure * self.surface_area * effective_lift_coefficient.abs();

        let velocity_unit = velocity.normalize();
        Vector2D::new(-velocity_unit.y, velocity_unit.x)
            * lift_magnitude
            * effective_lift_coefficient.signum()
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
        let angle_of_attack = std::f64::consts::PI / 6.0; // 30 degrees
        let mut environment = create_earth_environment();
        environment.update(
            &Vector2D::new(0.0, 6_371_000.0),
            &[environment.current_body.clone()],
        );

        let lift = aero.calculate_lift(velocity, angle_of_attack, &environment);

        // For horizontal velocity, lift should be purely vertical
        assert_relative_eq!(lift.x, 0.0, epsilon = 1e-6);
        assert!(
            lift.y > 0.0,
            "Lift should be upward for positive angle of attack"
        );

        // Test zero angle of attack
        let zero_lift = aero.calculate_lift(velocity, 0.0, &environment);
        assert_relative_eq!(zero_lift.magnitude(), 0.0, epsilon = 1e-6);

        // Test negative angle of attack - lift should be downward
        let negative_aoa = -angle_of_attack;
        let negative_lift = aero.calculate_lift(velocity, negative_aoa, &environment);

        assert_relative_eq!(negative_lift.x, 0.0, epsilon = 1e-6);
        assert!(
            negative_lift.y < 0.0,
            "Lift should be downward for negative angle of attack"
        );
        assert_relative_eq!(negative_lift.magnitude(), lift.magnitude(), epsilon = 1e-6);
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
        let drag = aero.calculate_drag(velocity, angle_of_attack, &environment);
        let lift = aero.calculate_lift(velocity, angle_of_attack, &environment);

        let expected_force = drag + lift;
        assert_relative_eq!(force.x, expected_force.x, epsilon = 1e-6);
        assert_relative_eq!(force.y, expected_force.y, epsilon = 1e-6);

        assert!(
            force.x < 0.0,
            "Drag component should be opposite to velocity"
        );
        assert!(
            force.y > 0.0,
            "Lift component should be perpendicular to velocity"
        );
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

    #[test]
    fn test_mach_number_calculation() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let mut environment = create_earth_environment();

        // Update environment to sea level conditions
        environment.update(
            &Vector2D::new(0.0, 6_371_000.0),
            &[environment.current_body.clone()],
        );

        // At sea level, speed of sound is approximately 340 m/s
        let expected_speed_of_sound = 20.05 * environment.temperature.sqrt();

        let subsonic_velocity = 300.0;
        let subsonic_mach = aero.calculate_mach_number(subsonic_velocity, &environment);
        assert_relative_eq!(
            subsonic_mach,
            subsonic_velocity / expected_speed_of_sound,
            epsilon = 1e-6
        );
        assert!(subsonic_mach < 1.0, "Should be subsonic");

        let supersonic_velocity = 1000.0;
        let supersonic_mach = aero.calculate_mach_number(supersonic_velocity, &environment);
        assert_relative_eq!(
            supersonic_mach,
            supersonic_velocity / expected_speed_of_sound,
            epsilon = 1e-6
        );
        assert!(supersonic_mach > 1.0, "Should be supersonic");
    }

    #[test]
    fn test_transonic_drag_increase() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let mut environment = create_earth_environment();

        environment.update(
            &Vector2D::new(0.0, 6_371_000.0),
            &[environment.current_body.clone()],
        );

        // Calculate speed of sound
        let speed_of_sound = 20.05 * environment.temperature.sqrt();

        // Create a function to get drag at a specific Mach number
        let get_drag_at_mach = |mach: f64| -> f64 {
            let velocity = Vector2D::new(mach * speed_of_sound, 0.0);
            aero.calculate_drag(velocity, 0.0, &environment).magnitude()
        };

        // Test drag at different Mach numbers
        let subsonic_drag = get_drag_at_mach(0.7); // Subsonic
        let high_subsonic_drag = get_drag_at_mach(0.9); // High subsonic
        let transonic_drag = get_drag_at_mach(1.0); // Transonic

        // Verify increasing drag with Mach number
        assert!(
            high_subsonic_drag > subsonic_drag,
            "Drag should increase as Mach number increases"
        );

        assert!(
            transonic_drag > high_subsonic_drag,
            "Drag should increase in transonic region"
        );

        // Calculate percentage increases
        let subsonic_increase_pct = (high_subsonic_drag - subsonic_drag) / subsonic_drag;
        let transonic_increase_pct = (transonic_drag - high_subsonic_drag) / high_subsonic_drag;

        println!("Subsonic drag increase: {}%", subsonic_increase_pct * 100.0);
        println!(
            "Transonic drag increase: {}%",
            transonic_increase_pct * 100.0
        );

        // The transonic increase should be significant
        assert!(
            transonic_increase_pct > 0.2,
            "Transonic drag increase should be significant (>20%)"
        )
    }

    #[test]
    fn test_stall_behavior() {
        let aero = Aerodynamics::new(0.5, 10.0, 0.3);
        let mut environment = create_earth_environment();

        environment.update(
            &Vector2D::new(0.0, 6_371_000.0),
            &[environment.current_body.clone()],
        );

        let velocity = Vector2D::new(100.0, 0.0);

        // Check lift at various angles of attack
        let max_lift_angle = 0.26;
        let stall_angle = 0.35;

        let max_lift = aero.calculate_lift(velocity, max_lift_angle, &environment);

        let pre_stall_aoa = (max_lift_angle + stall_angle) / 2.0;
        let pre_stall_lift = aero.calculate_lift(velocity, pre_stall_aoa, &environment);

        let post_stall_aoa = stall_angle + 0.1;
        let post_stall_lift = aero.calculate_lift(velocity, post_stall_aoa, &environment);

        assert!(
            post_stall_lift.magnitude() < pre_stall_lift.magnitude(),
            "Lift should decrease after stall angle: Max lift: {}, Pre-stall: {}, Post-stall: {}",
            max_lift.magnitude(),
            pre_stall_lift.magnitude(),
            post_stall_lift.magnitude()
        );
    }
}
