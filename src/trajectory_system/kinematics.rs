use crate::{
    control::{environment::Environment, rocket::RocketState},
    utils::vector2d::Vector2D,
};

use super::aerodynamics::Aerodynamics;

#[derive(Debug)]
pub struct Kinematics {
    pub position: Vector2D,
    pub velocity: Vector2D,
    pub acceleration: Vector2D,
    pub orientation: f64,
    pub launch_angle: f64,
    pub launch_site_position: Vector2D,
    pub time: f64,
}

#[allow(dead_code)]
impl Kinematics {
    pub fn new(launch_angle: f64, launch_site_position: Vector2D) -> Self {
        let launch_angle_rad = launch_angle.to_radians();
        Kinematics {
            position: launch_site_position,
            velocity: Vector2D::new(0.0, 0.0),
            acceleration: Vector2D::new(0.0, 0.0),
            orientation: launch_angle_rad,
            launch_angle: launch_angle_rad,
            launch_site_position,
            time: 0.0,
        }
    }

    pub fn update(
        &mut self,
        delta_time: f64,
        thrust_magnitude: f64,
        total_mass: f64,
        aerodynamics: &Aerodynamics,
        environment: &mut Environment,
        orientation_change: f64,
        rocket_state: &RocketState,
    ) {
        environment.update(self.position.y);
        self.apply_rotation(orientation_change);

        let initial_state = (self.position, self.velocity);
        let k1 = self.calculate_derivatives(
            initial_state,
            thrust_magnitude,
            total_mass,
            aerodynamics,
            environment,
        );
        let k2 = self.calculate_derivatives(
            (
                initial_state.0 + k1.0 * (delta_time / 2.0),
                initial_state.1 + k1.1 * (delta_time / 2.0),
            ),
            thrust_magnitude,
            total_mass,
            aerodynamics,
            environment,
        );
        let k3 = self.calculate_derivatives(
            (
                initial_state.0 + k2.0 * (delta_time / 2.0),
                initial_state.1 + k2.1 * (delta_time / 2.0),
            ),
            thrust_magnitude,
            total_mass,
            aerodynamics,
            environment,
        );
        let k4 = self.calculate_derivatives(
            (
                initial_state.0 + k3.0 * delta_time,
                initial_state.1 + k3.1 * delta_time,
            ),
            thrust_magnitude,
            total_mass,
            aerodynamics,
            environment,
        );

        self.position =
            initial_state.0 + (delta_time / 6.0) * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
        self.velocity =
            initial_state.1 + (delta_time / 6.0) * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);

        // Update acceleration for the final state
        self.acceleration = self.calculate_acceleration(
            self.velocity,
            thrust_magnitude,
            total_mass,
            aerodynamics,
            environment,
        );

        if matches!(rocket_state, RocketState::Returning) {
            let ground_threshold = self.launch_site_position.y + 0.1; // 10 cm above ground
            if self.position.y < ground_threshold && self.velocity.y < 0.0 {
                self.position.y = self.launch_site_position.y;
                self.velocity.y = 0.0;
                println!("Ground collision detected! Exiting simulation.");
                std::process::exit(0); // Exit after ground collision is detected
            }
        }

        // Update time
        self.time += delta_time;

        // Assertions
        assert!(
            self.velocity.magnitude() < 10000.0,
            "Velocity exceeds reasonable limits"
        );
        assert!(total_mass > 0.0, "Total mass must be positive");
        assert!(
            self.acceleration.magnitude() < 100.0 * environment.gravity,
            "Acceleration exceeds reasonable limits"
        );
    }

    fn apply_rotation(&mut self, orientation_change: f64) {
        self.orientation += orientation_change;
        self.orientation = self.orientation % (2.0 * std::f64::consts::PI);
        if self.orientation < 0.0 {
            self.orientation += 2.0 * std::f64::consts::PI;
        }
    }

    fn calculate_derivatives(
        &self,
        state: (Vector2D, Vector2D),
        thrust_magnitude: f64,
        total_mass: f64,
        aerodynamics: &Aerodynamics,
        environment: &Environment,
    ) -> (Vector2D, Vector2D) {
        let (_position, velocity) = state;
        let acceleration = self.calculate_acceleration(
            velocity,
            thrust_magnitude,
            total_mass,
            aerodynamics,
            environment,
        );
        (velocity, acceleration)
    }

    fn calculate_acceleration(
        &self,
        velocity: Vector2D,
        thrust_magnitude: f64,
        total_mass: f64,
        aerodynamics: &Aerodynamics,
        environment: &Environment,
    ) -> Vector2D {
        let thrust_vector = Vector2D::new(
            thrust_magnitude * self.get_orientation().cos(),
            thrust_magnitude * self.get_orientation().sin(),
        );

        // Calculate angle of attack
        let velocity_angle = if velocity.magnitude() > 1e-6 {
            velocity.angle()
        } else {
            self.get_orientation()
        };
        let angle_of_attack = self.get_orientation() - velocity_angle;

        let gravity_vector = Vector2D::new(0.0, -environment.gravity * total_mass);
        let aerodynamic_force =
            aerodynamics.calculate_aerodynamic_force(velocity, angle_of_attack, environment);

        let net_force = thrust_vector + gravity_vector + aerodynamic_force;
        let acceleration = net_force / total_mass;

        acceleration
    }

    pub fn get_orientation(&self) -> f64 {
        self.orientation
    }

    pub fn get_flight_path(&self) -> Vector2D {
        self.position - self.launch_site_position
    }

    pub fn get_launch_angle(&self) -> f64 {
        self.launch_angle
    }

    pub fn get_launch_site_position(&self) -> Vector2D {
        self.launch_site_position
    }

    pub fn get_time(&self) -> f64 {
        self.time
    }

    pub fn get_altitude(&self) -> f64 {
        self.position.y - self.launch_site_position.y
    }

    pub fn get_ground_distance(&self) -> f64 {
        self.get_flight_path().x
    }

    pub fn get_velocity_magnitude(&self) -> f64 {
        self.velocity.magnitude()
    }

    pub fn get_acceleration_magnitude(&self) -> f64 {
        self.acceleration.magnitude()
    }

    pub fn get_flight_angle(&self) -> f64 {
        // Use launch angle if velocity is zero
        if self.velocity.magnitude() < 1e-6 {
            self.launch_angle
        } else {
            self.velocity.y.atan2(self.velocity.x)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    // FIX LATER ROCKET STATE NOW FOR SIMPILICTY WE ADD EVRYWHERE LAUNCHED

    fn create_test_aerodynamics() -> Aerodynamics {
        Aerodynamics::new(0.5, 10.0, 0.1)
    }

    #[test]
    fn test_kinematics_initial_state() {
        let kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 100.0));
        assert_eq!(kinematics.position, Vector2D::new(0.0, 100.0));
        assert_eq!(kinematics.velocity, Vector2D::new(0.0, 0.0));
        assert_eq!(kinematics.acceleration, Vector2D::new(0.0, 0.0));
    }

    #[test]
    fn test_kinematics_gravity_effect() {
        let mut kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 0.0));
        let mut environment = Environment::new();
        let aerodynamics = create_test_aerodynamics();
        let delta_time = 0.1;
        let mass = 1000.0;

        // Set initial position to 100 meters above launch site
        kinematics.position.y = 100.0;

        // Apply gravity for one step
        kinematics.update(
            delta_time,
            0.0,
            mass,
            &aerodynamics,
            &mut environment,
            0.0,
            &RocketState::Launched,
        );

        println!(
            "After first update: velocity = {}, altitude = {}",
            kinematics.velocity.y,
            kinematics.get_altitude()
        );

        assert!(
            kinematics.velocity.y < -0.9,
            "Rocket should start falling. Velocity y: {}",
            kinematics.velocity.y
        );
        assert!(
            kinematics.get_altitude() < 100.0 && kinematics.get_altitude() > 99.9,
            "Rocket should lose a small amount of altitude. Altitude: {}",
            kinematics.get_altitude()
        );

        // Continue simulation for a few more steps
        for i in 0..9 {
            kinematics.update(
                delta_time,
                0.0,
                mass,
                &aerodynamics,
                &mut environment,
                0.0,
                &RocketState::Launched,
            );
            println!(
                "Step {}: velocity = {}, altitude = {}",
                i + 2,
                kinematics.velocity.y,
                kinematics.get_altitude()
            );
        }

        assert!(
            kinematics.velocity.y < -9.5,
            "Rocket should be falling faster. Velocity y: {}",
            kinematics.velocity.y
        );
        assert!(
            kinematics.get_altitude() < 96.0 && kinematics.get_altitude() > 95.0,
            "Rocket should have lost significant altitude. Altitude: {}",
            kinematics.get_altitude()
        );
    }

    #[test]
    fn test_kinematics_with_thrust() {
        let mut kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 0.0));
        let mut environment = Environment::new();
        let aerodynamics = create_test_aerodynamics();
        let delta_time = 0.1;
        let mass = 1000.0;
        let thrust = 15000.0; // Thrust greater than gravity * mass

        for _ in 0..10 {
            kinematics.update(
                delta_time,
                thrust,
                mass,
                &aerodynamics,
                &mut environment,
                0.0,
                &RocketState::Launched,
            );
        }

        assert!(
            kinematics.position.y > 0.0,
            "Rocket should gain altitude with thrust. Position y: {}",
            kinematics.position.y
        );
        assert!(
            kinematics.velocity.y > 0.0,
            "Rocket should have positive vertical velocity. Velocity y: {}",
            kinematics.velocity.y
        );
    }

    #[test]
    fn test_kinematics_angled_launch() {
        let launch_angle = 60.0; // 60 degree launch angle
        let mut kinematics = Kinematics::new(launch_angle, Vector2D::new(0.0, 0.0));
        let mut environment = Environment::new();
        let aerodynamics = create_test_aerodynamics();
        let delta_time = 0.1;
        let mass = 1000.0;
        let thrust = 15000.0;

        for _ in 0..10 {
            kinematics.update(
                delta_time,
                thrust,
                mass,
                &aerodynamics,
                &mut environment,
                0.0,
                &RocketState::Launched,
            );
        }

        assert!(
            kinematics.position.y > 0.0,
            "Rocket should gain altitude. Position y: {}",
            kinematics.position.y
        );
        assert!(
            kinematics.position.x > 0.0,
            "Rocket should have horizontal displacement. Position x: {}",
            kinematics.position.x
        );
        assert!(
            kinematics.velocity.y > 0.0,
            "Rocket should have positive vertical velocity. Velocity y: {}",
            kinematics.velocity.y
        );
        assert!(
            kinematics.velocity.x > 0.0,
            "Rocket should have positive horizontal velocity. Velocity x: {}",
            kinematics.velocity.x
        );
    }

    #[test]
    fn test_kinematics_pitch_over() {
        let mut kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 0.0));
        let mut environment = Environment::new();
        let aerodynamics = create_test_aerodynamics();
        let delta_time = 0.1;
        let mass = 1000.0;
        let thrust = 15000.0;
        let orientation_change = -0.01;

        // Simulate for 10 seconds (100 steps)
        for _ in 0..100 {
            kinematics.update(
                delta_time,
                thrust,
                mass,
                &aerodynamics,
                &mut environment,
                orientation_change,
                &RocketState::Launched,
            );
        }

        // Check if the rocket has gained both vertical and horizontal velocity
        assert!(
            kinematics.velocity.y > 0.0,
            "Rocket should have positive vertical velocity. Velocity y: {}",
            kinematics.velocity.y
        );
        assert!(
            kinematics.velocity.x > 0.0,
            "Rocket should have positive horizontal velocity due to pitch over. Velocity x: {}",
            kinematics.velocity.x
        );

        // Check if the rocket has moved both vertically and horizontally
        assert!(
            kinematics.position.y > 0.0,
            "Rocket should gain altitude. Position y: {}",
            kinematics.position.y
        );
        assert!(
            kinematics.position.x > 0.0,
            "Rocket should have horizontal displacement due to pitch over. Position x: {}",
            kinematics.position.x
        );

        assert!(
            kinematics.orientation < PI / 2.0,
            "Rocket orientation should have decreased due to pitch over. Orientation: {}",
            kinematics.orientation
        );
    }

    #[test]
    fn test_kinematics_thrust_cutoff() {
        let mut kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 0.0));
        let mut environment = Environment::new();
        let aerodynamics = create_test_aerodynamics();
        let delta_time = 0.1;
        let mass = 1000.0;
        let thrust = 15000.0;

        // Apply thrust for 5 steps
        for _ in 0..5 {
            kinematics.update(
                delta_time,
                thrust,
                mass,
                &aerodynamics,
                &mut environment,
                0.0,
                &RocketState::Launched,
            );
        }

        let peak_altitude = kinematics.position.y;
        let peak_velocity = kinematics.velocity.y;

        println!("Peak altitude: {}", peak_altitude);
        println!("Peak velocity: {}", peak_velocity);

        for i in 0..10 {
            kinematics.update(
                delta_time,
                0.0,
                mass,
                &aerodynamics,
                &mut environment,
                0.0,
                &RocketState::Launched,
            );
            println!(
                "Step {}: altitude = {}, velocity = {}",
                i + 1,
                kinematics.position.y,
                kinematics.velocity.y
            );
        }

        assert!(
            kinematics.position.y < peak_altitude,
            "Rocket should fall after thrust cutoff. Current altitude: {}, Peak altitude: {}",
            kinematics.position.y,
            peak_altitude
        );
        assert!(
               kinematics.velocity.y < peak_velocity,
               "Rocket's vertical velocity should decrease after thrust cutoff. Current velocity: {}, Peak velocity: {}",
               kinematics.velocity.y,
               peak_velocity
           );
    }

    #[test]
    fn test_kinematics_thrust_cutoff_with_environment() {
        let mut kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 0.0));
        let mut environment = Environment::new();
        let aerodynamics = create_test_aerodynamics();
        let delta_time = 0.1;
        let mass = 1000.0;
        let thrust = 15000.0;

        // Apply thrust for 50 steps
        for _ in 0..50 {
            kinematics.update(
                delta_time,
                thrust,
                mass,
                &aerodynamics,
                &mut environment,
                0.0,
                &RocketState::Launched,
            );
        }

        let peak_velocity = kinematics.velocity.y;
        let initial_position = kinematics.position.y;

        println!("Initial position after thrust: {}", initial_position);
        println!("Peak velocity: {}", peak_velocity);

        // Continue without thrust for 100 more steps
        for i in 0..100 {
            kinematics.update(
                delta_time,
                0.0,
                mass,
                &aerodynamics,
                &mut environment,
                0.0,
                &RocketState::Launched,
            );
            if i % 10 == 0 {
                println!(
                    "Step {}: altitude = {}, velocity = {}, gravity = {}",
                    i + 1,
                    kinematics.position.y,
                    kinematics.velocity.y,
                    environment.gravity
                );
            }
        }

        assert!(
                kinematics.velocity.y < peak_velocity,
                "Rocket's vertical velocity should decrease. Initial velocity: {}, Current velocity: {}",
                peak_velocity,
                kinematics.velocity.y
            );

        // The rocket may still be gaining altitude due to its momentum,
        // but its rate of ascent should be decreasing
        let altitude_gain_rate = (kinematics.position.y - initial_position) / (100.0 * delta_time);
        assert!(
            altitude_gain_rate < peak_velocity,
            "Rate of altitude gain should decrease. Initial velocity: {}, Current rate: {}",
            peak_velocity,
            altitude_gain_rate
        );
    }

    #[test]
    fn test_kinematics_orientation_change() {
        let mut kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 0.0));
        let mut environment = Environment::new();
        let aerodynamics = create_test_aerodynamics();
        let delta_time = 0.1;
        let mass = 1000.0;
        let thrust = 15000.0;
        let orientation_change = 0.1;

        let initial_orientation = kinematics.orientation;

        // Apply 10 updates with orientation change
        for _ in 0..10 {
            kinematics.update(
                delta_time,
                thrust,
                mass,
                &aerodynamics,
                &mut environment,
                orientation_change,
                &RocketState::Launched,
            );
        }

        assert!(
            kinematics.orientation > initial_orientation,
            "Rocket orientation should have increased. Initial: {}, Current: {}",
            initial_orientation,
            kinematics.orientation
        );

        assert_relative_eq!(
            kinematics.orientation,
            initial_orientation + 10.0 * orientation_change,
            epsilon = 1e-6
        );

        // Verify that orientation stays within [0, 2π]
        assert!(kinematics.orientation >= 0.0 && kinematics.orientation < 2.0 * PI);
    }

    #[test]
    fn test_gravity_decreases_with_altitude_during_flight() {
        let mut environment = Environment::new();
        let mut kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 0.0));
        let aerodynamics = create_test_aerodynamics();
        let delta_time = 0.1;
        let mass = 1000.0;
        let thrust = 0.0;

        // Initial gravity at sea level
        kinematics.update(
            delta_time,
            thrust,
            mass,
            &aerodynamics,
            &mut environment,
            0.0,
            &RocketState::Launched,
        );
        let sea_level_gravity = environment.gravity;

        // Simulate rocket reaching 10,000 meters
        kinematics.position.y = 10_000.0;
        kinematics.update(
            delta_time,
            thrust,
            mass,
            &aerodynamics,
            &mut environment,
            0.0,
            &RocketState::Launched,
        );
        let high_altitude_gravity = environment.gravity;

        // Ensure that gravity decreases as altitude increases
        assert!(
            high_altitude_gravity < sea_level_gravity,
            "Gravity should decrease at higher altitudes"
        );
    }
}
