use crate::control::environment::Environment;
use crate::control::mission::CelestialBody;
use crate::utils::vector2d::Vector2D;
use crate::GRAVITATIONAL_CONSTANT;

use super::aerodynamics::Aerodynamics;

#[derive(Debug)]
pub struct Kinematics {
    pub position: Vector2D,
    pub velocity: Vector2D,
    pub acceleration: Vector2D,
    pub orientation: f64,
    pub total_mass: f64,
}

impl Kinematics {
    pub fn new(launch_angle: f64, launch_site_position: Vector2D) -> Self {
        println!("Initial position: {:?}", launch_site_position);
        Kinematics {
            position: launch_site_position,
            velocity: Vector2D::new(0.0, 0.0),
            acceleration: Vector2D::new(0.0, 0.0),
            orientation: launch_angle.to_radians(),
            total_mass: 0.0, // This should be set after initialization
        }
    }

    pub fn update(
        &mut self,
        delta_time: f64,
        thrust_magnitude: f64,
        total_mass: f64,
        aerodynamics: &Aerodynamics,
        environment: &Environment,
        orientation_change: f64,
        celestial_bodies: &[CelestialBody],
    ) {
        self.total_mass = total_mass;
        self.apply_rotation(orientation_change, delta_time);

        let acceleration = |position: Vector2D, velocity: Vector2D| {
            let thrust_vector = Vector2D::new(
                thrust_magnitude * self.orientation.sin(),
                thrust_magnitude * self.orientation.cos(),
            );

            let gravitational_acceleration =
                Self::calculate_total_gravity(position, celestial_bodies);

            let aerodynamic_force = if environment.is_in_atmosphere(&position) {
                aerodynamics.calculate_aerodynamic_force(velocity, self.orientation, environment)
            } else {
                Vector2D::new(0.0, 0.0)
            };

            let net_force =
                thrust_vector + (gravitational_acceleration * total_mass) + aerodynamic_force;
            net_force / total_mass
        };

        let (new_position, new_velocity) = self.rk4_step(delta_time, acceleration);

        self.position = new_position;
        self.velocity = new_velocity;
        self.acceleration = acceleration(self.position, self.velocity);
    }

    pub fn set_mass(&mut self, new_mass: f64) {
        self.total_mass = new_mass;
    }

    fn rk4_step(
        &self,
        dt: f64,
        acceleration: impl Fn(Vector2D, Vector2D) -> Vector2D,
    ) -> (Vector2D, Vector2D) {
        let k1v = acceleration(self.position, self.velocity);
        let k1p = self.velocity;

        let k2v = acceleration(
            self.position + k1p * (dt / 2.0),
            self.velocity + k1v * (dt / 2.0),
        );
        let k2p = self.velocity + k1v * (dt / 2.0);

        let k3v = acceleration(
            self.position + k2p * (dt / 2.0),
            self.velocity + k2v * (dt / 2.0),
        );
        let k3p = self.velocity + k2v * (dt / 2.0);

        let k4v = acceleration(self.position + k3p * dt, self.velocity + k3v * dt);
        let k4p = self.velocity + k3v * dt;

        let new_position = self.position + (k1p + 2.0 * k2p + 2.0 * k3p + k4p) * (dt / 6.0);
        let new_velocity = self.velocity + (k1v + 2.0 * k2v + 2.0 * k3v + k4v) * (dt / 6.0);

        (new_position, new_velocity)
    }

    fn calculate_total_gravity(position: Vector2D, celestial_bodies: &[CelestialBody]) -> Vector2D {
        celestial_bodies
            .iter()
            .map(|body| Self::calculate_gravity_from_body(position, body))
            .sum()
    }

    fn calculate_gravity_from_body(position: Vector2D, body: &CelestialBody) -> Vector2D {
        let r = body.position - position;
        let r_mag = r.magnitude();
        if r_mag < 1e-6 {
            return Vector2D::new(0.0, 0.0);
        }
        let g = GRAVITATIONAL_CONSTANT * body.mass / r_mag.powi(2);
        -r.normalize() * g
    }

    pub fn get_altitude(&self, celestial_body: &CelestialBody) -> f64 {
        self.position.y - celestial_body.radius
    }

    pub fn get_velocity_magnitude(&self) -> f64 {
        self.velocity.magnitude()
    }

    pub fn get_acceleration_magnitude(&self) -> f64 {
        self.acceleration.magnitude()
    }

    pub fn get_orientation(&self) -> f64 {
        self.orientation
    }

    pub fn is_orbital_velocity_reached(&self, celestial_body: &CelestialBody) -> bool {
        let orbital_velocity = self.calculate_orbital_velocity(celestial_body);
        self.get_velocity_magnitude() >= orbital_velocity
    }

    pub fn calculate_orbital_velocity(&self, celestial_body: &CelestialBody) -> f64 {
        let altitude = self.get_altitude(celestial_body);
        let radius = celestial_body.radius + altitude;
        let gravity = celestial_body.gravity_at_altitude(altitude);

        (gravity * radius).sqrt()
    }

    pub fn apply_thrust(&mut self, thrust: f64, delta_time: f64) {
        let thrust_vector = Vector2D::new(
            thrust * self.orientation.cos(),
            thrust * self.orientation.sin(),
        );
        let acceleration = thrust_vector / self.total_mass;
        self.velocity = self.velocity + acceleration * delta_time;
        self.position = self.position + self.velocity * delta_time;
    }

    pub fn apply_orientation_change(&mut self, orientation_change: f64) {
        self.orientation += orientation_change;
        self.orientation %= 2.0 * std::f64::consts::PI;
        if self.orientation < 0.0 {
            self.orientation += 2.0 * std::f64::consts::PI;
        }
    }

    fn apply_rotation(&mut self, orientation_change: f64, delta_time: f64) {
        let max_rotation_rate = 0.1; // radians per second
        let clamped_change = orientation_change.clamp(
            -max_rotation_rate * delta_time,
            max_rotation_rate * delta_time,
        );
        self.orientation += clamped_change;
        self.orientation %= 2.0 * std::f64::consts::PI;
        if self.orientation < 0.0 {
            self.orientation += 2.0 * std::f64::consts::PI;
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    fn create_earth() -> CelestialBody {
        CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        )
    }

    fn create_mock_environment() -> Environment {
        Environment::new(create_earth())
    }

    fn create_mock_aerodynamics() -> Aerodynamics {
        Aerodynamics::new(0.5, 10.0, 0.3)
    }

    #[test]
    fn test_kinematics_initialization() {
        let launch_angle = 90.0; // Straight up
        let earth = create_earth();
        let launch_site = Vector2D::new(0.0, earth.radius);
        let kinematics = Kinematics::new(launch_angle, launch_site);

        assert_relative_eq!(kinematics.position.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(kinematics.position.y, earth.radius, epsilon = 1e-6);
        assert_relative_eq!(kinematics.velocity.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(kinematics.velocity.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(kinematics.acceleration.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(kinematics.acceleration.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(kinematics.orientation, PI / 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_kinematics_update_vertical_launch() {
        let mut kinematics = Kinematics::new(90.0, Vector2D::new(0.0, 6_371_000.0));
        let earth = create_earth();
        let environment = create_mock_environment();
        let aerodynamics = create_mock_aerodynamics();

        let thrust = 1_000_000.0; // 1 MN thrust
        let total_mass = 100_000.0; // 100 tonnes
        let delta_time = 1.0; // 1 second

        kinematics.update(
            delta_time,
            thrust,
            total_mass,
            &aerodynamics,
            &environment,
            0.0,
            &[earth],
        );

        // The rocket should move upwards
        assert!(
            kinematics.position.y > 6_371_000.0,
            "Rocket should move upwards"
        );
        assert!(
            kinematics.velocity.y > 0.0,
            "Rocket should have upward velocity"
        );
    }

    #[test]
    fn test_kinematics_update_angled_launch() {
        let mut kinematics = Kinematics::new(80.0, Vector2D::new(0.0, 6_371_000.0));
        let earth = create_earth();
        let environment = create_mock_environment();
        let aerodynamics = create_mock_aerodynamics();

        let thrust = 1_000_000.0;
        let total_mass = 100_000.0;
        let delta_time = 1.0;

        kinematics.update(
            delta_time,
            thrust,
            total_mass,
            &aerodynamics,
            &environment,
            0.0,
            &[earth],
        );

        // The rocket should move upwards and slightly to the side
        assert!(
            kinematics.position.y > 6_371_000.0,
            "Rocket should move upwards"
        );
        assert!(kinematics.position.x > 0.0, "Rocket should move sideways");
        assert!(
            kinematics.velocity.y > 0.0,
            "Rocket should have upward velocity"
        );
        assert!(
            kinematics.velocity.x > 0.0,
            "Rocket should have sideways velocity"
        );
    }

    #[test]
    fn test_get_altitude() {
        let earth = create_earth();
        let kinematics = Kinematics::new(90.0, Vector2D::new(0.0, earth.radius + 1000.0));

        let altitude = kinematics.get_altitude(&earth);
        assert_relative_eq!(altitude, 1000.0, epsilon = 1e-6);
    }

    #[test]
    fn test_calculate_orbital_velocity() {
        let earth = create_earth();
        let kinematics = Kinematics::new(0.0, Vector2D::new(0.0, earth.radius + 200_000.0));

        let orbital_velocity = kinematics.calculate_orbital_velocity(&earth);

        // Expected orbital velocity at 200 km altitude
        let expected_velocity = 7784.0; // m/s (approximate)
        assert_relative_eq!(orbital_velocity, expected_velocity, epsilon = 10.0);
    }

    #[test]
    fn test_apply_rotation() {
        let mut kinematics = Kinematics::new(0.0, Vector2D::new(0.0, 6_371_000.0));
        let delta_time = 1.0;
        let rotation_change = 0.2; // radians

        kinematics.apply_rotation(rotation_change, delta_time);

        assert_relative_eq!(kinematics.orientation, 0.1, epsilon = 1e-6);
    }

    #[test]
    fn test_gravity_calculation() {
        let earth = create_earth();
        let position = Vector2D::new(0.0, earth.radius);
        let gravity = Kinematics::calculate_gravity_from_body(position, &earth);

        let expected_gravity = Vector2D::new(0.0, 9.81);
        assert_relative_eq!(gravity.x, expected_gravity.x, epsilon = 0.01);
        assert_relative_eq!(gravity.y, expected_gravity.y, epsilon = 0.01);
    }
}
