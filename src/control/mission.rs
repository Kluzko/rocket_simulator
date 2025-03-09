use crate::constants::{EARTH_RADIUS, GRAVITATIONAL_CONSTANT};
use crate::utils::vector2d::Vector2D;

#[derive(Clone, Debug)]
pub struct CelestialBody {
    pub name: String,
    pub position: Vector2D,
    pub radius: f64,
    pub mass: f64,
}

impl CelestialBody {
    pub fn new(name: String, position: Vector2D, radius: f64, mass: f64) -> Self {
        CelestialBody {
            name,
            position,
            radius,
            mass,
        }
    }

    pub fn atmosphere_height(&self) -> f64 {
        match self.name.as_str() {
            "Earth" => 100_000.0, // Approximate Kármán line
            // Add other celestial bodies as needed
            _ => 0.0, // Assume no atmosphere for unknown bodies
        }
    }

    pub fn surface_gravity(&self) -> f64 {
        GRAVITATIONAL_CONSTANT * self.mass / self.radius.powi(2)
    }

    pub fn gravity_at_altitude(&self, altitude: f64) -> f64 {
        let distance = self.radius + altitude;
        GRAVITATIONAL_CONSTANT * self.mass / distance.powi(2)
    }

    pub fn escape_velocity(&self, altitude: f64) -> f64 {
        let distance = self.radius + altitude;
        (2.0 * GRAVITATIONAL_CONSTANT * self.mass / distance).sqrt()
    }
}

#[derive(Clone, Debug)]
pub struct Mission {
    pub name: String,
    pub start_body: CelestialBody,
    pub target_body: CelestialBody,
    pub target_orbit_altitude: Option<f64>,
    pub return_trip: bool,
    pub total_fuel: f64,
}

pub struct MissionFactory;

impl MissionFactory {
    pub fn create_earth_orbit(name: String, target_altitude: f64, total_fuel: f64) -> Mission {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            EARTH_RADIUS,
            5.97e24,
        );

        Mission {
            name,
            start_body: earth.clone(),
            target_body: earth,
            target_orbit_altitude: Some(target_altitude),
            return_trip: false,
            total_fuel,
        }
    }

    pub fn create_interplanetary(
        name: String,
        target_body: CelestialBody,
        target_orbit_altitude: Option<f64>,
        return_trip: bool,
        total_fuel: f64,
    ) -> Mission {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            EARTH_RADIUS,
            5.97e24,
        );

        Mission {
            name,
            start_body: earth,
            target_body,
            target_orbit_altitude,
            return_trip,
            total_fuel,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::EARTH_RADIUS;
    use crate::utils::vector2d::Vector2D;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_earth_orbit_mission() {
        // Create an Earth orbit mission with 200 km target altitude and 50000 kg fuel
        let mission =
            MissionFactory::create_earth_orbit("Earth Orbit".to_string(), 200_000.0, 50_000.0);

        // Check that the mission name and details are correct
        assert_eq!(mission.name, "Earth Orbit");
        assert_eq!(mission.start_body.name, "Earth");
        assert_eq!(mission.start_body.radius, EARTH_RADIUS);
        assert_eq!(mission.start_body.mass, 5.97e24);
        assert_eq!(mission.target_orbit_altitude, Some(200_000.0));
        assert_eq!(mission.total_fuel, 50_000.0);
        assert_eq!(mission.return_trip, false);

        // Check the surface gravity of Earth
        let surface_gravity = mission.start_body.surface_gravity();
        assert_abs_diff_eq!(surface_gravity, 9.81, epsilon = 1e-2);

        // Check gravity at the target altitude of 200 km
        let gravity_at_200km = mission.start_body.gravity_at_altitude(200_000.0);
        assert_abs_diff_eq!(gravity_at_200km, 9.23, epsilon = 1e-2);

        // Check escape velocity at the target altitude of 200 km
        let escape_velocity = mission.start_body.escape_velocity(200_000.0);
        assert_abs_diff_eq!(escape_velocity, 11_000.0, epsilon = 500.0);
    }

    #[test]
    fn test_interplanetary_mission() {
        // Create a fictional planet for the interplanetary mission
        let mars = CelestialBody::new(
            "Mars".to_string(),
            Vector2D::new(0.0, 0.0),
            3_389_500.0, // Radius of Mars in meters
            6.39e23,     // Mass of Mars in kg
        );

        // Create an interplanetary mission to Mars with 300 km target orbit and return trip
        let mission = MissionFactory::create_interplanetary(
            "Mars Mission".to_string(),
            mars.clone(),
            Some(300_000.0),
            true,
            150_000.0,
        );

        // Check that the mission name and details are correct
        assert_eq!(mission.name, "Mars Mission");
        assert_eq!(mission.start_body.name, "Earth");
        assert_eq!(mission.target_body.name, "Mars");
        assert_eq!(mission.target_body.radius, 3_389_500.0);
        assert_eq!(mission.target_body.mass, 6.39e23);
        assert_eq!(mission.target_orbit_altitude, Some(300_000.0));
        assert_eq!(mission.total_fuel, 150_000.0);
        assert_eq!(mission.return_trip, true);

        // Check the surface gravity of Mars
        let surface_gravity = mission.target_body.surface_gravity();
        assert_abs_diff_eq!(surface_gravity, 3.71, epsilon = 1e-2);

        // Check gravity at the target altitude of 300 km
        let gravity_at_300km = mission.target_body.gravity_at_altitude(300_000.0);
        assert_abs_diff_eq!(gravity_at_300km, 3.13, epsilon = 1e-2);

        // Check escape velocity at the target altitude of 300 km
        let escape_velocity = mission.target_body.escape_velocity(300_000.0);
        assert_abs_diff_eq!(escape_velocity, 5_000.0, epsilon = 300.0);
    }
}
