use crate::constants::{
    AIR_DENSITY_SEA_LEVEL, SEA_LEVEL_TEMPERATURE, TROPOSPHERE_HEIGHT, TROPOSPHERE_TEMP_GRADIENT,
};
use crate::control::mission::CelestialBody;
use crate::utils::vector2d::Vector2D;

pub struct Environment {
    pub air_density: f64,
    pub temperature: f64,
    pub pressure: f64,
    pub gravity: f64,
    pub current_body: CelestialBody,
}

impl Environment {
    pub fn new(start_body: CelestialBody) -> Self {
        Environment {
            air_density: AIR_DENSITY_SEA_LEVEL,
            temperature: SEA_LEVEL_TEMPERATURE,
            pressure: 101325.0, // sea level pressure (Pa)
            gravity: start_body.surface_gravity(),
            current_body: start_body,
        }
    }

    pub fn update(&mut self, position: &Vector2D, celestial_bodies: &[CelestialBody]) {
        self.current_body = self.determine_current_body(position, celestial_bodies);
        let altitude = self.calculate_altitude(position);

        if self.is_in_atmosphere(position) {
            self.update_atmospheric_conditions(altitude);
        } else {
            self.update_space_conditions();
        }

        self.gravity = self.current_body.gravity_at_altitude(altitude);
    }

    pub fn is_in_atmosphere(&self, position: &Vector2D) -> bool {
        let altitude = self.calculate_altitude(position);
        altitude < self.current_body.atmosphere_height()
    }

    fn determine_current_body(
        &self,
        position: &Vector2D,
        celestial_bodies: &[CelestialBody],
    ) -> CelestialBody {
        celestial_bodies
            .iter()
            .min_by_key(|body| (body.position - *position).magnitude() as i64)
            .unwrap()
            .clone()
    }

    fn calculate_altitude(&self, position: &Vector2D) -> f64 {
        (*position - self.current_body.position).magnitude() - self.current_body.radius
    }

    fn update_atmospheric_conditions(&mut self, altitude: f64) {
        if altitude < TROPOSPHERE_HEIGHT {
            // In the troposphere, apply standard lapse rate
            self.temperature = SEA_LEVEL_TEMPERATURE + TROPOSPHERE_TEMP_GRADIENT * altitude;
            self.pressure = 101325.0 * (self.temperature / SEA_LEVEL_TEMPERATURE).powf(5.255);
        } else if altitude < 47_000.0 {
            // Mesosphere (between 11 km to ~47 km), temperature decreases gradually
            self.temperature = 216.65 + (-2.8 / 1_000.0) * (altitude - TROPOSPHERE_HEIGHT); // Rough approximation
            self.pressure = 22632.0 * (-0.000157 * (altitude - TROPOSPHERE_HEIGHT)).exp();
        } else if altitude < 80_000.0 {
            // For altitudes between 47 km to 80 km, temperature drops to 197.65 K
            self.temperature = 197.65;
            self.pressure = 5474.89 * (-0.000157 * (altitude - 47_000.0)).exp();
        // Adjusted pressure
        } else {
            // Beyond 80 km, near space
            self.update_space_conditions();
        }

        // Calculate air density based on the pressure and temperature
        if self.pressure > 0.0 && self.temperature > 0.0 {
            self.air_density = self.pressure / (287.05 * self.temperature);
        } else {
            self.air_density = 0.0;
        }
    }

    fn update_space_conditions(&mut self) {
        self.air_density = 0.0;
        self.temperature = 2.7; // Cosmic microwave background temperature
        self.pressure = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_update_atmospheric_conditions_sea_level() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mut environment = Environment::new(earth.clone());

        // Simulate the rocket at sea level (altitude = 0)
        let position = Vector2D::new(0.0, earth.radius);
        environment.update(&position, &[environment.current_body.clone()]);

        // Check the atmospheric conditions at sea level
        assert_abs_diff_eq!(environment.temperature, 288.15, epsilon = 0.1); // 15°C
        assert_abs_diff_eq!(environment.pressure, 101_325.0, epsilon = 1.0); // Pa
        assert_abs_diff_eq!(environment.air_density, 1.225, epsilon = 0.01); // kg/m³
    }

    #[test]
    fn test_update_atmospheric_conditions_tropopause() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mut environment = Environment::new(earth.clone());

        // Simulate the rocket at the tropopause (~11 km altitude)
        let position = Vector2D::new(0.0, earth.radius + 11_000.0); // 11 km altitude
        environment.update(&position, &[environment.current_body.clone()]);

        // Check the atmospheric conditions at the tropopause
        assert_abs_diff_eq!(environment.temperature, 216.65, epsilon = 0.1); // Tropopause temp in K
        assert_abs_diff_eq!(environment.pressure, 22_632.0, epsilon = 1.0); // Pa
        assert_abs_diff_eq!(environment.air_density, 0.3639, epsilon = 0.01); // kg/m³
    }

    #[test]
    fn test_space_conditions() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mut environment = Environment::new(earth.clone());

        // Simulate the rocket in space (500 km altitude)
        let position = Vector2D::new(0.0, earth.radius + 500_000.0);
        environment.update(&position, &[environment.current_body.clone()]);

        // Check space conditions
        assert_abs_diff_eq!(environment.temperature, 2.7, epsilon = 0.1); // Cosmic background temp
        assert_abs_diff_eq!(environment.pressure, 0.0, epsilon = 1e-10); // Zero pressure
        assert_abs_diff_eq!(environment.air_density, 0.0, epsilon = 1e-10); // Zero density
    }

    #[test]
    fn test_mesosphere_conditions() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mut environment = Environment::new(earth.clone());

        // Simulate the rocket in the mesosphere (30 km altitude)
        let position = Vector2D::new(0.0, earth.radius + 30_000.0);
        environment.update(&position, &[environment.current_body.clone()]);

        // Verify temperature follows the expected gradient
        let expected_temp = 216.65 + (-2.8 / 1_000.0) * (30_000.0 - TROPOSPHERE_HEIGHT);
        assert_abs_diff_eq!(environment.temperature, expected_temp, epsilon = 0.1);

        // Verify pressure and density are positive but less than at tropopause
        assert!(environment.pressure > 0.0 && environment.pressure < 22_632.0);
        assert!(environment.air_density > 0.0 && environment.air_density < 0.3639);
    }

    #[test]
    fn test_gravity_variation_with_altitude() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mut environment = Environment::new(earth.clone());

        // Get gravity at sea level
        let position_sea_level = Vector2D::new(0.0, earth.radius);
        environment.update(&position_sea_level, &[environment.current_body.clone()]);
        let gravity_sea_level = environment.gravity;

        // Get gravity at 100 km
        let position_100km = Vector2D::new(0.0, earth.radius + 100_000.0);
        environment.update(&position_100km, &[environment.current_body.clone()]);
        let gravity_100km = environment.gravity;

        // Gravity should decrease with altitude
        assert!(gravity_100km < gravity_sea_level);

        // Check that it follows the inverse square law approximately
        let expected_ratio = (earth.radius / (earth.radius + 100_000.0)).powi(2);
        let actual_ratio = gravity_100km / gravity_sea_level;
        assert_abs_diff_eq!(actual_ratio, expected_ratio, epsilon = 0.01);
    }

    #[test]
    fn test_multiple_celestial_bodies() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );

        let moon = CelestialBody::new(
            "Moon".to_string(),
            Vector2D::new(384_400_000.0, 0.0), // ~384,400 km from Earth
            1_737_000.0,                       // Moon radius
            7.342e22,                          // Moon mass
        );

        let mut environment = Environment::new(earth.clone());

        // Position near Earth
        let position_near_earth = Vector2D::new(1_000_000.0, 0.0); // 1000 km from Earth center
        environment.update(&position_near_earth, &[earth.clone(), moon.clone()]);

        // Should determine Earth as current body
        assert_eq!(environment.current_body.name, "Earth");

        // Position near Moon
        let position_near_moon = Vector2D::new(384_000_000.0, 0.0); // Close to the moon
        environment.update(&position_near_moon, &[earth.clone(), moon.clone()]);

        // Should determine Moon as current body
        assert_eq!(environment.current_body.name, "Moon");
    }
}
