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
            self.pressure = 5474.89 * (-0.000157 * (altitude - 47_000.0)).exp();  // Adjusted pressure
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
}
