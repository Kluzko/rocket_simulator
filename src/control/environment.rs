use crate::constants::{
    AIR_DENSITY_SEA_LEVEL, EARTH_RADIUS, GRAVITY, SEA_LEVEL_TEMPERATURE, TROPOSPHERE_HEIGHT,
    TROPOSPHERE_TEMP_GRADIENT,
};

#[derive(Debug)]
pub struct Environment {
    pub altitude: f64,    // y-position
    pub air_density: f64, // kg/m³
    pub temperature: f64, // Kelvin
    pub gravity: f64,     // m/s²
    pub wind_speed: f64,  // m/s (optional)
}

impl Environment {
    pub fn new() -> Self {
        Environment {
            altitude: 0.0,
            air_density: AIR_DENSITY_SEA_LEVEL,
            temperature: SEA_LEVEL_TEMPERATURE,
            gravity: GRAVITY,
            wind_speed: 0.0, // Start with no wind
        }
    }

    pub fn update(&mut self, altitude: f64) {
        self.altitude = altitude;
        self.gravity = self.calculate_gravity(altitude);
        self.air_density = self.calculate_air_density(altitude);
        self.temperature = self.calculate_temperature(altitude);
        self.wind_speed = self.calculate_wind_speed(); // Optional
    }

    fn calculate_gravity(&self, altitude: f64) -> f64 {
        GRAVITY * (EARTH_RADIUS / (EARTH_RADIUS + altitude)).powi(2)
    }

    fn calculate_air_density(&self, altitude: f64) -> f64 {
        // Constants
        const R: f64 = 287.05; // Specific gas constant for dry air (J/(kg·K))
        const P0: f64 = 101325.0; // Pressure at sea level (Pa)
        const T0: f64 = 288.15; // Temperature at sea level (K)
        const L: f64 = 0.0065; // Temperature lapse rate in the troposphere (K/m)
        const P_STRAT: f64 = 22632.1; // Pressure at the base of the stratosphere (Pa)
        const T_STRAT: f64 = 216.65; // Constant temperature in the stratosphere (K)

        if altitude <= TROPOSPHERE_HEIGHT {
            // Troposphere: Temperature decreases with altitude
            let temperature = T0 - L * altitude; // Calculate temperature at the current altitude
                                                 // Barometric formula for troposphere pressure
            let pressure = P0 * (1.0 - (L * altitude / T0)).powf(5.2561);
            // Air density in the troposphere using the ideal gas law
            pressure / (R * temperature)
        } else {
            // Stratosphere: Pressure decreases exponentially, temperature constant
            // Exponential decay of pressure in the stratosphere
            let pressure = P_STRAT * (-GRAVITY / (R * T_STRAT) * (altitude - 11000.0)).exp();
            // Air density in the stratosphere using the ideal gas law
            pressure / (R * T_STRAT)
        }
    }

    fn calculate_temperature(&self, altitude: f64) -> f64 {
        if altitude <= TROPOSPHERE_HEIGHT {
            SEA_LEVEL_TEMPERATURE + (TROPOSPHERE_TEMP_GRADIENT * altitude)
        } else if altitude <= 20000.0 {
            216.65 // Temperature stays constant from 11km to 20km
        } else {
            216.65 + 0.001 * (altitude - 20000.0) // Temperature slowly increases above 20km
        }
    }

    fn calculate_wind_speed(&self) -> f64 {
        let base_wind_speed = 5.0; // Base wind speed at sea level
        let altitude_factor = self.altitude / 10000.0; // Simplified altitude factor
        let random_variation: f64 = rand::random::<f64>() * 2.0; // Randomize wind speed slightly
        base_wind_speed * (1.0 + altitude_factor) + random_variation
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::relative_eq;

    #[test]
    fn test_gravity_calculation() {
        let env = Environment::new();

        let altitudes = [0.0, 5000.0, 11000.0, 15000.0, 25000.0];
        let expected_gravities = [9.81, 9.7946, 9.7762, 9.7640, 9.7335];

        for (i, &altitude) in altitudes.iter().enumerate() {
            let calculated_gravity = env.calculate_gravity(altitude);
            assert!(
                relative_eq!(calculated_gravity, expected_gravities[i], epsilon = 1e-4),
                "Gravity calculation mismatch at altitude {}: expected {}, got {}",
                altitude,
                expected_gravities[i],
                calculated_gravity
            );
        }
    }

    #[test]
    fn test_air_density_calculation() {
        let env = Environment::new();

        let altitudes = [0.0, 5000.0, 11000.0, 15000.0, 25000.0];
        let expected_air_densities = [1.225, 0.736, 0.364, 0.194, 0.040];

        for (i, &altitude) in altitudes.iter().enumerate() {
            let calculated_air_density = env.calculate_air_density(altitude);
            assert!(
                relative_eq!(
                    calculated_air_density,
                    expected_air_densities[i],
                    epsilon = 1e-3
                ),
                "Air density calculation mismatch at altitude {}: expected {}, got {}",
                altitude,
                expected_air_densities[i],
                calculated_air_density
            );
        }
    }

    #[test]
    fn test_temperature_calculation() {
        let env = Environment::new();

        let altitudes = [0.0, 5000.0, 11000.0, 15000.0, 25000.0];
        let expected_temperatures = [288.15, 255.65, 216.65, 216.65, 221.65];

        for (i, &altitude) in altitudes.iter().enumerate() {
            let calculated_temperature = env.calculate_temperature(altitude);
            assert!(
                relative_eq!(
                    calculated_temperature,
                    expected_temperatures[i],
                    epsilon = 1e-9
                ),
                "Temperature calculation mismatch at altitude {}: expected {}, got {}",
                altitude,
                expected_temperatures[i],
                calculated_temperature
            );
        }
    }

    #[test]
    fn test_wind_speed_calculation() {
        let mut env = Environment::new();

        env.altitude = 5000.0;
        let wind_speed = env.calculate_wind_speed();
        assert!(
            wind_speed > 5.0,
            "Wind speed at altitude 5000m should be greater than 5 m/s."
        );
    }
}
