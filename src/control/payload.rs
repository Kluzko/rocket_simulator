#[derive(Clone)]
pub struct Payload {
    pub mass: f64, // kg
    pub deployed: bool,
    pub deployment_altitude: f64, // m
}

impl Payload {
    pub fn new(mass: f64, deployment_altitude: f64) -> Self {
        Payload {
            mass,
            deployed: false,
            deployment_altitude,
        }
    }

    pub fn check_deployment(&mut self, current_altitude: f64) {
        if current_altitude >= self.deployment_altitude && !self.deployed {
            self.deployed = true;
            println!(
                "Payload deployed at altitude {:.2} meters!",
                current_altitude
            );
        }
    }

    pub fn get_mass(&self) -> f64 {
        if self.deployed {
            0.0
        } else {
            self.mass
        }
    }

    pub fn is_deployed(&self) -> bool {
        self.deployed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_payload_initialization() {
        let payload = Payload::new(1000.0, 200_000.0);

        assert_eq!(payload.mass, 1000.0);
        assert_eq!(payload.deployment_altitude, 200_000.0);
        assert!(!payload.deployed); // Payload should not be deployed initially
    }

    #[test]
    fn test_payload_deployment() {
        let mut payload = Payload::new(500.0, 150_000.0);

        // Test at lower altitude, deployment should not happen
        payload.check_deployment(100_000.0);
        assert!(!payload.is_deployed());

        // Test at altitude higher than deployment altitude
        payload.check_deployment(160_000.0);
        assert!(payload.is_deployed()); // Payload should be deployed now
    }

    #[test]
    fn test_payload_mass_before_and_after_deployment() {
        let mut payload = Payload::new(800.0, 120_000.0);

        // Check mass before deployment
        assert_eq!(payload.get_mass(), 800.0);

        // Deploy the payload at the right altitude
        payload.check_deployment(130_000.0);
        assert!(payload.is_deployed());

        // Check mass after deployment
        assert_eq!(payload.get_mass(), 0.0); // Should return 0 after deployment
    }

    #[test]
    fn test_payload_deployment_exact_altitude() {
        let mut payload = Payload::new(750.0, 100_000.0);

        // Deploy the payload exactly at the deployment altitude
        payload.check_deployment(100_000.0);
        assert!(payload.is_deployed());
    }

    #[test]
    fn test_payload_no_deployment_below_altitude() {
        let mut payload = Payload::new(750.0, 200_000.0);

        // Check at altitude below deployment
        payload.check_deployment(199_000.0);
        assert!(!payload.is_deployed());
    }
}
