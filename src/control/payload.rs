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
    fn test_new_payload() {
        let payload = Payload::new(100.0, 1000.0);
        assert_eq!(payload.mass, 100.0);
        assert_eq!(payload.deployment_altitude, 1000.0);
        assert_eq!(payload.deployed, false);
    }

    #[test]
    fn test_check_deployment() {
        let mut payload = Payload::new(100.0, 1000.0);

        payload.check_deployment(500.0);
        assert_eq!(payload.deployed, false);

        payload.check_deployment(1000.0);
        assert_eq!(payload.deployed, true);
    }

    #[test]
    fn test_get_mass() {
        let mut payload = Payload::new(100.0, 1000.0);
        assert_eq!(payload.get_mass(), 100.0);

        payload.check_deployment(1000.0);
        assert_eq!(payload.get_mass(), 0.0);
    }

    #[test]
    fn test_is_deployed() {
        let mut payload = Payload::new(100.0, 1000.0);
        assert_eq!(payload.is_deployed(), false);

        payload.check_deployment(1000.0);
        assert_eq!(payload.is_deployed(), true);
    }
}
