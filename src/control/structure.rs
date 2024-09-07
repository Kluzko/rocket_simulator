use super::{launch_stages::Stage, payload::Payload};

pub struct Structure {
    pub stages: Vec<Stage>, // Rocket stages
    pub payload: Payload,   // Rocket payload
}

impl Structure {
    pub fn new(stages: Vec<Stage>, payload: Payload) -> Self {
        Structure { stages, payload }
    }

    pub fn remove_jettisoned_stages(&mut self) {
        self.stages.retain(|stage| stage.is_active);
    }

    pub fn get_total_mass(&self) -> f64 {
        let stages_mass: f64 = self.stages.iter().map(|stage| stage.get_total_mass()).sum();
        stages_mass + self.payload.get_mass()
    }

    pub fn get_total_fuel(&self) -> f64 {
        self.stages
            .iter()
            .map(|stage| stage.propulsion.fuel_mass)
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_stage(fuel_mass: f64, dry_mass: f64) -> Stage {
        Stage::new(fuel_mass, dry_mass, 10.0, 100.0)
    }

    #[test]
    fn test_new_structure() {
        let stages = vec![
            create_test_stage(1000.0, 500.0),
            create_test_stage(800.0, 400.0),
        ];
        let payload = Payload::new(200.0, 100000.0);
        let structure = Structure::new(stages, payload);

        assert_eq!(structure.stages.len(), 2);
        assert_eq!(structure.payload.get_mass(), 200.0);
        assert_eq!(structure.get_total_mass(), 2900.0); // 1000 + 500 + 800 + 400 + 200
    }

    #[test]
    fn test_update_mass() {
        let stages = vec![
            create_test_stage(1000.0, 500.0),
            create_test_stage(800.0, 400.0),
        ];
        let payload = Payload::new(200.0, 100000.0);
        let mut structure = Structure::new(stages, payload);

        // Simulate fuel consumption
        structure.stages[0].propulsion.fuel_mass -= 200.0;
        structure.stages[1].propulsion.fuel_mass -= 100.0;

        assert_eq!(structure.get_total_mass(), 2600.0); // 800 + 500 + 700 + 400 + 200
    }

    #[test]
    fn test_remove_jettisoned_stages() {
        let stages = vec![
            create_test_stage(1000.0, 500.0),
            create_test_stage(800.0, 400.0),
            create_test_stage(600.0, 300.0),
        ];
        let payload = Payload::new(200.0, 100000.0);
        let mut structure = Structure::new(stages, payload);

        // Activate second stage, deactivate first and third
        structure.stages[0].is_active = false;
        structure.stages[1].is_active = true;
        structure.stages[2].is_active = false;

        structure.remove_jettisoned_stages();

        assert_eq!(structure.stages.len(), 1);
        assert_eq!(structure.get_total_mass(), 1400.0); // 800 + 400 + 200
    }

    #[test]
    fn test_get_total_mass() {
        let stages = vec![
            create_test_stage(1000.0, 500.0),
            create_test_stage(800.0, 400.0),
        ];
        let payload = Payload::new(200.0, 100000.0);
        let structure = Structure::new(stages, payload);

        assert_eq!(structure.get_total_mass(), 2900.0);
    }

    #[test]
    fn test_get_total_fuel() {
        let stages = vec![
            create_test_stage(1000.0, 500.0),
            create_test_stage(800.0, 400.0),
        ];
        let payload = Payload::new(200.0, 100000.0);
        let mut structure = Structure::new(stages, payload);

        assert_eq!(structure.get_total_fuel(), 1800.0); // 1000 + 800

        // Simulate fuel consumption
        structure.stages[0].propulsion.fuel_mass -= 200.0;
        structure.stages[1].propulsion.fuel_mass -= 100.0;

        assert_eq!(structure.get_total_fuel(), 1500.0); // 800 + 700
    }
}
