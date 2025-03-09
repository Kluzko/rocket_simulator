use super::{
    fuel_managment::{FlightPhase, FuelManagementSystem},
    launch_stages::Stage,
    payload::Payload,
};

pub struct Structure {
    pub stages: Vec<Stage>,
    pub payload: Payload,
    pub fuel_management: FuelManagementSystem,
    current_phase: FlightPhase,
}

impl Structure {
    pub fn new(stages: Vec<Stage>, payload: Payload) -> Self {
        let total_fuel: f64 = stages.iter().map(|stage| stage.propulsion.fuel_mass).sum();
        let fuel_management = FuelManagementSystem::new(total_fuel);

        Structure {
            stages,
            payload,
            fuel_management,
            current_phase: FlightPhase::Ascent,
        }
    }

    pub fn set_flight_phase(&mut self, phase: FlightPhase) {
        self.current_phase = phase;
        self.fuel_management.set_current_phase(phase);
    }

    pub fn update(&mut self, delta_time: f64, throttle: f64, atmospheric_pressure: f64) {
        if let Some(active_stage) = self.stages.first_mut() {
            active_stage.update(
                throttle,
                delta_time,
                atmospheric_pressure,
                &mut self.fuel_management,
                self.current_phase,
            );

            if active_stage.is_depleted() {
                self.jettison_stage();
            }
        }

        self.synchronize_fuel_management();
    }

    pub fn activate_first_stage(&mut self) {
        if let Some(first_stage) = self.stages.first_mut() {
            first_stage.activate();
        }
    }

    pub fn get_total_thrust(&self) -> f64 {
        self.stages.iter().map(|stage| stage.get_thrust()).sum()
    }

    pub fn is_thrust_sufficient(&self, gravity: f64) -> bool {
        let total_thrust = self.get_total_thrust();
        let total_mass = self.get_total_mass();
        let weight = total_mass * gravity;

        total_thrust > weight
    }

    fn jettison_stage(&mut self) {
        if !self.stages.is_empty() {
            let jettisoned_stage = self.stages.remove(0);
            println!(
                "Stage jettisoned. Remaining fuel: {}",
                jettisoned_stage.propulsion.fuel_mass
            );
            if let Some(next_stage) = self.stages.first_mut() {
                next_stage.activate();
            }
            self.synchronize_fuel_management();
        }
    }

    fn synchronize_fuel_management(&mut self) {
        let actual_remaining_fuel: f64 = self
            .stages
            .iter()
            .map(|stage| stage.propulsion.fuel_mass)
            .sum();
        self.fuel_management.synchronize_fuel(actual_remaining_fuel);
    }

    pub fn get_total_mass(&self) -> f64 {
        self.stages
            .iter()
            .map(|stage| stage.get_total_mass())
            .sum::<f64>()
            + self.payload.get_mass()
    }

    pub fn get_total_fuel(&self) -> f64 {
        self.stages
            .iter()
            .map(|stage| stage.propulsion.fuel_mass)
            .sum()
    }

    pub fn active_stages_count(&self) -> usize {
        self.stages.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn create_mock_stage(fuel_mass: f64, dry_mass: f64) -> Stage {
        Stage::new(fuel_mass, dry_mass, 1.0, 10000.0)
    }

    fn create_mock_payload(mass: f64) -> Payload {
        Payload::new(mass, 200_000.0)
    }

    #[test]
    fn test_structure_initialization() {
        let stages = vec![
            create_mock_stage(1000.0, 500.0),
            create_mock_stage(500.0, 250.0),
        ];
        let payload = create_mock_payload(100.0);
        let structure = Structure::new(stages, payload);

        assert_eq!(structure.stages.len(), 2);
        assert_eq!(structure.get_total_fuel(), 1500.0);
        assert_eq!(structure.get_total_mass(), 2350.0);
        assert_eq!(structure.current_phase, FlightPhase::Ascent);
    }

    #[test]
    fn test_activate_first_stage() {
        let stages = vec![create_mock_stage(1000.0, 500.0)];
        let payload = create_mock_payload(100.0);
        let mut structure = Structure::new(stages, payload);

        structure.activate_first_stage();
        assert!(structure.stages[0].is_active);
    }

    #[test]
    fn test_update_and_jettison_stage() {
        let stages = vec![
            create_mock_stage(100.0, 500.0),
            create_mock_stage(500.0, 250.0),
        ];
        let payload = create_mock_payload(100.0);
        let mut structure = Structure::new(stages, payload);

        structure.activate_first_stage();

        // Run updates until the first stage is depleted
        let mut updates = 0;
        while structure.stages.len() > 1 && updates < 1000 {
            structure.update(0.1, 1.0, 101325.0);
            updates += 1;
        }

        println!("Updates performed: {}", updates);
        println!("Remaining stages: {}", structure.stages.len());
        println!("Is active: {}", structure.stages[0].is_active);

        assert_eq!(structure.stages.len(), 1);
        assert!(structure.stages[0].is_active);
    }

    #[test]
    fn test_get_total_thrust() {
        let stages = vec![create_mock_stage(1000.0, 500.0)];
        let payload = create_mock_payload(100.0);
        let mut structure = Structure::new(stages, payload);

        structure.activate_first_stage();

        // Update once to start the engine
        structure.update(0.1, 1.0, 101325.0);

        let total_thrust = structure.get_total_thrust();
        println!("Total thrust: {}", total_thrust);
        assert!(total_thrust > 0.0);
    }

    #[test]
    fn test_is_thrust_sufficient() {
        let stages = vec![create_mock_stage(1000.0, 500.0)];
        let payload = create_mock_payload(100.0);
        let mut structure = Structure::new(stages, payload);

        structure.activate_first_stage();

        // Update once to start the engine
        structure.update(0.1, 1.0, 101325.0);

        let is_sufficient = structure.is_thrust_sufficient(9.81);
        println!("Is thrust sufficient: {}", is_sufficient);
        assert!(is_sufficient);
    }

    #[test]
    fn test_get_total_mass() {
        let stages = vec![
            create_mock_stage(1000.0, 500.0),
            create_mock_stage(500.0, 250.0),
        ];
        let payload = create_mock_payload(100.0);
        let structure = Structure::new(stages, payload);

        assert_relative_eq!(structure.get_total_mass(), 2350.0, epsilon = 0.001);
    }

    #[test]
    fn test_set_flight_phase() {
        let stages = vec![create_mock_stage(1000.0, 500.0)];
        let payload = create_mock_payload(100.0);
        let mut structure = Structure::new(stages, payload);

        structure.set_flight_phase(FlightPhase::Transfer);
        assert_eq!(structure.current_phase, FlightPhase::Transfer);
    }

    #[test]
    fn test_synchronize_fuel_management() {
        let stages = vec![
            create_mock_stage(1000.0, 500.0),
            create_mock_stage(500.0, 250.0),
        ];
        let payload = create_mock_payload(100.0);
        let mut structure = Structure::new(stages, payload);

        // Simulate fuel consumption
        structure.stages[0].propulsion.consume_fuel(200.0);
        structure.synchronize_fuel_management();

        assert_relative_eq!(
            structure.fuel_management.get_total_remaining_fuel(),
            1300.0,
            epsilon = 0.001
        );
    }
}
