use crate::{
    trajectory_system::{aerodynamics::Aerodynamics, kinematics::Kinematics},
    utils::vector2d::Vector2D,
};

use super::{
    environment::Environment,
    fuel_managment::FlightPhase,
    guidance::{GuidanceCommand, GuidanceSystem},
    mission::Mission,
    structure::Structure,
};

use std::time::Duration;

#[derive(PartialEq, Debug, Clone)]
pub enum RocketState {
    Idle,
    LaunchSequence,
    Launched,
    Ascent,
    OrbitInsertion,
    OrbitStabilization,
    Transfer,
    Approach,
    Rendezvous,
    Descent,
    Landing,
    Landed,
    Returning,
    MissionComplete,
}

pub struct Rocket {
    pub structure: Structure,
    pub guidance: GuidanceSystem,
    pub environment: Environment,
    pub aerodynamics: Aerodynamics,
    pub kinematics: Kinematics,
    pub state: RocketState,
    pub mission: Mission,
    launch_sequence_start: Option<std::time::Instant>,
    engine_startup_duration: Duration,
}

impl Rocket {
    pub fn new(
        structure: Structure,
        mission: Mission,
        aerodynamics: Aerodynamics,
        launch_angle: f64,
    ) -> Self {
        let launch_site_position = Vector2D::new(0.0, mission.start_body.radius);
        let initial_orientation = launch_angle.to_radians();

        let environment = Environment::new(mission.start_body.clone());
        let guidance = GuidanceSystem::new(mission.clone());
        Rocket {
            structure,
            guidance,
            environment,
            aerodynamics,
            kinematics: Kinematics::new(initial_orientation, launch_site_position),
            state: RocketState::Idle,
            mission,
            launch_sequence_start: None,
            engine_startup_duration: Duration::from_secs(5),
        }
    }

    pub fn start_launch_sequence(&mut self) {
        if self.state == RocketState::Idle {
            self.state = RocketState::LaunchSequence;
            self.structure.activate_first_stage();
            self.launch_sequence_start = Some(std::time::Instant::now());
            println!("Launch sequence initiated. Engine startup sequence begun.");
        }
    }

    pub fn update(&mut self, delta_time: f64) -> Result<(), &'static str> {
        match self.state {
            RocketState::Idle => Ok(()),
            RocketState::LaunchSequence => self.update_launch_sequence(delta_time),
            _ => self.update_flight(delta_time),
        }
    }

    fn update_launch_sequence(&mut self, delta_time: f64) -> Result<(), &'static str> {
        if let Some(start_time) = self.launch_sequence_start {
            println!("Launch sequence elapsed time: {:?}", start_time.elapsed());

            // Check if the engine is fully active
            if self
                .structure
                .stages
                .first()
                .unwrap()
                .propulsion
                .is_active()
            {
                self.state = RocketState::Launched;
                println!("Engine startup complete. Rocket launched!");
            } else {
                // Update the structure, allowing the PropulsionSystem to handle startup
                self.structure
                    .update(delta_time, 1.0, self.environment.pressure);
                println!(
                    "Engine is starting. Current elapsed time: {:?}",
                    start_time.elapsed()
                );
            }
        }
        Ok(())
    }

    fn update_flight(&mut self, delta_time: f64) -> Result<(), &'static str> {
        let flight_data = self.gather_flight_data();
        self.check_thrust_sufficiency(&flight_data)?;
        self.update_rocket_systems(delta_time, &flight_data)?;
        self.update_state();

        Ok(())
    }

    fn gather_flight_data(&self) -> FlightData {
        FlightData {
            current_altitude: self.kinematics.get_altitude(&self.mission.start_body),
            current_velocity: self.kinematics.get_velocity_magnitude(),
            gravity: self
                .mission
                .start_body
                .gravity_at_altitude(self.kinematics.get_altitude(&self.mission.start_body)),
            total_thrust: self.structure.get_total_thrust(),
            target_altitude: self.mission.target_orbit_altitude.unwrap_or(0.0),
            target_velocity: self
                .kinematics
                .calculate_orbital_velocity(&self.mission.start_body),
        }
    }

    fn check_thrust_sufficiency(&self, flight_data: &FlightData) -> Result<(), &'static str> {
        if self.state == RocketState::Ascent
            && !self.structure.is_thrust_sufficient(flight_data.gravity)
        {
            println!("Thrust is insufficient to overcome gravity at current altitude.");
            return Err("Thrust is insufficient.");
        }
        Ok(())
    }

    fn update_rocket_systems(
        &mut self,
        delta_time: f64,
        flight_data: &FlightData,
    ) -> Result<(), &'static str> {
        let current_phase = self.get_current_flight_phase();
        self.structure.set_flight_phase(current_phase);

        let optimized_throttle = self.structure.fuel_management.optimize_fuel_consumption(
            flight_data.current_altitude,
            flight_data.target_altitude,
            flight_data.current_velocity,
            flight_data.target_velocity,
        );

        let guidance_command = self.guidance.update(
            &self.state,
            self.kinematics.position,
            self.kinematics.velocity,
            self.kinematics.orientation,
            delta_time,
            self.structure.get_total_fuel(),
        )?;

        self.kinematics.set_mass(self.structure.get_total_mass());
        self.structure.update(
            delta_time,
            guidance_command.thrust * optimized_throttle,
            self.environment.pressure,
        );
        self.apply_guidance_command(&guidance_command, delta_time);
        self.update_environment_and_kinematics(delta_time, guidance_command.thrust);
        self.structure
            .payload
            .check_deployment(flight_data.current_altitude);

        Ok(())
    }

    fn update_environment_and_kinematics(&mut self, delta_time: f64, thrust: f64) {
        self.environment.update(
            &self.kinematics.position,
            &[
                self.mission.start_body.clone(),
                self.mission.target_body.clone(),
            ],
        );

        self.kinematics.update(
            delta_time,
            thrust,
            self.structure.get_total_mass(),
            &self.aerodynamics,
            &self.environment,
            self.kinematics.orientation,
            &[
                self.mission.start_body.clone(),
                self.mission.target_body.clone(),
            ],
        );
    }

    fn apply_guidance_command(&mut self, command: &GuidanceCommand, delta_time: f64) {
        self.kinematics.apply_thrust(command.thrust, delta_time);
        self.kinematics
            .apply_orientation_change(command.orientation_change);
    }

    fn update_state(&mut self) {
        self.state = match self.state {
            RocketState::Idle if self.should_launch() => RocketState::Launched,
            RocketState::Launched => RocketState::Ascent,
            RocketState::Ascent if self.has_reached_orbit_altitude() => RocketState::OrbitInsertion,
            RocketState::OrbitInsertion if self.is_in_stable_orbit() => {
                RocketState::OrbitStabilization
            }
            RocketState::OrbitStabilization if self.is_ready_for_transfer() => {
                RocketState::Transfer
            }
            RocketState::Transfer if self.is_approaching_target() => RocketState::Approach,
            RocketState::Approach if self.is_close_to_target() => RocketState::Rendezvous,
            RocketState::Rendezvous if self.should_begin_descent() => RocketState::Descent,
            RocketState::Descent if self.is_near_surface() => RocketState::Landing,
            RocketState::Landing if self.has_landed() => RocketState::Landed,
            RocketState::Landed if self.mission.return_trip && self.should_return() => {
                RocketState::Returning
            }
            RocketState::Returning if self.has_returned_to_start() => RocketState::Descent,
            RocketState::Landed if !self.mission.return_trip => RocketState::MissionComplete,
            _ => self.state.clone(),
        };
    }

    fn get_current_flight_phase(&self) -> FlightPhase {
        match self.state {
            RocketState::Launched | RocketState::Ascent => FlightPhase::Ascent,
            RocketState::OrbitInsertion
            | RocketState::OrbitStabilization
            | RocketState::Transfer
            | RocketState::Approach
            | RocketState::Rendezvous => FlightPhase::Transfer,
            RocketState::Descent | RocketState::Landing | RocketState::Returning => {
                FlightPhase::Descent
            }
            RocketState::MissionComplete | RocketState::Landed => FlightPhase::Descent,
            _ => FlightPhase::Ascent,
        }
    }

    fn should_launch(&self) -> bool {
        self.state == RocketState::Idle
    }

    fn has_reached_orbit_altitude(&self) -> bool {
        let target_altitude = self.mission.target_orbit_altitude.unwrap_or(0.0);
        self.kinematics.get_altitude(&self.mission.start_body) >= target_altitude
    }

    fn is_in_stable_orbit(&self) -> bool {
        let target_altitude = self.mission.target_orbit_altitude.unwrap_or(0.0);
        let current_altitude = self.kinematics.get_altitude(&self.mission.start_body);
        let current_velocity = self.kinematics.get_velocity_magnitude();
        let orbital_velocity = self
            .kinematics
            .calculate_orbital_velocity(&self.mission.start_body);

        (current_altitude - target_altitude).abs() < 1000.0
            && (current_velocity - orbital_velocity).abs() < 10.0
    }

    fn is_ready_for_transfer(&self) -> bool {
        // Check if we're in a stable orbit and aligned for an efficient transfer
        self.is_in_stable_orbit() && self.is_aligned_for_transfer()
    }

    fn is_aligned_for_transfer(&self) -> bool {
        let transfer_angle = self.calculate_transfer_angle();
        let current_angle = (self.kinematics.position - self.mission.start_body.position).angle();
        (current_angle - transfer_angle).abs() < 0.1 // Within ~5.7 degrees of ideal angle
    }

    fn calculate_transfer_angle(&self) -> f64 {
        // Calculate the ideal angle for a Hohmann transfer
        let r1 = (self.kinematics.position - self.mission.start_body.position).magnitude();
        let r2 = (self.mission.target_body.position - self.mission.start_body.position).magnitude();
        (1.0 - (r1 / r2).powf(3.0 / 2.0)).acos()
    }

    fn is_approaching_target(&self) -> bool {
        let distance_to_target =
            (self.kinematics.position - self.mission.target_body.position).magnitude();
        let initial_distance =
            (self.mission.start_body.position - self.mission.target_body.position).magnitude();
        distance_to_target < initial_distance * 0.1 // Within 10% of initial distance
    }

    fn is_close_to_target(&self) -> bool {
        let distance_to_target =
            (self.kinematics.position - self.mission.target_body.position).magnitude();
        distance_to_target < self.mission.target_body.radius * 5.0 // Within 5 target body radii
    }

    fn should_begin_descent(&self) -> bool {
        self.is_close_to_target() && self.kinematics.get_velocity_magnitude() < 1000.0
        // Slow enough to begin descent
    }

    fn is_near_surface(&self) -> bool {
        let altitude = self.calculate_altitude_above_target();
        altitude < self.mission.target_body.radius * 0.1 // Within 10% of target body radius
    }

    fn calculate_altitude_above_target(&self) -> f64 {
        (self.kinematics.position - self.mission.target_body.position).magnitude()
            - self.mission.target_body.radius
    }

    fn has_landed(&self) -> bool {
        self.calculate_altitude_above_target() < 10.0 && // Within 10 meters of surface
            self.kinematics.get_velocity_magnitude() < 1.0 // Almost stationary
    }

    fn should_return(&self) -> bool {
        self.has_landed()
            && self.mission.return_trip
            && self.structure.get_total_fuel() > self.mission.total_fuel * 0.3 // Ensure we have enough fuel for return trip
    }

    fn has_returned_to_start(&self) -> bool {
        let distance_to_start =
            (self.kinematics.position - self.mission.start_body.position).magnitude();
        distance_to_start < self.mission.start_body.radius * 1.5 // Close to starting body
    }
}

struct FlightData {
    current_altitude: f64,
    current_velocity: f64,
    gravity: f64,
    total_thrust: f64,
    target_altitude: f64,
    target_velocity: f64,
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{MissionFactory, Payload, Stage};
    use std::{thread, time::Duration};

    #[test]
    fn test_rocket_launch_and_ascent() {
        let stage = Stage::new(100_000.0, 20_000.0, 500.0, 200.0);
        let structure = Structure::new(vec![stage], Payload::new(5_000.0, 200_000.0));
        let mission =
            MissionFactory::create_earth_orbit("Test Orbit".to_string(), 200_000.0, 300_000.0);
        let aerodynamics = Aerodynamics::new(0.5, 10.0, 0.3);
        let mut rocket = Rocket::new(structure, mission, aerodynamics, 90.0);

        rocket.start_launch_sequence();
        assert_eq!(rocket.state, RocketState::LaunchSequence);

        let update_interval = Duration::from_millis(100);
        let mut elapsed_time = 0.0;
        while rocket.state == RocketState::LaunchSequence {
            rocket
                .update(0.1)
                .expect("Failed to update during launch sequence");
            elapsed_time += 0.1;
            thread::sleep(update_interval);
        }

        assert_eq!(rocket.state, RocketState::Launched);
        println!("Rocket launched after {:.1} seconds", elapsed_time);

        let mut elapsed_time = 0.0;
        let update_interval = 1.0; // 100ms
        let simulation_duration = 10.0; // 60 seconds
        let initial_altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);

        while elapsed_time < simulation_duration {
            match rocket.update(update_interval) {
                Ok(_) => {
                    let flight_data = rocket.gather_flight_data();
                    println!(
                                "Time: {:.1}s | State: {:?} | Altitude: {:.2} m | Velocity: {:.2} m/s | Thrust: {:.2} N",
                                elapsed_time,
                                rocket.state,
                                flight_data.current_altitude,
                                flight_data.current_velocity,
                                flight_data.total_thrust
                            );

                    if flight_data.current_altitude > 100_000.0 {
                        // Break if we reach 100 km
                        break;
                    }
                }
                Err(e) => {
                    println!("Error during update: {}", e);
                    break;
                }
            }
            elapsed_time += update_interval;
        }

        let final_altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);
        assert!(
               final_altitude > initial_altitude,
               "Rocket's altitude should have increased after ascent. Initial altitude: {:.2} m, Final altitude: {:.2} m",
               initial_altitude,
               final_altitude
           );

        let final_velocity = rocket.kinematics.get_velocity_magnitude();
        assert!(
            final_velocity > 0.0,
            "Rocket's velocity should be positive during ascent. Current velocity: {:.2} m/s",
            final_velocity
        );
    }
}
