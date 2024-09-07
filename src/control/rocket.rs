use crate::{
    trajectory_system::{aerodynamics::Aerodynamics, kinematics::Kinematics},
    utils::vector2d::Vector2D,
};

use super::{environment::Environment, guidance::GuidanceSystem, structure::Structure};

pub struct Rocket {
    pub structure: Structure,
    pub guidance: GuidanceSystem,
    pub environment: Environment,
    pub kinematics: Kinematics,
    pub aerodynamics: Aerodynamics,
    pub state: RocketState,
    pub target_distance: f64,
}

#[derive(PartialEq, Debug, Clone)]
pub enum RocketState {
    Idle,
    Launched,
    Freefall,
    AtTarget,
    Returning,
    Landed,
}

impl Rocket {
    pub fn new(
        mut structure: Structure,
        guidance: GuidanceSystem,
        environment: Environment,
        launch_angle: f64,
        launch_site_position: Vector2D,
        drag_coefficient: f64,
        surface_area: f64,
        lift_coefficient: f64,
        target_distance: f64,
    ) -> Self {
        if let Some(first_stage) = structure.stages.first_mut() {
            first_stage.activate();
        }

        Rocket {
            structure,
            guidance,
            environment,
            kinematics: Kinematics::new(launch_angle, launch_site_position),
            aerodynamics: Aerodynamics::new(drag_coefficient, surface_area, lift_coefficient),
            state: RocketState::Idle,
            target_distance,
        }
    }

    pub fn launch(&mut self) {
        self.state = RocketState::Launched;
        println!("Rocket launched!");
    }

    pub fn update(&mut self, delta_time: f64) {
        match self.state {
            RocketState::Idle => {
                // Do nothing, waiting for launch command
            }

            RocketState::Launched => {
                self.guidance.update(
                    self.kinematics.get_altitude(),
                    self.kinematics.get_orientation(),
                    self.kinematics.get_velocity_magnitude(),
                    None,
                    delta_time,
                );

                let (_, throttle, _) = self.guidance.calculate_control_commands();
                let orientation_change = self.guidance.get_orientation_change(delta_time);

                let thrust_magnitude = self.calculate_thrust(throttle, delta_time);
                let total_mass = self.structure.get_total_mass();

                self.kinematics.update(
                    delta_time,
                    thrust_magnitude,
                    total_mass,
                    &self.aerodynamics,
                    &mut self.environment,
                    orientation_change,
                    &self.state,
                );

                self.structure.remove_jettisoned_stages();

                let current_distance = self.kinematics.position.y;
                if current_distance >= self.target_distance {
                    self.state = RocketState::AtTarget;
                    println!(
                        "Rocket has reached the target distance: {:.2} meters",
                        self.target_distance
                    );
                }

                if self.structure.get_total_fuel() <= 0.0 {
                    self.state = RocketState::Freefall;
                }
            }

            RocketState::AtTarget => {
                // Start returning to Earth
                self.state = RocketState::Returning;
                println!("Rocket is returning to Earth.");
            }

            RocketState::Freefall => {
                if self.kinematics.position.y <= 0.0 {
                    self.state = RocketState::Landed;
                }
            }

            RocketState::Returning => {
                // Simulate return flight
                // Update similar to "Launched", but adjust the angle for descent
                println!("Returning state");
            }

            RocketState::Landed => {
                println!("Rocket has landed safely.");
            }
        }
    }

    fn calculate_thrust(&mut self, throttle: f64, delta_time: f64) -> f64 {
        if let Some(active_stage) = self.structure.stages.first_mut() {
            active_stage.update(throttle, delta_time);
            let thrust_magnitude = active_stage.propulsion.get_thrust() * throttle;

            // Calculate the gravitational force acting on the rocket
            let total_mass = self.structure.get_total_mass();
            let gravity_force = total_mass * self.environment.gravity;

            assert!(
                thrust_magnitude > gravity_force,
                "Insufficient thrust to overcome gravity! Thrust: {} N, Gravity: {} N. \
                       Increase thrust to at least {} N for liftoff.",
                thrust_magnitude,
                gravity_force,
                gravity_force
            );

            thrust_magnitude
        } else {
            0.0
        }
    }
}
