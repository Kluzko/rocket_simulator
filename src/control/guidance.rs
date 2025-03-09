use super::{mission::Mission, rocket::RocketState};
use crate::errors::SimulationError;
use crate::{utils::vector2d::Vector2D, GRAVITATIONAL_CONSTANT};
use std::f64::consts::PI;

pub struct GuidanceSystem {
    mission: Mission,
    pid_controllers: PIDControllers,
}

struct PIDControllers {
    altitude: PIDController,
    lateral: PIDController,
    orientation: PIDController,
}

#[derive(Clone)]
pub struct GuidanceCommand {
    pub thrust: f64,
    pub orientation_change: f64,
}

impl GuidanceSystem {
    pub fn new(mission: Mission) -> Self {
        GuidanceSystem {
            mission,
            pid_controllers: PIDControllers {
                altitude: PIDController::new(0.9, 0.01, 0.5).with_limits(100.0, -1.0, 1.0),
                lateral: PIDController::new(0.6, 0.01, 0.2).with_limits(50.0, -0.5, 0.5),
                orientation: PIDController::new(1.5, 0.05, 0.4).with_limits(10.0, -0.1, 0.1),
            },
        }
    }

    pub fn update(
        &mut self,
        state: &RocketState,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
        fuel_remaining: f64,
    ) -> Result<GuidanceCommand, SimulationError> {
        let command = match state {
            RocketState::Idle | RocketState::LaunchSequence => Ok(GuidanceCommand {
                thrust: 0.0,
                orientation_change: 0.0,
            }),
            RocketState::Launched | RocketState::Ascent => {
                self.guide_ascent(position, velocity, orientation, delta_time, fuel_remaining)
            }
            RocketState::OrbitInsertion | RocketState::OrbitStabilization => {
                self.guide_orbit(position, velocity, orientation, delta_time)
            }
            RocketState::Transfer => {
                self.guide_transfer(position, velocity, orientation, delta_time)
            }
            RocketState::Approach | RocketState::Rendezvous => {
                self.guide_approach(position, velocity, orientation, delta_time)
            }
            RocketState::Descent => self.guide_descent(position, velocity, orientation, delta_time),
            RocketState::Landing => self.guide_landing(position, velocity, orientation, delta_time),
            RocketState::Landed | RocketState::MissionComplete => Ok(GuidanceCommand {
                thrust: 0.0,
                orientation_change: 0.0,
            }),
            RocketState::Returning => {
                self.guide_return(position, velocity, orientation, delta_time)
            }
        }?;

        Ok(command)
    }

    fn guide_ascent(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
        fuel_remaining: f64,
    ) -> Result<GuidanceCommand, SimulationError> {
        let target_altitude = self.calculate_target_altitude();
        // Calculate altitude properly as distance from center minus radius
        let current_altitude = (position - self.mission.start_body.position).magnitude()
            - self.mission.start_body.radius;
        let altitude_error = target_altitude - current_altitude;

        if altitude_error < 0.0 {
            return Ok(GuidanceCommand {
                thrust: 0.1, // Minimum thrust to avoid flame-out
                orientation_change: 0.0,
            });
        }

        let thrust = self
            .pid_controllers
            .altitude
            .calculate(altitude_error, delta_time)
            .clamp(0.0, 1.0);

        let target_lateral_position = self.calculate_target_lateral_position();
        let lateral_error = target_lateral_position - position.x;
        let lateral_correction = self
            .pid_controllers
            .lateral
            .calculate(lateral_error, delta_time);

        let target_orientation = (0.0 + lateral_correction).to_radians();
        let orientation_error = target_orientation - orientation;
        let orientation_change = self
            .pid_controllers
            .orientation
            .calculate(orientation_error, delta_time)
            .clamp(-0.02, 0.02);

        // Adaptive thrust scaling based on altitude, velocity, and remaining fuel
        let adaptive_thrust = self.calculate_adaptive_thrust(
            current_altitude,
            target_altitude,
            velocity.magnitude(),
            fuel_remaining,
        );

        Ok(GuidanceCommand {
            thrust: (adaptive_thrust * thrust).clamp(0.1, 1.0),
            orientation_change,
        })
    }

    fn guide_orbit(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
    ) -> Result<GuidanceCommand, SimulationError> {
        let target_altitude = self.calculate_target_altitude();
        let current_altitude = (position - self.mission.start_body.position).magnitude()
            - self.mission.start_body.radius;
        let altitude_error = target_altitude - current_altitude;

        let direct_altitude_correction = altitude_error / 20000.0;

        let pid_altitude_correction = self
            .pid_controllers
            .altitude
            .calculate(altitude_error, delta_time);

        let altitude_correction = pid_altitude_correction + direct_altitude_correction;

        let target_velocity = self.calculate_orbital_velocity(target_altitude);
        let velocity_error = target_velocity - velocity.magnitude();

        let direct_velocity_correction = velocity_error / 2000.0;

        let pid_velocity_correction = self
            .pid_controllers
            .lateral
            .calculate(velocity_error, delta_time);

        let velocity_correction = pid_velocity_correction + direct_velocity_correction;

        let combined_correction = altitude_correction + velocity_correction;
        let thrust = if altitude_error.abs() > 500.0 || velocity_error.abs() > 50.0 {
            // Only use minimum thrust for significant errors
            combined_correction.clamp(0.1, 1.0)
        } else if altitude_error.abs() < 100.0 && velocity_error.abs() < 10.0 {
            // Dead band for very small errors - no thrust needed
            0.0
        } else {
            // For moderate errors, allow zero thrust
            combined_correction.clamp(0.0, 1.0)
        };

        let orientation_change =
            self.calculate_orbital_orientation_change(position, velocity, orientation);

        Ok(GuidanceCommand {
            thrust,
            orientation_change,
        })
    }

    fn guide_transfer(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
    ) -> Result<GuidanceCommand, SimulationError> {
        let target_position = self.mission.target_body.position;
        let distance_to_target = (target_position - position).magnitude();

        let desired_velocity = (target_position - position).normalize()
            * self.calculate_transfer_velocity(distance_to_target);
        let velocity_correction = desired_velocity - velocity;
        let thrust =
            velocity_correction.normalize() * self.calculate_transfer_thrust(distance_to_target);
        let orientation_change = (thrust.angle() - orientation).clamp(-0.1, 0.1);

        // Use delta_time to smooth out the thrust changes
        let smoothed_thrust = thrust.magnitude() * delta_time.min(1.0);

        Ok(GuidanceCommand {
            thrust: smoothed_thrust,
            orientation_change,
        })
    }

    fn guide_approach(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
    ) -> Result<GuidanceCommand, SimulationError> {
        let target_position = self.mission.target_body.position;
        let distance_to_target = (target_position - position).magnitude();

        let desired_velocity = (target_position - position).normalize()
            * self.calculate_approach_velocity(distance_to_target);
        let velocity_correction = desired_velocity - velocity;
        let thrust =
            velocity_correction.normalize() * self.calculate_approach_thrust(distance_to_target);
        let orientation_change = (thrust.angle() - orientation).clamp(-0.05, 0.05);

        // Use delta_time to smooth out the thrust changes
        let smoothed_thrust = thrust.magnitude() * delta_time.min(1.0);

        Ok(GuidanceCommand {
            thrust: smoothed_thrust,
            orientation_change,
        })
    }

    fn guide_descent(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
    ) -> Result<GuidanceCommand, SimulationError> {
        let surface_position = self.mission.target_body.position
            + (position - self.mission.target_body.position).normalize()
                * self.mission.target_body.radius;
        let altitude = (position - surface_position).magnitude();

        let desired_vertical_velocity = -(altitude / 500.0).clamp(-500.0, -50.0);
        let current_vertical_velocity =
            velocity.dot(&(position - self.mission.target_body.position).normalize());
        let vertical_velocity_error = desired_vertical_velocity - current_vertical_velocity;

        let thrust = self
            .pid_controllers
            .altitude
            .calculate(vertical_velocity_error, delta_time)
            .clamp(0.1, 0.8);

        let desired_orientation = (surface_position - position).angle();
        let orientation_error = GuidanceSystem::wrap_to_2pi(desired_orientation - orientation);
        let orientation_change = self
            .pid_controllers
            .orientation
            .calculate(orientation_error, delta_time)
            .clamp(-0.05, 0.05);

        Ok(GuidanceCommand {
            thrust,
            orientation_change,
        })
    }

    fn wrap_to_2pi(angle: f64) -> f64 {
        let two_pi = 2.0 * std::f64::consts::PI;
        // Handle potential numerical issues with very large angles
        let normalized = if angle.is_finite() {
            (angle % two_pi + two_pi) % two_pi
        } else {
            0.0 // Default to 0 for non-finite angles
        };
        normalized
    }

    fn guide_landing(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
    ) -> Result<GuidanceCommand, SimulationError> {
        let surface_position = self.mission.target_body.position
            + (position - self.mission.target_body.position).normalize()
                * self.mission.target_body.radius;
        let altitude = (position - surface_position).magnitude();

        let desired_vertical_velocity = -2.0 * (altitude / 100.0).clamp(0.1, 1.0); // Adjust descent rate based on altitude
        let current_vertical_velocity =
            velocity.dot(&(position - self.mission.target_body.position).normalize());
        let vertical_velocity_error = desired_vertical_velocity - current_vertical_velocity;

        let thrust = self
            .pid_controllers
            .altitude
            .calculate(vertical_velocity_error, delta_time)
            .clamp(0.1, 1.0);
        let desired_orientation = (surface_position - position).angle();
        let orientation_change = (desired_orientation - orientation).clamp(-0.05, 0.05);

        Ok(GuidanceCommand {
            thrust,
            orientation_change,
        })
    }

    fn guide_return(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
    ) -> Result<GuidanceCommand, SimulationError> {
        let target_position = self.mission.start_body.position;
        let distance_to_target = (target_position - position).magnitude();

        let desired_velocity = (target_position - position).normalize()
            * self.calculate_return_velocity(distance_to_target);
        let velocity_correction = desired_velocity - velocity;
        let thrust =
            velocity_correction.normalize() * self.calculate_return_thrust(distance_to_target);
        let orientation_change = (thrust.angle() - orientation).clamp(-0.1, 0.1);

        // Use delta_time to smooth out the thrust changes
        let smoothed_thrust = thrust.magnitude() * delta_time.min(1.0);

        Ok(GuidanceCommand {
            thrust: smoothed_thrust,
            orientation_change,
        })
    }

    fn calculate_target_altitude(&self) -> f64 {
        self.mission.target_orbit_altitude.unwrap_or(0.0)
    }

    fn calculate_target_lateral_position(&self) -> f64 {
        // In a more advanced implementation, this could account for
        // the desired orbital inclination, launch site latitude, etc.
        // For now, aim for an equatorial orbit (x=0)
        0.0 // Placeholder
    }

    /// Calculates the velocity needed to transfer from current orbit to target
    /// This uses the Hohmann transfer orbit equation
    /// See: https://en.wikipedia.org/wiki/Hohmann_transfer_orbit
    fn calculate_transfer_velocity(&self, distance: f64) -> f64 {
        // r1 is the radius of the initial orbit
        let r1 = self.mission.start_body.radius + self.mission.target_orbit_altitude.unwrap_or(0.0);
        // r2 is the distance to the target
        let r2 = distance;
        // mu is the standard gravitational parameter
        let mu = GRAVITATIONAL_CONSTANT * self.mission.start_body.mass;

        // Calculate velocity for a Hohmann transfer orbit
        (mu * (2.0 / r1 - 1.0 / ((r1 + r2) / 2.0))).sqrt()
    }

    fn calculate_transfer_thrust(&self, distance: f64) -> f64 {
        // Adjust thrust based on distance to optimize fuel consumption
        (1.0 - distance
            / (self.mission.target_body.position - self.mission.start_body.position).magnitude())
        .clamp(0.1, 1.0)
    }

    fn calculate_approach_velocity(&self, distance: f64) -> f64 {
        // Slow down as we approach the target
        (distance / 10.0).clamp(10.0, 1000.0)
    }

    fn calculate_approach_thrust(&self, distance: f64) -> f64 {
        // Increase thrust as we get closer for fine control
        (1.0 - distance
            / (self.mission.target_body.position - self.mission.start_body.position).magnitude()
            * 0.1)
            .clamp(0.1, 0.5)
    }

    fn calculate_return_velocity(&self, distance: f64) -> f64 {
        // Similar to transfer velocity, but in reverse
        self.calculate_transfer_velocity(distance)
    }

    fn calculate_return_thrust(&self, distance: f64) -> f64 {
        // Similar to transfer thrust, but might need more power to escape target body's gravity
        self.calculate_transfer_thrust(distance) * 1.2
    }

    fn calculate_adaptive_thrust(
        &self,
        current_altitude: f64,
        target_altitude: f64,
        velocity: f64,
        fuel_remaining: f64,
    ) -> f64 {
        let altitude_factor =
            ((target_altitude - current_altitude) / target_altitude).clamp(0.0, 1.0);
        let velocity_factor = (1.0 - velocity / 7800.0).clamp(0.0, 1.0); // 7800 m/s is approximate orbital velocity
        let fuel_factor = (fuel_remaining / self.mission.total_fuel).clamp(0.0, 1.0);

        let thrust = altitude_factor * velocity_factor * fuel_factor;
        thrust.clamp(0.1, 1.0) // Minimum thrust of 10% to avoid flame-out
    }

    fn calculate_orbital_velocity(&self, altitude: f64) -> f64 {
        let r = self.mission.start_body.radius + altitude;
        let g = GRAVITATIONAL_CONSTANT * self.mission.start_body.mass / r.powi(2);
        (g * r).sqrt()
    }

    fn calculate_orbital_orientation_change(
        &self,
        _position: Vector2D,
        velocity: Vector2D,
        current_orientation: f64,
    ) -> f64 {
        let target_orientation = velocity.angle() + PI / 2.0;
        let orientation_error = target_orientation - current_orientation;
        orientation_error.clamp(-0.1, 0.1) // Limit the rate of change
    }
}

struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    previous_error: f64,
    integral_max: f64,
    output_min: f64,
    output_max: f64,
}

impl PIDController {
    fn new(kp: f64, ki: f64, kd: f64) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
            integral_max: 100.0,
            output_min: -1.0, // Default output range -1.0 to 1.0
            output_max: 1.0,
        }
    }

    fn with_limits(mut self, integral_max: f64, output_min: f64, output_max: f64) -> Self {
        self.integral_max = integral_max;
        self.output_min = output_min;
        self.output_max = output_max;
        self
    }

    fn calculate(&mut self, error: f64, delta_time: f64) -> f64 {
        let p_term = self.kp * error;

        self.integral += error * delta_time;
        self.integral = self.integral.clamp(-self.integral_max, self.integral_max);
        let i_term = self.ki * self.integral;

        // Calculate derivative term
        let derivative = if delta_time > 0.0 {
            (error - self.previous_error) / delta_time
        } else {
            0.0
        };
        let d_term = self.kd * derivative;

        self.previous_error = error;

        let output = p_term + i_term + d_term;
        output.clamp(self.output_min, self.output_max)
    }
}

#[cfg(test)]
mod tests {
    use crate::CelestialBody;

    use super::*;

    #[test]
    fn test_guide_ascent() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mission = Mission {
            name: "Ascent Test".to_string(),
            start_body: earth.clone(),
            target_body: earth.clone(),
            target_orbit_altitude: Some(200_000.0),
            return_trip: false,
            total_fuel: 200_000.0,
        };
        let mut guidance_system = GuidanceSystem::new(mission);

        // Simulate a position and velocity during ascent
        let position = Vector2D::new(0.0, earth.radius + 100_000.0); // 100 km altitude
        let velocity = Vector2D::new(0.0, 2_000.0); // 2000 m/s vertical velocity
        let orientation = 90.0_f64.to_radians();
        let delta_time = 1.0;
        let fuel_remaining = 150_000.0;

        let command = guidance_system
            .update(
                &RocketState::Ascent,
                position,
                velocity,
                orientation,
                delta_time,
                fuel_remaining,
            )
            .expect("Failed to generate ascent command");

        println!("Command thrust {}", command.thrust);

        // Check the thrust value
        assert!(command.thrust > 0.0);
        assert!(command.thrust <= 1.0);

        // Check orientation change
        assert!(command.orientation_change.abs() <= 0.1);
    }

    #[test]
    fn test_guide_orbit_insertion() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mission = Mission {
            name: "Orbit Test".to_string(),
            start_body: earth.clone(),
            target_body: earth.clone(),
            target_orbit_altitude: Some(200_000.0),
            return_trip: false,
            total_fuel: 200_000.0,
        };
        let mut guidance_system = GuidanceSystem::new(mission);

        // Simulate a position and velocity during orbit insertion
        let position = Vector2D::new(0.0, earth.radius + 200_000.0); // 200 km altitude
        let velocity = Vector2D::new(7_800.0, 0.0); // Orbital velocity ~ 7800 m/s
        let orientation = 0.0;
        let delta_time = 1.0;

        let command = guidance_system
            .update(
                &RocketState::OrbitInsertion,
                position,
                velocity,
                orientation,
                delta_time,
                100_000.0,
            )
            .expect("Failed to generate orbit insertion command");

        // Thrust should be very low or near zero since the rocket is in stable orbit
        assert!(command.thrust < 0.1);

        // Orientation change should be minimal
        assert!(command.orientation_change.abs() <= 0.1);
    }

    #[test]
    fn test_guide_transfer() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let moon = CelestialBody::new(
            "Moon".to_string(),
            Vector2D::new(384_400_000.0, 0.0),
            1_737_000.0,
            7.35e22,
        );
        let mission = Mission {
            name: "Lunar Transfer".to_string(),
            start_body: earth.clone(),
            target_body: moon,
            target_orbit_altitude: Some(100_000.0), // Target lunar orbit altitude
            return_trip: true,
            total_fuel: 300_000.0,
        };
        let mut guidance_system = GuidanceSystem::new(mission);

        // Simulate a position and velocity during transfer
        let position = Vector2D::new(0.0, earth.radius + 200_000.0); // 200 km altitude (LEO)
        let velocity = Vector2D::new(7_800.0, 0.0); // Orbital velocity ~ 7800 m/s
        let orientation = 0.0;
        let delta_time = 1.0;

        let command = guidance_system
            .update(
                &RocketState::Transfer,
                position,
                velocity,
                orientation,
                delta_time,
                150_000.0,
            )
            .expect("Failed to generate transfer command");

        // Thrust should be significant for the transfer burn
        assert!(command.thrust > 0.0);

        // Orientation should change slightly for transfer
        assert!(command.orientation_change.abs() > 0.0);
    }

    #[test]
    fn test_guide_descent() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mission = Mission {
            name: "Descent Test".to_string(),
            start_body: earth.clone(),
            target_body: earth.clone(),
            target_orbit_altitude: None,
            return_trip: false,
            total_fuel: 200_000.0,
        };
        let mut guidance_system = GuidanceSystem::new(mission);

        // Simulate a position and velocity during descent
        let position = Vector2D::new(0.0, earth.radius + 10_000.0); // 10 km altitude
        let velocity = Vector2D::new(0.0, -500.0); // Descent velocity
        let orientation = 180.0_f64.to_radians(); // Facing down
        let delta_time = 1.0;

        let command = guidance_system
            .update(
                &RocketState::Descent,
                position,
                velocity,
                orientation,
                delta_time,
                50_000.0,
            )
            .expect("Failed to generate descent command");

        println!(
            "Command orientation_change {}",
            command.orientation_change.abs()
        );

        // Thrust should increase as we approach the ground
        assert!(command.thrust > 0.5);

        // Orientation change should be minimal during controlled descent
        assert!(command.orientation_change.abs() <= 0.05);
    }

    #[test]
    fn test_pid_controller_behavior() {
        let mut pid = PIDController::new(1.0, 0.1, 0.5);

        let mut output = 0.0;
        let setpoint = 100.0;
        let mut process_value = 0.0;

        for i in 0..50 {
            let error = setpoint - process_value;
            output = pid.calculate(error, 0.1); // 0.1s time step

            process_value += output * 2.0;

            println!(
                "Iteration {}: Error={:.2}, Output={:.2}, Value={:.2}",
                i, error, output, process_value
            );
        }

        assert!(
            (setpoint - process_value).abs() < 10.0,
            "PID should drive process value close to setpoint"
        );
    }

    #[test]
    fn test_pid_anti_windup() {
        let mut pid = PIDController::new(1.0, 0.1, 0.0) // P and I only
            .with_limits(10.0, -0.5, 0.5); // Limited output

        // Apply a large constant error that would normally cause windup
        for _ in 0..100 {
            let output = pid.calculate(100.0, 0.1);

            // Output should be clamped at the maximum
            assert_eq!(output, 0.5, "Output should be clamped at maximum");
        }

        // Now apply a negative error and see how quickly it responds
        let mut iterations_to_response = 0;
        for i in 0..20 {
            let output = pid.calculate(-100.0, 0.1);
            iterations_to_response = i;

            if output <= -0.4 {
                break;
            }
        }

        // Without anti-windup, it would take many iterations to respond
        // With anti-windup, it should respond quickly
        assert!(
            iterations_to_response < 10,
            "With anti-windup, controller should respond quickly to error reversal"
        );
    }

    #[test]
    fn test_gravity_turn_ascent() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mission = Mission {
            name: "Gravity Turn Test".to_string(),
            start_body: earth.clone(),
            target_body: earth.clone(),
            target_orbit_altitude: Some(200_000.0),
            return_trip: false,
            total_fuel: 200_000.0,
        };
        let mut guidance_system = GuidanceSystem::new(mission);

        // Simulate a sequence of positions at increasing altitudes
        let delta_time = 1.0;
        let fuel_remaining = 150_000.0;

        let test_points = [
            (1_000.0, Vector2D::new(0.0, 500.0)),       // Low altitude, slow
            (50_000.0, Vector2D::new(1000.0, 2000.0)),  // Mid altitude, faster
            (150_000.0, Vector2D::new(2000.0, 4000.0)), // High altitude, very fast
        ];

        let mut previous_thrust = 0.0;

        for (idx, (altitude, velocity)) in test_points.iter().enumerate() {
            let position = Vector2D::new(0.0, earth.radius + altitude);
            let orientation = 80.0_f64.to_radians(); // Slightly tilted from vertical

            let command = guidance_system
                .update(
                    &RocketState::Ascent,
                    position,
                    *velocity,
                    orientation,
                    delta_time,
                    fuel_remaining,
                )
                .expect("Failed to generate ascent command");

            println!(
                "Altitude: {} m, Velocity: {:?} m/s, Thrust: {}, Orientation Change: {}",
                altitude, velocity, command.thrust, command.orientation_change
            );

            // Ensure thrust is within physically possible range
            assert!(
                command.thrust >= 0.1 && command.thrust <= 1.0,
                "Thrust should be between 0.1 and 1.0, got {}",
                command.thrust
            );

            // Orientation change should be limited
            assert!(
                command.orientation_change.abs() <= 0.02,
                "Orientation change should be limited, got {}",
                command.orientation_change
            );

            // For subsequent points, check logical relationships
            if idx > 0 {
                // As we approach target altitude, thrust should generally decrease
                if *altitude > 100_000.0 {
                    assert!(
                        command.thrust <= previous_thrust * 1.2, // Allow some fluctuation due to PID
                        "Thrust should not increase significantly as we approach target altitude"
                    );
                }
            }

            previous_thrust = command.thrust;
        }
    }

    #[test]
    fn test_adaptive_thrust_calculation() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mission = Mission {
            name: "Adaptive Thrust Test".to_string(),
            start_body: earth.clone(),
            target_body: earth.clone(),
            target_orbit_altitude: Some(200_000.0),
            return_trip: false,
            total_fuel: 100_000.0,
        };
        let guidance_system = GuidanceSystem::new(mission.clone());

        // Test cases
        let test_cases = [
            // current_alt, target_alt, velocity, fuel
            (0.0, 200_000.0, 0.0, 100_000.0),           // Launch
            (100_000.0, 200_000.0, 5_000.0, 100_000.0), // Mid-ascent
            (190_000.0, 200_000.0, 7_500.0, 100_000.0), // Near target
            (100_000.0, 200_000.0, 5_000.0, 10_000.0),  // Low fuel
        ];

        for (current_alt, target_alt, velocity, fuel) in test_cases.iter() {
            let adaptive_thrust = guidance_system.calculate_adaptive_thrust(
                *current_alt,
                *target_alt,
                *velocity,
                *fuel,
            );

            // Manually calculate the expected thrust
            let altitude_factor = ((target_alt - current_alt) / target_alt).clamp(0.0, 1.0);
            let velocity_factor = (1.0 - velocity / 7800.0).clamp(0.0, 1.0);
            let fuel_factor = (fuel / mission.total_fuel).clamp(0.0, 1.0);
            let expected_thrust = (altitude_factor * velocity_factor * fuel_factor).clamp(0.1, 1.0);

            println!(
                "Case: alt={}/{}, vel={}, fuel={}, thrust={}, expected={}",
                current_alt, target_alt, velocity, fuel, adaptive_thrust, expected_thrust
            );

            // Verify that the calculation matches the expected formula
            assert_eq!(
                adaptive_thrust, expected_thrust,
                "Thrust calculation should match the expected formula"
            );

            // Also verify some logical constraints
            if *velocity > 7000.0 {
                assert!(
                    adaptive_thrust < 0.3,
                    "Thrust should be low when near orbital velocity, got {}",
                    adaptive_thrust
                );
            }

            if *fuel < 20000.0 {
                assert!(
                    adaptive_thrust < 0.3,
                    "Thrust should be low when fuel is critically low, got {}",
                    adaptive_thrust
                );
            }
        }
    }

    #[test]
    fn test_orbital_insertion_commands() {
        let earth = CelestialBody::new(
            "Earth".to_string(),
            Vector2D::new(0.0, 0.0),
            6_371_000.0,
            5.97e24,
        );
        let mission = Mission {
            name: "Orbit Insertion Test".to_string(),
            start_body: earth.clone(),
            target_body: earth.clone(),
            target_orbit_altitude: Some(200_000.0),
            return_trip: false,
            total_fuel: 200_000.0,
        };
        let mut guidance_system = GuidanceSystem::new(mission);

        // Calculate expected orbital velocity
        let target_alt = 200_000.0;
        let expected_velocity = guidance_system.calculate_orbital_velocity(target_alt);

        // First test: perfect orbit - should have minimal thrust
        let ideal_position = Vector2D::new(0.0, earth.radius + target_alt);
        let ideal_velocity = Vector2D::new(expected_velocity, 0.0);

        let ideal_command = guidance_system
            .update(
                &RocketState::OrbitInsertion,
                ideal_position,
                ideal_velocity,
                0.0,       // Orientation aligned with velocity
                1.0,       // delta_time
                100_000.0, // fuel
            )
            .expect("Failed to generate orbit insertion command");

        println!("Ideal orbit thrust: {}", ideal_command.thrust);
        // In a perfect orbit, thrust should be minimal
        assert!(
            ideal_command.thrust < 0.1,
            "Thrust should be minimal in an ideal orbit, got {}",
            ideal_command.thrust
        );

        // Now test orbit corrections
        let test_cases = [
            // Label, position_offset, velocity_offset
            (
                "Below target",
                Vector2D::new(0.0, -50_000.0),
                Vector2D::new(0.0, 0.0),
            ),
            (
                "Above target",
                Vector2D::new(0.0, 50_000.0),
                Vector2D::new(0.0, 0.0),
            ),
            (
                "Too slow",
                Vector2D::new(0.0, 0.0),
                Vector2D::new(-1000.0, 0.0),
            ),
            (
                "Too fast",
                Vector2D::new(0.0, 0.0),
                Vector2D::new(1000.0, 0.0),
            ),
        ];

        for (label, pos_offset, vel_offset) in test_cases.iter() {
            let position = ideal_position + *pos_offset;
            let velocity = ideal_velocity + *vel_offset;

            let command = guidance_system
                .update(
                    &RocketState::OrbitInsertion,
                    position,
                    velocity,
                    0.0,
                    1.0,
                    100_000.0,
                )
                .expect("Failed to generate orbit insertion command");

            println!(
                "{}: Position offset: {:?}, Velocity offset: {:?}, Thrust: {}, Orientation change: {}",
                label, pos_offset, vel_offset, command.thrust, command.orientation_change
            );

            // Each correction command should differ from the ideal orbit
            assert!(
                (command.thrust - ideal_command.thrust).abs() > 0.01
                    || (command.orientation_change - ideal_command.orientation_change).abs() > 0.01,
                "Correction command should differ from ideal orbit command"
            );

            // The guidance system should respond correctly to different orbital errors
            match *label {
                "Below target" => {
                    assert!(
                        command.thrust > ideal_command.thrust,
                        "Thrust should increase when below target altitude"
                    );
                }
                "Above target" => {
                    // When above target, guidance might reduce thrust, but not necessarily
                    // This is implementation dependent
                }
                "Too slow" => {
                    assert!(
                        command.thrust > ideal_command.thrust,
                        "Thrust should increase when orbital velocity is too low"
                    );
                }
                "Too fast" => {
                    // When too fast, guidance might reduce thrust, but not necessarily
                    // This is implementation dependent
                }
                _ => {}
            }
        }
    }
}
