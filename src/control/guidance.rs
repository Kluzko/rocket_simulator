use super::{mission::Mission, rocket::RocketState};
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
                altitude: PIDController::new(0.9, 0.01, 0.5),
                lateral: PIDController::new(0.6, 0.01, 0.2),
                orientation: PIDController::new(1.5, 0.05, 0.4),
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
    ) -> Result<GuidanceCommand, &'static str> {
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
    ) -> Result<GuidanceCommand, &'static str> {
        let target_altitude = self.calculate_target_altitude();
        let current_altitude = position.y - self.mission.start_body.radius;
        let altitude_error = target_altitude - current_altitude;

        if altitude_error < 0.0 {
            return Ok(GuidanceCommand {
                thrust: 0.0,
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
            thrust: adaptive_thrust * thrust,
            orientation_change,
        })
    }

    fn guide_orbit(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
    ) -> Result<GuidanceCommand, &'static str> {
        let target_altitude = self.calculate_target_altitude();
        let altitude_error = target_altitude - position.y;
        let altitude_correction = self
            .pid_controllers
            .altitude
            .calculate(altitude_error, delta_time);

        let target_velocity = self.calculate_orbital_velocity(target_altitude);
        let velocity_error = target_velocity - velocity.magnitude();
        let velocity_correction = self
            .pid_controllers
            .lateral
            .calculate(velocity_error, delta_time);

        let thrust = (altitude_correction + velocity_correction).clamp(0.0, 1.0);
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
    ) -> Result<GuidanceCommand, &'static str> {
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
    ) -> Result<GuidanceCommand, &'static str> {
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
    ) -> Result<GuidanceCommand, &'static str> {
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
        (angle % two_pi + two_pi) % two_pi
    }

    fn guide_landing(
        &mut self,
        position: Vector2D,
        velocity: Vector2D,
        orientation: f64,
        delta_time: f64,
    ) -> Result<GuidanceCommand, &'static str> {
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
    ) -> Result<GuidanceCommand, &'static str> {
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
        // Implement lateral position calculation based on mission parameters
        0.0 // Placeholder
    }

    fn calculate_transfer_velocity(&self, distance: f64) -> f64 {
        // Use Hohmann transfer orbit calculation
        let r1 = self.mission.start_body.radius + self.mission.target_orbit_altitude.unwrap_or(0.0);
        let r2 = distance;
        let mu = GRAVITATIONAL_CONSTANT * self.mission.start_body.mass;
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
        let g = 9.81; // m/s^2
        let r = self.mission.start_body.radius + altitude;
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
}

impl PIDController {
    fn new(kp: f64, ki: f64, kd: f64) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
        }
    }

    fn calculate(&mut self, error: f64, delta_time: f64) -> f64 {
        self.integral += error * delta_time;
        let derivative = (error - self.previous_error) / delta_time;
        self.previous_error = error;

        self.kp * error + self.ki * self.integral + self.kd * derivative
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
}
