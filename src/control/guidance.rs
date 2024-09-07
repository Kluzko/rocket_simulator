struct PIDController {
    kp: f64, // Proportional gain
    ki: f64, // Integral gain
    kd: f64, // Derivative gain
    previous_error: f64,
    integral: f64,
    output: f64,
}

impl PIDController {
    fn new(kp: f64, ki: f64, kd: f64) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            previous_error: 0.0,
            integral: 0.0,
            output: 0.0,
        }
    }

    fn calculate(&mut self, error: f64, delta_time: f64) -> f64 {
        self.integral = (self.integral + error * delta_time).clamp(-10.0, 10.0);
        let derivative = (error - self.previous_error) / delta_time;
        self.previous_error = error;
        self.output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        self.output.clamp(-1.0, 1.0)
    }
}

pub struct GuidanceSystem {
    pub target_altitude: f64,
    pub current_orientation: f64,
    pub current_altitude: f64,
    pub target_heading: f64,
    pitch_controller: PIDController,
    yaw_controller: PIDController,
    throttle_controller: PIDController,
    previous_altitude: f64,
    altitude_rate: f64,
}

impl GuidanceSystem {
    pub fn new(target_altitude: f64, target_heading: f64) -> Self {
        GuidanceSystem {
            target_altitude,
            current_orientation: 0.0,
            current_altitude: 0.0,
            target_heading,
            pitch_controller: PIDController::new(0.46, 0.08, 0.5),
            yaw_controller: PIDController::new(0.1, 0.01, 0.05),
            throttle_controller: PIDController::new(0.05, 0.001, 0.01),
            previous_altitude: 0.0,
            altitude_rate: 0.0,
        }
    }

    pub fn update(
        &mut self,
        current_altitude: f64,
        current_orientation: f64,
        current_velocity: f64,
        sensor_data: Option<f64>,
        delta_time: f64,
    ) {
        self.altitude_rate = (current_altitude - self.previous_altitude) / delta_time;
        self.previous_altitude = self.current_altitude;
        self.current_altitude = current_altitude;
        self.current_orientation = current_orientation;

        if let Some(sensor_orientation) = sensor_data {
            self.current_orientation = sensor_orientation;
        }

        let target_pitch = self.get_target_pitch(current_altitude);
        let pitch_error = target_pitch - self.current_orientation;
        self.pitch_controller.calculate(pitch_error, delta_time);

        let throttle = self.calculate_throttle(current_altitude, current_velocity);
        self.throttle_controller
            .calculate(throttle - 0.5, delta_time);

        let orientation_error = normalize_angle(self.target_heading - self.current_orientation);
        if orientation_error.abs() > (5.0_f64).to_radians() {
            self.yaw_controller.calculate(orientation_error, delta_time);
        }
    }

    pub fn calculate_control_commands(&self) -> (f64, f64, f64) {
        let pitch_command = self.pitch_controller.output;
        let throttle = self.calculate_throttle(self.current_altitude, self.altitude_rate);
        let yaw_command = self.yaw_controller.output;

        (pitch_command, throttle, yaw_command)
    }

    fn get_target_pitch(&self, altitude: f64) -> f64 {
        let initial_pitch = 85.0_f64.to_radians();
        let final_pitch = 45.0_f64.to_radians();
        let pitch_change = initial_pitch - final_pitch;
        let altitude_ratio = (altitude / self.target_altitude).min(1.0);
        initial_pitch - pitch_change * altitude_ratio
    }

    fn calculate_throttle(&self, altitude: f64, velocity: f64) -> f64 {
        let altitude_error = self.target_altitude - altitude;
        let altitude_factor = altitude_error / self.target_altitude;
        let velocity_factor = 1.0 - (velocity / 100.0).min(1.0);
        (0.5 + 0.5 * altitude_factor * velocity_factor).clamp(0.1, 1.0)
    }

    pub fn get_orientation_change(&self, delta_time: f64) -> f64 {
        let desired_orientation = 90.0_f64.to_radians(); // Vertical
        let orientation_error = desired_orientation - self.current_orientation;
        orientation_error.clamp(-0.5, 0.5) * delta_time
    }
}

fn normalize_angle(angle: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    ((angle % two_pi) + two_pi) % two_pi
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    fn test_pid_controller() {
        let mut pid = PIDController::new(0.1, 0.01, 0.05);
        let mut error = 100.0;
        let delta_time = 0.1;

        for _ in 0..10 {
            let output = pid.calculate(error, delta_time);
            error -= output * 10.0; // Simulate error reduction
        }

        assert!(error.abs() < 50.0, "PID should reduce error over time");
    }

    #[test]
    fn test_get_target_pitch() {
        let guidance = GuidanceSystem::new(1000.0, 90.0);

        assert_relative_eq!(
            guidance.get_target_pitch(0.0),
            85.0_f64.to_radians(),
            epsilon = 1e-6
        );
        assert_relative_eq!(
            guidance.get_target_pitch(500.0),
            65.0_f64.to_radians(),
            epsilon = 1e-6
        );
        assert_relative_eq!(
            guidance.get_target_pitch(1000.0),
            45.0_f64.to_radians(),
            epsilon = 1e-6
        );
        assert_relative_eq!(
            guidance.get_target_pitch(1500.0),
            45.0_f64.to_radians(),
            epsilon = 1e-6
        );
    }

    #[test]
    fn test_calculate_throttle() {
        let guidance = GuidanceSystem::new(1000.0, 90.0);

        assert!(
            guidance.calculate_throttle(0.0, 0.0) > 0.9,
            "Throttle should be high at start"
        );
        let mid_throttle = guidance.calculate_throttle(500.0, 50.0);
        assert!(
            mid_throttle > 0.5 && mid_throttle < 0.9,
            "Throttle should be moderate mid-flight"
        );
        assert!(
            guidance.calculate_throttle(1000.0, 100.0) < 0.7,
            "Throttle should be relatively low near target altitude"
        );
    }

    #[test]
    fn test_orientation_change() {
        let mut guidance = GuidanceSystem::new(1000.0, 90.0);
        let delta_time = 0.1;

        guidance.current_orientation = 0.0;
        assert!(
            guidance.get_orientation_change(delta_time) > 0.0,
            "Should rotate clockwise when below 90 degrees"
        );

        guidance.current_orientation = PI;
        assert!(
            guidance.get_orientation_change(delta_time) < 0.0,
            "Should rotate counter-clockwise when above 90 degrees"
        );

        guidance.current_orientation = PI / 2.0;
        assert_relative_eq!(
            guidance.get_orientation_change(delta_time),
            0.0,
            epsilon = 1e-6
        );
    }

    #[test]
    fn test_guidance_update() {
        let mut guidance = GuidanceSystem::new(1000.0, 90.0);
        let delta_time = 0.1;

        guidance.update(100.0, 85.0_f64.to_radians(), 10.0, None, delta_time);

        assert!(
            guidance.altitude_rate > 0.0,
            "Altitude rate should be positive"
        );
        assert!(
            guidance.pitch_controller.output != 0.0,
            "Pitch controller should have non-zero output"
        );
        assert!(
            guidance.throttle_controller.output != 0.0,
            "Throttle controller should have non-zero output"
        );
        assert!(
            guidance.yaw_controller.output.abs() < 0.5,
            "Yaw controller output should be small when close to target heading"
        );
    }

    #[test]
    fn test_guidance_full_flight() {
        let mut guidance = GuidanceSystem::new(1000.0, 90.0);
        let delta_time = 0.1;
        let mut altitude = 0.0;
        let mut velocity = 0.0;
        let mut orientation = 85.0_f64.to_radians();

        for i in 0..1000 {
            guidance.update(altitude, orientation, velocity, None, delta_time);
            let (pitch, throttle, yaw) = guidance.calculate_control_commands();

            // Simple physics simulation
            let acceleration = (throttle * 20.0) - 9.81; // Simplified thrust and gravity
            velocity += acceleration * delta_time;
            altitude += velocity * delta_time;
            orientation += (pitch - orientation) * 0.1 * delta_time;

            if i % 100 == 0 {
                println!("Step {}: Altitude: {:.2}, Velocity: {:.2}, Orientation: {:.2}, Pitch: {:.4}, Throttle: {:.4}, Yaw: {:.4}",
                         i, altitude, velocity, orientation.to_degrees(), pitch, throttle, yaw);
            }

            if altitude >= 1000.0 || altitude < 0.0 {
                break;
            }
        }

        println!(
            "Final state: Altitude: {:.2}, Velocity: {:.2}, Orientation: {:.2}",
            altitude,
            velocity,
            orientation.to_degrees()
        );

        assert_relative_eq!(altitude, 1000.0, epsilon = 10.0, max_relative = 0.01);
        assert!(
            velocity.abs() < 50.0,
            "Final velocity should be relatively low. Final velocity: {}",
            velocity
        );
        assert_relative_eq!(orientation.to_degrees(), 45.0, epsilon = 5.0);
    }
}
