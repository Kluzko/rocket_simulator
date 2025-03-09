use crate::control::rocket::{Rocket, RocketState};
use crate::utils::vector2d::Vector2D;

pub struct Telemetry {
    pub log: Vec<String>,
    max_velocity: f64,
    max_altitude: f64,
    min_fuel: f64,
    max_acceleration: f64,
    state_times: Vec<(RocketState, f64)>,
    simulation_time: f64,
}

impl Telemetry {
    pub fn new() -> Self {
        Telemetry {
            log: Vec::new(),
            max_velocity: 0.0,
            max_altitude: 0.0,
            min_fuel: f64::MAX,
            max_acceleration: 0.0,
            state_times: Vec::new(),
            simulation_time: 0.0,
        }
    }

    fn format_vector2d(vec: &Vector2D, precision: usize) -> String {
        format!(
            "x = {:.precision$} m, y = {:.precision$} m",
            vec.x,
            vec.y,
            precision = precision
        )
    }

    fn format_time(elapsed_time: f64) -> String {
        if elapsed_time >= 3600.0 {
            let hours = (elapsed_time / 3600.0).floor();
            let minutes = ((elapsed_time % 3600.0) / 60.0).floor();
            let seconds = elapsed_time % 60.0;
            format!("{:.0}h {:.0}m {:.2}s", hours, minutes, seconds)
        } else if elapsed_time >= 60.0 {
            let minutes = (elapsed_time / 60.0).floor();
            let seconds = elapsed_time % 60.0;
            format!("{:.0}m {:.2}s", minutes, seconds)
        } else {
            format!("{:.2}s", elapsed_time)
        }
    }

    fn format_altitude(altitude: f64) -> String {
        if altitude >= 1000.0 {
            format!("{:.2} km", altitude / 1000.0)
        } else {
            format!("{:.2} m", altitude)
        }
    }

    pub fn collect_data(&mut self, rocket: &Rocket, delta_time: f64) {
        self.simulation_time += delta_time;
        let velocity_magnitude = rocket.kinematics.get_velocity_magnitude();
        let altitude = rocket
            .kinematics
            .get_altitude(&rocket.mission.start_body.clone());
        let acceleration_magnitude = rocket.kinematics.get_acceleration_magnitude();
        let fuel = rocket.structure.get_total_fuel();

        // Update key metrics
        if velocity_magnitude > self.max_velocity {
            self.max_velocity = velocity_magnitude;
        }
        if altitude > self.max_altitude {
            self.max_altitude = altitude;
        }
        if fuel < self.min_fuel {
            self.min_fuel = fuel;
        }
        if acceleration_magnitude > self.max_acceleration {
            self.max_acceleration = acceleration_magnitude;
        }

        let formatted_time = Self::format_time(self.simulation_time);
        let data = format!(
            "Time: {}\n\
                 Position: {}\n\
                 Velocity: {} (Magnitude: {:.2} m/s)\n\
                 Acceleration: {} (Magnitude: {:.2} m/s²)\n\
                 Thrust: {:.2} N\n\
                 Fuel: {:.2} kg\n\
                 Air Density: {:.4} kg/m³\n\
                 Gravity: {:.4} m/s²\n\
                 Total Mass: {:.2} kg\n\
                 Orientation: {:.2}°\n",
            formatted_time,
            Self::format_vector2d(&rocket.kinematics.position, 2),
            Self::format_vector2d(&rocket.kinematics.velocity, 2),
            velocity_magnitude,
            Self::format_vector2d(&rocket.kinematics.acceleration, 2),
            acceleration_magnitude,
            rocket
                .structure
                .stages
                .first()
                .map_or(0.0, |stage| stage.propulsion.get_thrust()),
            fuel,
            rocket.environment.air_density,
            rocket.environment.gravity,
            rocket.structure.get_total_mass(),
            rocket.kinematics.get_orientation().to_degrees()
        );
        self.log.push(data);

        // Track state transitions
        if let Some((last_state, _)) = self.state_times.last() {
            if *last_state != rocket.state {
                self.state_times
                    .push((rocket.state.clone(), self.simulation_time));
            }
        } else {
            self.state_times
                .push((rocket.state.clone(), self.simulation_time));
        }
    }

    pub fn display_data(&self) {
        println!("--- Telemetry Data ---");
        for entry in &self.log {
            println!("{}", entry);
        }
        println!("--- End of Telemetry ---");

        println!("\n--- Simulation Summary ---");
        println!("Max Velocity: {:.2} m/s", self.max_velocity);
        println!("Max Altitude: {}", Self::format_altitude(self.max_altitude));
        println!("Min Fuel: {:.2} kg", self.min_fuel);
        println!("Max Acceleration: {:.2} m/s²", self.max_acceleration);

        println!("\n--- State Transitions ---");
        for (state, time) in &self.state_times {
            println!("State {:?} reached at: {}", state, Self::format_time(*time));
        }
    }
}
