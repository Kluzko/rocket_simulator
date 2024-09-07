mod constants;
mod control;
mod telemetry_system;
mod trajectory_system;
mod utils;

use crate::{
    constants::{
        MAX_SIMULATION_TIME, ROCKET_CROSS_SECTIONAL_AREA, ROCKET_DRAG_COEFFICIENT, TIME_STEP,
    },
    control::{
        environment::Environment,
        guidance::GuidanceSystem,
        launch_stages::Stage,
        payload::Payload,
        rocket::{Rocket, RocketState},
        structure::Structure,
    },
    telemetry_system::telemetry::Telemetry,
    utils::vector2d::Vector2D,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    launch_rocket()
}

fn launch_rocket() -> Result<(), Box<dyn std::error::Error>> {
    let environment = Environment::new();
    let stages = vec![
        Stage::new(150_000.0, 50_000.0, 500.0, 180.0),
        Stage::new(50_000.0, 20_000.0, 200.0, 120.0),
    ];
    let payload = Payload::new(5_000.0, 200_000.0);
    let structure = Structure::new(stages, payload);

    let target_altitude = 200_000.0;
    let target_heading = 85.0;
    let guidance = GuidanceSystem::new(target_altitude, target_heading);

    let launch_angle = 89.5;
    let launch_site_position = Vector2D::new(0.0, 0.0);
    let initial_altitude = 10.0; // Start 10 meters above the ground
    let initial_position = Vector2D::new(
        launch_site_position.x,
        launch_site_position.y + initial_altitude,
    );

    let target_distance = 384_400_000.00; // meters to moon

    let mut rocket = Rocket::new(
        structure,
        guidance,
        environment,
        launch_angle,
        initial_position,
        ROCKET_DRAG_COEFFICIENT,
        ROCKET_CROSS_SECTIONAL_AREA,
        0.1,
        target_distance,
    );

    let mut telemetry = Telemetry::new();

    rocket.launch();

    let mut elapsed_time = 0.0;
    let mut ground_contact_time = 0.0;

    while elapsed_time < MAX_SIMULATION_TIME {
        rocket.update(TIME_STEP);
        telemetry.collect_data(&rocket);
        elapsed_time += TIME_STEP;

        if rocket.state == RocketState::Returning && rocket.kinematics.get_altitude() <= 0.001 {
            ground_contact_time += TIME_STEP;
            if ground_contact_time >= 1.0 {
                println!(
                    "Rocket has landed and remained on the ground for 1 second. Ending simulation."
                );
                break;
            }
        } else {
            ground_contact_time = 0.0;
        }
    }

    telemetry.display_data();

    Ok(())
}
