use rocket_simulation::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let stages = vec![
        Stage::new(150_000.0, 50_000.0, 500.0, 180.0),
        Stage::new(50_000.0, 20_000.0, 200.0, 120.0),
    ];

    let payload = Payload::new(5_000.0, 200_000.0);
    let structure = Structure::new(stages, payload);

    let target_altitude = 200_000.0;
    let mission =
        MissionFactory::create_earth_orbit("Earth Orbit".to_string(), target_altitude, 300_000.0);

    let launch_angle = 0.0;
    let aerodynamics = Aerodynamics::new(0.2, 10.0, 0.3);

    let mut rocket = Rocket::new(structure, mission, aerodynamics, launch_angle);

    rocket.start_launch_sequence();

    let mut telemetry = Telemetry::new();

    let mut elapsed_time = 0.0;
    while elapsed_time < MAX_SIMULATION_TIME {
        match rocket.update(TIME_STEP) {
            Ok(_) => {
                telemetry.collect_data(&rocket, TIME_STEP);
                elapsed_time += TIME_STEP;

                if rocket.state == RocketState::Landed {
                    println!("Rocket has landed. Ending simulation.");
                    break;
                }
            }
            Err(e) => {
                println!("Error during simulation step: {}", e);
                break;
            }
        }
    }

    telemetry.display_data();

    Ok(())
}
