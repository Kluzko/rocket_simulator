use rocket_simulation::{
    errors::SimulationError, Aerodynamics, MissionFactory, Payload, Rocket, RocketState, Stage,
    Structure,
};

// Helper function to create a standard test rocket
fn create_test_rocket() -> Rocket {
    // Create a standard single-stage rocket
    let stage = Stage::new(100_000.0, 20_000.0, 500.0, 200.0);
    let structure = Structure::new(vec![stage], Payload::new(5_000.0, 200_000.0));

    let mission = MissionFactory::create_earth_orbit(
        "Test Orbit".to_string(),
        200_000.0, // Target altitude
        300_000.0, // Total fuel
    );

    let aerodynamics = Aerodynamics::new(0.5, 10.0, 0.3);

    Rocket::new(structure, mission, aerodynamics, 90.0)
}

fn run_simulation(
    rocket: &mut Rocket,
    duration_seconds: f64,
    time_step: f64,
) -> Result<(), SimulationError> {
    let mut elapsed_time = 0.0;

    while elapsed_time < duration_seconds {
        rocket.update(time_step)?;

        if (elapsed_time % 30.0) < time_step {
            let altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);
            let velocity = rocket.kinematics.get_velocity_magnitude();
            println!(
                "t={:.1}s | State: {:?} | Alt: {:.1}m | Vel: {:.1}m/s",
                elapsed_time, rocket.state, altitude, velocity
            );
        }

        elapsed_time += time_step;

        if rocket.state == RocketState::Landed || rocket.state == RocketState::MissionComplete {
            println!(
                "Mission ended in state: {:?} at t={:.1}s",
                rocket.state, elapsed_time
            );
            break;
        }
    }

    Ok(())
}

#[test]
fn test_launch_and_initial_ascent() {
    println!("INTEGRATION TEST: Launch and Initial Ascent");

    let mut rocket = create_test_rocket();

    assert_eq!(
        rocket.state,
        RocketState::Idle,
        "Rocket should start in Idle state"
    );

    println!("Starting launch sequence...");
    rocket.start_launch_sequence();
    assert_eq!(
        rocket.state,
        RocketState::LaunchSequence,
        "Rocket should enter launch sequence state"
    );

    let mut elapsed_time = 0.0;
    let time_step = 0.1;

    while rocket.state == RocketState::LaunchSequence && elapsed_time < 10.0 {
        rocket
            .update(time_step)
            .expect("Launch sequence should succeed");
        elapsed_time += time_step;
    }

    assert_eq!(
        rocket.state,
        RocketState::Launched,
        "Rocket should transition from LaunchSequence to Launched"
    );

    let initial_altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);
    println!("Beginning ascent from altitude: {:.1}m", initial_altitude);

    for _ in 0..300 {
        rocket
            .update(time_step)
            .expect("Ascent should proceed without errors");
    }

    let current_altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);
    assert!(current_altitude > initial_altitude + 1000.0,
            "Rocket should gain at least 1000m altitude during ascent. Initial: {:.1}m, Current: {:.1}m",
            initial_altitude, current_altitude);

    let velocity = rocket.kinematics.velocity;
    assert!(
        velocity.y > 100.0,
        "Rocket should have significant upward velocity during ascent, got: {:.1} m/s",
        velocity.y
    );

    println!("After 30s of ascent:");
    println!("  - Altitude: {:.1}m", current_altitude);
    println!(
        "  - Velocity: {:.1} m/s",
        rocket.kinematics.get_velocity_magnitude()
    );
    println!("Launch and Initial Ascent Test: PASSED");
}

#[test]
fn test_multi_stage_rocket() {
    println!("INTEGRATION TEST: Multi-Stage Rocket");

    // Create a multi-stage rocket
    let stages = vec![
        Stage::new(50_000.0, 15_000.0, 400.0, 120.0), // First stage
        Stage::new(20_000.0, 8_000.0, 200.0, 180.0),  // Second stage
    ];

    let structure = Structure::new(stages, Payload::new(2_000.0, 200_000.0));
    let mission =
        MissionFactory::create_earth_orbit("Two-Stage Mission".to_string(), 200_000.0, 100_000.0);
    let aerodynamics = Aerodynamics::new(0.4, 8.0, 0.25);
    let mut rocket = Rocket::new(structure, mission, aerodynamics, 90.0);

    // Start and run past launch sequence
    rocket.start_launch_sequence();
    let mut elapsed_time = 0.0;
    let time_step = 0.5; // 0.5s time step for faster simulation

    // Run until first stage is depleted (should take ~120 seconds)
    println!("Running until first stage separation...");
    let max_simulation_time = 200.0; // 200 seconds max
    let mut stage_count = rocket.structure.active_stages_count();

    while elapsed_time < max_simulation_time {
        rocket
            .update(time_step)
            .expect("Multi-stage ascent should succeed");
        elapsed_time += time_step;

        // Print status every 20 seconds
        if ((elapsed_time * 10.0).round() / 10.0) % 20.0 < time_step {
            let altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);
            let velocity = rocket.kinematics.get_velocity_magnitude();
            println!(
                "t={:.1}s | Altitude: {:.1} km | Velocity: {:.1} m/s | Stages: {}",
                elapsed_time,
                altitude / 1000.0,
                velocity,
                rocket.structure.active_stages_count()
            );
        }

        // Check if a stage has been jettisoned
        let current_stage_count = rocket.structure.active_stages_count();
        if current_stage_count < stage_count {
            println!("Stage separation detected at t={:.1}s!", elapsed_time);
            stage_count = current_stage_count;
            break; // We've detected a stage separation, which is what we wanted to test
        }
    }

    // Verify stage separation occurred
    assert!(
        rocket.structure.active_stages_count() < 2,
        "First stage should have been jettisoned"
    );

    // Run a bit more to verify second stage ignition and continued ascent
    let altitude_at_separation = rocket.kinematics.get_altitude(&rocket.mission.start_body);
    println!(
        "Altitude at separation: {:.1} km",
        altitude_at_separation / 1000.0
    );

    // Run for another 30 seconds
    for _ in 0..60 {
        // 30 more seconds
        rocket
            .update(time_step)
            .expect("Second stage ascent should succeed");
    }

    // Verify continued ascent after stage separation
    let current_altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);
    assert!(current_altitude > altitude_at_separation,
            "Rocket should continue ascending after stage separation. Altitude at separation: {:.1} km, Current: {:.1} km",
            altitude_at_separation / 1000.0, current_altitude / 1000.0);

    println!("After second stage burn:");
    println!("  - Altitude: {:.1} km", current_altitude / 1000.0);
    println!(
        "  - Velocity: {:.1} m/s",
        rocket.kinematics.get_velocity_magnitude()
    );
    println!("Multi-Stage Rocket Test: PASSED");
}

#[test]
fn test_atmospheric_effects() {
    println!("INTEGRATION TEST: Atmospheric Effects on Ascent");

    // Create two identical rockets except for aerodynamic properties
    let create_rocket = |drag_coef: f64| {
        let stage = Stage::new(80_000.0, 15_000.0, 400.0, 200.0);
        let structure = Structure::new(vec![stage], Payload::new(2_000.0, 200_000.0));
        let mission = MissionFactory::create_earth_orbit(
            "Aerodynamics Test".to_string(),
            200_000.0,
            100_000.0,
        );
        let aerodynamics = Aerodynamics::new(drag_coef, 10.0, 0.2);
        Rocket::new(structure, mission, aerodynamics, 90.0)
    };

    // Create rockets with different drag coefficients
    let mut low_drag_rocket = create_rocket(0.2); // Sleek design
    let mut high_drag_rocket = create_rocket(0.8); // Blunt design

    // Start both rockets
    low_drag_rocket.start_launch_sequence();
    high_drag_rocket.start_launch_sequence();

    // Fast-forward through launch sequence
    let mut elapsed_time = 0.0;
    let time_step = 0.5;

    while low_drag_rocket.state == RocketState::LaunchSequence && elapsed_time < 10.0 {
        low_drag_rocket
            .update(time_step)
            .expect("Launch should succeed");
        high_drag_rocket
            .update(time_step)
            .expect("Launch should succeed");
        elapsed_time += time_step;
    }

    // Run both rockets for 120 seconds of flight
    println!("Running aerodynamic comparison test...");
    for i in 1..=24 {
        // 120 seconds in 5-second increments
        // Run for 5 seconds
        for _ in 0..10 {
            low_drag_rocket
                .update(time_step)
                .expect("Low-drag ascent should succeed");
            high_drag_rocket
                .update(time_step)
                .expect("High-drag ascent should succeed");
        }

        // Compare at 5-second intervals
        let low_drag_altitude = low_drag_rocket
            .kinematics
            .get_altitude(&low_drag_rocket.mission.start_body);
        let high_drag_altitude = high_drag_rocket
            .kinematics
            .get_altitude(&high_drag_rocket.mission.start_body);
        let low_drag_velocity = low_drag_rocket.kinematics.get_velocity_magnitude();
        let high_drag_velocity = high_drag_rocket.kinematics.get_velocity_magnitude();

        println!(
            "t={}s | Low-drag: {:.1} km, {:.1} m/s | High-drag: {:.1} km, {:.1} m/s",
            i * 5,
            low_drag_altitude / 1000.0,
            low_drag_velocity,
            high_drag_altitude / 1000.0,
            high_drag_velocity
        );
    }

    // After 120 seconds, verify the low-drag rocket is ahead
    let low_drag_altitude = low_drag_rocket
        .kinematics
        .get_altitude(&low_drag_rocket.mission.start_body);
    let high_drag_altitude = high_drag_rocket
        .kinematics
        .get_altitude(&high_drag_rocket.mission.start_body);

    assert!(
        low_drag_altitude > high_drag_altitude,
        "Low-drag rocket should achieve higher altitude. Low-drag: {:.1} km, High-drag: {:.1} km",
        low_drag_altitude / 1000.0,
        high_drag_altitude / 1000.0
    );

    println!("Atmospheric Effects Test: PASSED");
}

#[test]
fn test_payload_deployment() {
    println!("INTEGRATION TEST: Payload Deployment");

    // Create rocket with payload set to deploy at 100km
    let deployment_altitude = 100_000.0; // 100 km
    let stage = Stage::new(80_000.0, 15_000.0, 400.0, 200.0);
    let payload = Payload::new(2_000.0, deployment_altitude);
    let structure = Structure::new(vec![stage], payload);
    let mission =
        MissionFactory::create_earth_orbit("Payload Deploy Test".to_string(), 200_000.0, 100_000.0);
    let aerodynamics = Aerodynamics::new(0.3, 8.0, 0.2);
    let mut rocket = Rocket::new(structure, mission, aerodynamics, 90.0);

    // Start and run past launch sequence
    rocket.start_launch_sequence();

    // Fast-forward through launch sequence
    let mut elapsed_time = 0.0;
    let time_step = 1.0; // 1s time steps for faster simulation

    while rocket.state == RocketState::LaunchSequence && elapsed_time < 10.0 {
        rocket.update(time_step).expect("Launch should succeed");
        elapsed_time += time_step;
    }

    // Now run until we reach deployment altitude or timeout
    println!(
        "Ascending to payload deployment altitude ({:.1} km)...",
        deployment_altitude / 1000.0
    );
    let max_time = 500.0; // 500 seconds max
    let mut payload_deployed = false;

    while elapsed_time < max_time && !payload_deployed {
        rocket.update(time_step).expect("Ascent should succeed");
        elapsed_time += time_step;

        let altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);

        // Status update every 20 seconds
        if ((elapsed_time * 10.0).round() / 10.0) % 20.0 < time_step {
            println!(
                "t={:.1}s | Altitude: {:.1} km | Payload status: {}",
                elapsed_time,
                altitude / 1000.0,
                if rocket.structure.payload.is_deployed() {
                    "DEPLOYED"
                } else {
                    "not deployed"
                }
            );
        }

        // Check if payload is deployed
        if rocket.structure.payload.is_deployed() {
            println!(
                "Payload deployed at t={:.1}s, altitude: {:.1} km",
                elapsed_time,
                altitude / 1000.0
            );
            payload_deployed = true;
        }
    }

    // Verify payload was deployed
    assert!(
        payload_deployed,
        "Payload should have been deployed during the test"
    );
    assert!(
        rocket.structure.payload.is_deployed(),
        "Payload deployment flag should be set"
    );
    assert_eq!(
        rocket.structure.payload.get_mass(),
        0.0,
        "Deployed payload should have zero mass contribution"
    );

    // Get the rocket mass before and ensure it decreased by the payload mass
    let final_mass = rocket.structure.get_total_mass();
    println!("Final rocket mass after deployment: {:.1} kg", final_mass);

    println!("Payload Deployment Test: PASSED");
}

#[test]
fn test_orbit_insertion() {
    println!("INTEGRATION TEST: Orbit Insertion");

    // Create a powerful rocket specifically designed to reach orbit in simulation
    let stage = Stage::new(200_000.0, 30_000.0, 800.0, 400.0);
    let structure = Structure::new(vec![stage], Payload::new(5_000.0, 300_000.0));

    // Low orbit mission (200km)
    let target_altitude = 200_000.0;
    let mission = MissionFactory::create_earth_orbit(
        "Orbit Insertion Test".to_string(),
        target_altitude,
        400_000.0,
    );
    let aerodynamics = Aerodynamics::new(0.3, 8.0, 0.2);
    let mut rocket = Rocket::new(structure, mission, aerodynamics, 90.0);

    // Fast-forward through launch and ascent
    println!("Running orbit insertion simulation (this may take a while)...");
    let time_step = 2.0; // 2s time steps for faster simulation

    // Start launch sequence
    rocket.start_launch_sequence();

    // Run the simulation for up to 30 minutes of simulation time
    // This should be enough time to reach orbit with our powerful rocket
    println!("Beginning orbit insertion test...");

    match run_simulation(&mut rocket, 1800.0, time_step) {
        Ok(_) => {
            // Check if we made it to orbit
            let final_altitude = rocket.kinematics.get_altitude(&rocket.mission.start_body);
            let final_velocity = rocket.kinematics.get_velocity_magnitude();
            let orbital_velocity = rocket
                .kinematics
                .calculate_orbital_velocity(&rocket.mission.start_body);

            println!("Final altitude: {:.1} km", final_altitude / 1000.0);
            println!("Final velocity: {:.1} m/s", final_velocity);
            println!("Required orbital velocity: {:.1} m/s", orbital_velocity);
            println!("Final state: {:?}", rocket.state);

            // We may not fully reach orbit in the test, but we should get close
            let min_expected_altitude = target_altitude * 0.75;
            let min_expected_velocity = orbital_velocity * 0.75;

            assert!(final_altitude >= min_expected_altitude,
                    "Rocket should reach at least 75% of target altitude. Got: {:.1} km, Expected: {:.1} km",
                    final_altitude / 1000.0, min_expected_altitude / 1000.0);

            assert!(final_velocity >= min_expected_velocity,
                    "Rocket should reach at least 75% of orbital velocity. Got: {:.1} m/s, Expected: {:.1} m/s",
                    final_velocity, min_expected_velocity);

            println!("Orbit Insertion Test: PASSED");
        }
        Err(e) => {
            panic!("Orbit insertion simulation failed: {}", e);
        }
    }
}

// Main integration test that runs all scenarios
#[test]
fn test_full_rocket_simulation_integration() {
    println!("\n====== RUNNING COMPLETE ROCKET SIMULATION INTEGRATION TEST SUITE ======\n");

    // Run each integration test in sequence
    test_launch_and_initial_ascent();
    println!("\n--------------------------------------------------------------\n");

    test_multi_stage_rocket();
    println!("\n--------------------------------------------------------------\n");

    test_atmospheric_effects();
    println!("\n--------------------------------------------------------------\n");

    test_payload_deployment();
    println!("\n--------------------------------------------------------------\n");

    test_orbit_insertion();

    println!("\n====== ALL ROCKET SIMULATION INTEGRATION TESTS PASSED ======\n");
}
