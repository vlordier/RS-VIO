//! Command-line interface for manual calibration workflow
//!
//! This example demonstrates how to perform a complete calibration session
//! with operator guidance and quality gates.

use rs_vio::calibration::manual_workflow::{
    ManualCalibrationWorkflow, WorkflowConfig,
};
use std::io::{self, Write};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== RS-VIO Manual Calibration Workflow ===\n");

    // Get platform ID
    print!("Enter platform ID (e.g., drone serial number): ");
    io::stdout().flush()?;
    let mut platform_id = String::new();
    io::stdin().read_line(&mut platform_id)?;
    let platform_id = platform_id.trim().to_string();

    // Create workflow
    let mut workflow = ManualCalibrationWorkflow::new(platform_id.clone());

    // Get metadata
    print!("Operator ID: ");
    io::stdout().flush()?;
    let mut operator_id = String::new();
    io::stdin().read_line(&mut operator_id)?;

    print!("Location/Facility: ");
    io::stdout().flush()?;
    let mut location = String::new();
    io::stdin().read_line(&mut location)?;

    print!("Temperature (°C, optional): ");
    io::stdout().flush()?;
    let mut temp_str = String::new();
    io::stdin().read_line(&mut temp_str)?;
    let _temperature = temp_str.trim().parse::<f64>().ok();

    print!("Weather conditions: ");
    io::stdout().flush()?;
    let mut weather = String::new();
    io::stdin().read_line(&mut weather)?;

    // Start workflow
    println!("\n{}", workflow.start()?);
    println!("\n{}", workflow.get_guidance());

    // Simulation: Perform IMU calibration
    println!("\n--- Simulating IMU Calibration ---");
    println!("(In production, this would read real IMU data)");
    
    // Simulate collecting samples for each pose
    for pose_idx in 0..6 {
        println!("\nPose {}: Ready? (press Enter)", pose_idx);
        let mut _ready = String::new();
        io::stdin().read_line(&mut _ready)?;

        // Simulate collecting samples
        println!("Collecting samples for pose {}...", pose_idx);
        for i in 0..100 {
            // Simulated IMU data (in reality, this comes from sensors)
            // In real implementation: workflow.add_imu_sample(sample)?;
            
            if i % 20 == 0 {
                print!(".");
                io::stdout().flush()?;
            }
        }
        println!(" Done!");
    }

    println!("\n✓ IMU calibration complete");
    println!("  Gyro bias: [0.001, -0.002, 0.0005] rad/s");
    println!("  Accel bias: [0.05, -0.03, 0.08] m/s²");
    println!("  Quality: PASS");

    // Quality gates summary
    println!("\n=== Quality Gates Summary ===");
    println!("✓ IMU calibration: PASS");
    println!("○ Left camera intrinsics: SKIP (demo)");
    println!("○ Right camera intrinsics: SKIP (demo)");
    println!("○ Stereo extrinsics: SKIP (demo)");
    println!("○ Time offset: SKIP (demo)");

    // Save session
    print!("\nSave calibration? (y/n): ");
    io::stdout().flush()?;
    let mut save_choice = String::new();
    io::stdin().read_line(&mut save_choice)?;

    if save_choice.trim().to_lowercase() == "y" {
        match workflow.save() {
            Ok(path) => println!("✓ Calibration saved to: {}", path.display()),
            Err(e) => println!("✗ Failed to save: {}", e),
        }
    }

    // List previous sessions
    println!("\n=== Previous Calibration Sessions ===");
    match ManualCalibrationWorkflow::list_sessions(
        &platform_id,
        &std::path::PathBuf::from("./calibration_sessions"),
    ) {
        Ok(sessions) => {
            if sessions.is_empty() {
                println!("No previous sessions found.");
            } else {
                for (i, session_path) in sessions.iter().enumerate() {
                    println!(
                        "{}. {}",
                        i + 1,
                        session_path.file_name().unwrap().to_string_lossy()
                    );
                }
            }
        }
        Err(e) => println!("Could not list sessions: {}", e),
    }

    println!("\nCalibration workflow complete!");
    Ok(())
}
