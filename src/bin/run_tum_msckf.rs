use clap::Parser;
use env_logger::{Builder, Env};
use image::ImageReader;
use log::{error, info, LevelFilter};
use rand::rngs::StdRng;
use rand::SeedableRng;
use rs_vio::msckf::MsckfPipeline;
use rs_vio::{Config, datasets::{ImageData, ImuData}};
use rs_vio::types::{Matrix3x3, UnitQuaternion};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process;

fn main() {
    let _rng = StdRng::seed_from_u64(42);

    Builder::from_env(Env::default().default_filter_or("info"))
        .filter_module("rerun", LevelFilter::Warn)
        .format_timestamp_millis()
        .init();

    let args = Args::parse();
    let config = match Config::load(&args.config_file) {
        Ok(config) => config,
        Err(err) => {
            error!("failed to load config: {}", err);
            process::exit(-1);
        }
    };
    let image_data = match load_image_timestamps(&args.dataset_path) {
        Ok(data) if !data.is_empty() => data,
        Ok(_) => {
            error!("no images found in dataset");
            process::exit(-1);
        }
        Err(err) => {
            error!("failed to load image timestamps: {}", err);
            process::exit(-1);
        }
    };
    let imu_data_all = if args.no_imu {
        Vec::new()
    } else {
        load_imu_data(&args.dataset_path)
    };

    let mut pipeline = MsckfPipeline::new(config);
    let mut previous_frame_timestamp = 0_i64;

    for (frame_index, image) in image_data.iter().enumerate() {
        let (left_image, right_image) = match (
            load_image(&args.dataset_path, &image.filename, 0),
            load_image(&args.dataset_path, &image.filename, 1),
        ) {
            (Ok(left), Ok(right)) => (left, right),
            (Err(err), _) | (_, Err(err)) => {
                error!("failed to load frame {}: {}", frame_index, err);
                process::exit(-1);
            }
        };

        let imu_slice = if imu_data_all.is_empty() {
            None
        } else {
            Some(get_imu_data_between_frames(
                &imu_data_all,
                previous_frame_timestamp,
                image.timestamp,
            ))
        };

        match pipeline.process_frame(left_image, right_image, image.timestamp, imu_slice) {
            Ok(summary) => {
                if frame_index % 20 == 0 {
                    info!(
                        "[MSCKF] frame={} init={} mode={:?} clone={:?} pairs={} mature={} triangulated={} seed_fail={} dlt_fail={} cond_fail={} depth_fail={} reproj_fail={} accepted={} rejected={} state_dim={}",
                        summary.frame_id,
                        summary.initialized,
                        summary.mode,
                        summary.clone_created,
                        summary.paired_observations,
                        summary.mature_tracks,
                        summary.triangulated_tracks,
                        summary.triangulation_seed_failures,
                        summary.triangulation_dlt_failures,
                        summary.triangulation_condition_failures,
                        summary.triangulation_depth_failures,
                        summary.triangulation_reprojection_failures,
                        summary.accepted_tracks,
                        summary.rejected_tracks,
                        summary.state_dim,
                    );
                }
            }
            Err(err) => {
                error!("pipeline failed on frame {}: {}", frame_index, err);
                process::exit(-1);
            }
        }

        previous_frame_timestamp = image.timestamp;
    }

    if let Err(err) = save_trajectory(&pipeline.pose_log, &args.dataset_path) {
        error!("failed to save trajectory: {}", err);
        process::exit(-1);
    }

    info!(
        "[MSCKF] processed {} frames and recorded {} poses",
        image_data.len(),
        pipeline.pose_log.len(),
    );
}

#[derive(Parser, Debug)]
#[command(name = "tum_msckf")]
#[command(about = "TUM-VI MSCKF dataset runner")]
struct Args {
    #[arg(help = "Path to configuration file")]
    config_file: String,

    #[arg(help = "Path to TUM-VI dataset directory")]
    dataset_path: String,

    #[arg(long, help = "Disable IMU data")]
    no_imu: bool,
}

fn load_image_timestamps(dataset_path: &str) -> anyhow::Result<Vec<ImageData>> {
    let data_dir = Path::new(dataset_path).join("mav0/cam0/data");
    let mut image_data = Vec::new();
    for entry in std::fs::read_dir(&data_dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.extension().is_some_and(|ext| ext == "png") {
            if let Some(stem) = path.file_stem().and_then(|value| value.to_str()) {
                if let Ok(timestamp) = stem.parse::<i64>() {
                    image_data.push(ImageData {
                        timestamp,
                        filename: format!("{}.png", stem),
                    });
                }
            }
        }
    }
    image_data.sort_by_key(|image| image.timestamp);
    Ok(image_data)
}

fn load_image(dataset_path: &str, filename: &str, cam_id: u32) -> anyhow::Result<Vec<u8>> {
    let cam_folder = if cam_id == 0 { "cam0" } else { "cam1" };
    let full_path = Path::new(dataset_path)
        .join("mav0")
        .join(cam_folder)
        .join("data")
        .join(filename);
    let image = ImageReader::open(&full_path)?.decode()?.to_luma8();
    Ok(image.into_raw())
}

fn load_imu_data(dataset_path: &str) -> Vec<ImuData> {
    let imu_csv = Path::new(dataset_path).join("mav0/imu0/data.csv");
    let file = match File::open(&imu_csv) {
        Ok(file) => file,
        Err(err) => {
            error!("cannot open IMU file {}: {}", imu_csv.display(), err);
            return Vec::new();
        }
    };

    let mut samples = Vec::new();
    for line in BufReader::new(file).lines().map_while(Result::ok) {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let parts = line.split(',').collect::<Vec<_>>();
        if parts.len() < 7 {
            continue;
        }
        if let (Ok(timestamp), Ok(gx), Ok(gy), Ok(gz), Ok(ax), Ok(ay), Ok(az)) = (
            parts[0].trim().parse::<i64>(),
            parts[1].trim().parse::<f64>(),
            parts[2].trim().parse::<f64>(),
            parts[3].trim().parse::<f64>(),
            parts[4].trim().parse::<f64>(),
            parts[5].trim().parse::<f64>(),
            parts[6].trim().parse::<f64>(),
        ) {
            samples.push(ImuData {
                timestamp,
                gyro: [gx, gy, gz],
                accel: [ax, ay, az],
            });
        }
    }
    samples.sort_by_key(|sample| sample.timestamp);
    samples
}

fn get_imu_data_between_frames(imu_all: &[ImuData], prev_ts: i64, curr_ts: i64) -> &[ImuData] {
    let start = imu_all.partition_point(|sample| sample.timestamp <= prev_ts);
    let end = imu_all[start..].partition_point(|sample| sample.timestamp <= curr_ts);
    &imu_all[start..start + end]
}

fn save_trajectory(pose_log: &[(i64, rs_vio::types::Matrix4x4)], dataset_path: &str) -> anyhow::Result<()> {
    use std::io::Write;

    let output_path = Path::new(dataset_path).join("estimated_trajectory_msckf.txt");
    let mut file = std::fs::File::create(&output_path)?;
    writeln!(file, "# timestamp tx ty tz qx qy qz qw")?;

    for &(timestamp_ns, ref pose) in pose_log {
        let translation = pose.fixed_view::<3, 1>(0, 3);
        let rotation: Matrix3x3 = pose.fixed_view::<3, 3>(0, 0).into();
        let quaternion = UnitQuaternion::from_matrix(&rotation);
        writeln!(
            file,
            "{:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9} {:.9}",
            timestamp_ns as f64 * 1e-9,
            translation[0],
            translation[1],
            translation[2],
            quaternion.i,
            quaternion.j,
            quaternion.k,
            quaternion.w,
        )?;
    }

    info!("[MSCKF] wrote trajectory to {}", output_path.display());
    Ok(())
}