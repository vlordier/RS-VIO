//! GPU-accelerated ORB descriptor matching using wgpu.
//!
//! This module provides a `GpuOrbMatcher` that offloads the Hamming distance
//! calculation for ORB descriptors to the GPU using `wgpu`. This can significantly
//! speed up loop closure candidate verification for large numbers of descriptors.

use std::borrow::Cow;
use std::sync::Arc;

use log::debug;
use wgpu::{
    util::{BufferInitDescriptor, DeviceExt},
    Adapter, Buffer, BufferUsages, Device, Queue,
};

/// WGSL Compute Shader for Hamming Distance calculation
///
/// This shader takes two buffers of ORB descriptors (each a [u8; 32])
/// and computes the Hamming distance for all pairs, storing the result
/// in an output buffer.
///
/// Each ORB descriptor is 256 bits (32 bytes). We can treat this as 8 `u32` values.
/// The Hamming distance between two binary strings is the number of positions
/// at which the corresponding symbols are different. For two `u32` values,
/// this is equivalent to `(a XOR b).count_ones()`.
const SHADER_CODE: &str = r#"
// orb_matcher.wgsl
//
// A compute shader for calculating Hamming distances between ORB descriptors.
// Each descriptor is 32 bytes (256 bits), represented as 8 u32 values.

struct OrbDescriptor {
    data: array<u32, 8>, // 32 bytes = 8 * 4 bytes
};

// Input buffer for query descriptors (e.g., from current keyframe)
@group(0) @binding(0)
var<storage, read> query_descriptors: array<OrbDescriptor>;

// Input buffer for database descriptors (e.g., from historical keyframes)
@group(0) @binding(1)
var<storage, read> target_descriptors: array<OrbDescriptor>;

// Output buffer for Hamming distances (query_idx * target_count + target_idx)
@group(0) @binding(2)
var<storage, read_write> distances: array<u32>;

// Workgroup size
// Assuming 256 threads per workgroup
// Each thread will calculate one query descriptor against one target descriptor
@compute
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let query_idx = global_id.x; // Index for the current query descriptor
    let target_idx = global_id.y; // Index for the current target descriptor

    // Bounds check
    if (query_idx >= arrayLength(&query_descriptors) || target_idx >= arrayLength(&target_descriptors)) {
        return;
    }

    let query_desc = query_descriptors[query_idx];
    let target_desc = target_descriptors[target_idx];

    var hamming_distance = 0u;
    for (var i = 0u; i < 8u; i = i + 1u) {
        // XOR the corresponding u32 parts and count set bits
        hamming_distance += countOneBits(query_desc.data[i] ^ target_desc.data[i]);
    }

    // Store the calculated Hamming distance
    // The output buffer is flattened, so we need to calculate the linear index
    let output_index = query_idx * arrayLength(&target_descriptors) + target_idx;
    distances[output_index] = hamming_distance;
}
"#;

/// GPU-accelerated ORB descriptor matcher.
pub struct GpuOrbMatcher {
    device: Arc<Device>,
    queue: Arc<Queue>,
    compute_pipeline: wgpu::ComputePipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    // Store empty buffers for layout purposes or small fixed-size data
    // Actual data buffers will be created on the fly per match operation.
}

impl GpuOrbMatcher {
    /// Creates a new `GpuOrbMatcher`.
    ///
    /// Requires a `wgpu::Device` and `wgpu::Queue` to be already initialized.
    pub async fn new(device: Arc<Device>, queue: Arc<Queue>) -> Self {
        debug!("[GpuOrbMatcher] Initializing WGPU resources for ORB matching.");

        let shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("OrbMatcher Shader"),
            source: wgpu::ShaderSource::Wgsl(Cow::from(SHADER_CODE)),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("OrbMatcher Bind Group Layout"),
            entries: &[
                // Query descriptors
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Target descriptors
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Distances output
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("OrbMatcher Pipeline Layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let compute_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("OrbMatcher Compute Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader_module,
            entry_point: "main",
            compilation_options: Default::default(),
        });

        debug!("[GpuOrbMatcher] WGPU resources initialized.");

        Self {
            device,
            queue,
            compute_pipeline,
            bind_group_layout,
        }
    }

    /// Finds the best matches for each query descriptor against a set of target descriptors.
    ///
    /// Returns a vector of tuples `(query_idx, best_target_idx, min_distance)`.
    pub async fn match_descriptors(
        &self,
        query_descriptors: &[[u8; 32]],
        target_descriptors: &[[u8; 32]],
        distance_threshold: u32,
    ) -> Vec<(usize, usize, u32)> {
        if query_descriptors.is_empty() || target_descriptors.is_empty() {
            return Vec::new();
        }

        let query_count = query_descriptors.len();
        let target_count = target_descriptors.len();
        let output_size = query_count * target_count;

        debug!(
            "[GpuOrbMatcher] Matching {} query descriptors against {} target descriptors on GPU.",
            query_count, target_count
        );

        // Convert descriptors from `[[u8; 32]]` to `[[u32; 8]]` for WGSL
        let query_descriptors_u32: Vec<[u32; 8]> = query_descriptors
            .iter()
            .map(|desc_bytes| {
                let mut desc_u32 = [0u32; 8];
                for i in 0..8 {
                    desc_u32[i] = u32::from_le_bytes([
                        desc_bytes[i * 4],
                        desc_bytes[i * 4 + 1],
                        desc_bytes[i * 4 + 2],
                        desc_bytes[i * 4 + 3],
                    ]);
                }
                desc_u32
            })
            .collect();

        let target_descriptors_u32: Vec<[u32; 8]> = target_descriptors
            .iter()
            .map(|desc_bytes| {
                let mut desc_u32 = [0u32; 8];
                for i in 0..8 {
                    desc_u32[i] = u32::from_le_bytes([
                        desc_bytes[i * 4],
                        desc_bytes[i * 4 + 1],
                        desc_bytes[i * 4 + 2],
                        desc_bytes[i * 4 + 3],
                    ]);
                }
                desc_u32
            })
            .collect();

        // Create buffers for input and output
        let query_buffer = self.device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Query Descriptors Buffer"),
            contents: bytemuck::cast_slice(&query_descriptors_u32),
            usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC,
        });
        let target_buffer = self.device.create_buffer_init(&BufferInitDescriptor {
            label: Some("Target Descriptors Buffer"),
            contents: bytemuck::cast_slice(&target_descriptors_u32),
            usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC,
        });
        let output_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Distances Output Buffer"),
            size: (output_size * std::mem::size_of::<u32>()) as u64,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // Create bind group
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: query_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: target_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: output_buffer.as_entire_binding(),
                },
            ],
            label: Some("OrbMatcher Bind Group"),
        });

        // Create command encoder and compute pass
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("OrbMatcher Command Encoder"),
            });
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("OrbMatcher Compute Pass"),
                timestamp_writes: None, // Can be used for profiling
            });
            compute_pass.set_pipeline(&self.compute_pipeline);
            compute_pass.set_bind_group(0, &bind_group, &[]);
            compute_pass.dispatch_workgroups(query_count as u32, target_count as u32, 1);
        }

        // Submit commands
        self.queue.submit(Some(encoder.finish()));

        // Read back results asynchronously
        let distances_slice = output_buffer.slice(0..output_buffer.size());
        let (sender, receiver) = futures_intrusive::channel::shared::oneshot_channel();
        distances_slice.map_async(wgpu::MapMode::Read, move |result| {
            sender.send(result).unwrap();
        });

        // Wait for the GPU to finish computation
        self.device.poll(wgpu::Maintain::Wait);

        // Await the result
        let read_result = receiver.receive().await;
        if read_result.is_err() || read_result.unwrap().is_err() {
            log::error!("[GpuOrbMatcher] Failed to read GPU results.");
            return Vec::new();
        }

        let distance_data = distances_slice.get_mapped_range();
        let distances_raw: Cow<[u32]> = bytemuck::cast_slice(&distance_data).into();

        let mut matches = Vec::new();
        for query_idx in 0..query_count {
            let mut min_dist = u32::MAX;
            let mut best_target_idx: Option<usize> = None;

            for target_idx in 0..target_count {
                let distance = distances_raw[query_idx * target_count + target_idx];
                if distance < min_dist && distance <= distance_threshold {
                    min_dist = distance;
                    best_target_idx = Some(target_idx);
                }
            }

            if let Some(best_idx) = best_target_idx {
                matches.push((query_idx, best_idx, min_dist));
            }
        }

        drop(distance_data); // Unmap the buffer
        output_buffer.unmap();

        debug!(
            "[GpuOrbMatcher] GPU matching completed. Found {} matches.",
            matches.len()
        );

        matches
    }
}

// Helper to initialize WGPU globally or per-thread/context
pub async fn init_wgpu() -> Option<(Arc<Device>, Arc<Queue>, Adapter)> {
    debug!("[GpuOrbMatcher] Initializing WGPU adapter, device, and queue.");

    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        dx12_shader_compiler: Default::default(),
        flags: wgpu::InstanceFlags::from_build_config(),
        // gles_minor_version: wgpu::Gles3MinorVersion::Automatic,
    });

    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            // Request an adapter which can process compute shaders
            compatible_surface: None,
        })
        .await
        .expect("Failed to find an appropriate adapter");

    let (device, queue) = adapter
        .request_device(
            &wgpu::DeviceDescriptor {
                label: None,
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::downlevel_defaults(),
            },
            None,
        )
        .await
        .expect("Failed to create device");

    debug!("[GpuOrbMatcher] WGPU adapter, device, and queue initialized.");
    Some((Arc::new(device), Arc::new(queue), adapter))
}
