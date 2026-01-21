//! Frontier-based exploration strategy for multi-drone SLAM
//!
//! Implements coordinated exploration using frontier detection and
//! utility-based task allocation for efficient coverage.

use crate::multi_drone::boids::Vector3;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// Cell in occupancy grid
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellState {
    Unknown,
    Free,
    Occupied,
}

/// 3D occupancy grid for mapping
pub struct OccupancyGrid {
    /// Grid resolution (meters per cell)
    pub resolution: f32,
    /// Grid origin (minimum corner)
    pub origin: Vector3,
    /// Grid dimensions (cells)
    pub size_x: usize,
    pub size_y: usize,
    pub size_z: usize,
    /// Cell states (flattened 3D array)
    cells: Vec<CellState>,
}

impl OccupancyGrid {
    /// Create new occupancy grid
    pub fn new(origin: Vector3, resolution: f32, size_x: usize, size_y: usize, size_z: usize) -> Self {
        let total_cells = size_x * size_y * size_z;
        Self {
            resolution,
            origin,
            size_x,
            size_y,
            size_z,
            cells: vec![CellState::Unknown; total_cells],
        }
    }

    /// Convert world coordinates to grid index
    pub fn world_to_grid(&self, point: &Vector3) -> Option<(usize, usize, usize)> {
        let x = ((point.x - self.origin.x) / self.resolution) as isize;
        let y = ((point.y - self.origin.y) / self.resolution) as isize;
        let z = ((point.z - self.origin.z) / self.resolution) as isize;

        if x >= 0 && y >= 0 && z >= 0 
            && (x as usize) < self.size_x 
            && (y as usize) < self.size_y 
            && (z as usize) < self.size_z
        {
            Some((x as usize, y as usize, z as usize))
        } else {
            None
        }
    }

    /// Convert grid index to world coordinates (cell center)
    pub fn grid_to_world(&self, x: usize, y: usize, z: usize) -> Vector3 {
        Vector3::new(
            self.origin.x + (x as f32 + 0.5) * self.resolution,
            self.origin.y + (y as f32 + 0.5) * self.resolution,
            self.origin.z + (z as f32 + 0.5) * self.resolution,
        )
    }

    /// Get cell state
    pub fn get_cell(&self, x: usize, y: usize, z: usize) -> CellState {
        if x >= self.size_x || y >= self.size_y || z >= self.size_z {
            return CellState::Unknown;
        }
        let idx = x + y * self.size_x + z * self.size_x * self.size_y;
        self.cells[idx]
    }

    /// Set cell state
    pub fn set_cell(&mut self, x: usize, y: usize, z: usize, state: CellState) {
        if x >= self.size_x || y >= self.size_y || z >= self.size_z {
            return;
        }
        let idx = x + y * self.size_x + z * self.size_x * self.size_y;
        self.cells[idx] = state;
    }

    /// Mark point as free
    pub fn mark_free(&mut self, point: &Vector3) {
        if let Some((x, y, z)) = self.world_to_grid(point) {
            self.set_cell(x, y, z, CellState::Free);
        }
    }

    /// Mark point as occupied
    pub fn mark_occupied(&mut self, point: &Vector3) {
        if let Some((x, y, z)) = self.world_to_grid(point) {
            self.set_cell(x, y, z, CellState::Occupied);
        }
    }

    /// Check if cell is on frontier (free with unknown neighbors)
    pub fn is_frontier(&self, x: usize, y: usize, z: usize) -> bool {
        if self.get_cell(x, y, z) != CellState::Free {
            return false;
        }

        // Check 6-connected neighbors
        let neighbors = [
            (x.wrapping_sub(1), y, z),
            (x + 1, y, z),
            (x, y.wrapping_sub(1), z),
            (x, y + 1, z),
            (x, y, z.wrapping_sub(1)),
            (x, y, z + 1),
        ];

        for (nx, ny, nz) in neighbors {
            if nx < self.size_x && ny < self.size_y && nz < self.size_z {
                if self.get_cell(nx, ny, nz) == CellState::Unknown {
                    return true;
                }
            }
        }

        false
    }
}

/// Frontier cluster for exploration
#[derive(Debug, Clone)]
pub struct Frontier {
    /// Unique frontier ID
    pub id: usize,
    /// Centroid position
    pub centroid: Vector3,
    /// Number of cells in frontier
    pub size: usize,
    /// Estimated information gain
    pub information_gain: f32,
}

impl Frontier {
    pub fn new(id: usize, centroid: Vector3, size: usize) -> Self {
        Self {
            id,
            centroid,
            size,
            information_gain: size as f32, // Simple estimate
        }
    }
}

/// Configuration for exploration strategy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExplorationConfig {
    /// Minimum frontier size (cells)
    pub min_frontier_size: usize,
    /// Weight for information gain in utility
    pub information_gain_weight: f32,
    /// Weight for distance in utility
    pub distance_weight: f32,
    /// Maximum exploration range
    pub max_range: f32,
}

impl Default for ExplorationConfig {
    fn default() -> Self {
        Self {
            min_frontier_size: 5,
            information_gain_weight: 1.0,
            distance_weight: 0.5,
            max_range: 50.0,
        }
    }
}

/// Exploration task assignment
#[derive(Debug, Clone)]
pub struct ExplorationTask {
    pub drone_id: usize,
    pub frontier_id: usize,
    pub target_position: Vector3,
    pub utility: f32,
}

/// Frontier-based exploration manager
pub struct ExplorationManager {
    config: ExplorationConfig,
    /// Occupancy grid for the environment
    grid: OccupancyGrid,
    /// Detected frontiers
    frontiers: Vec<Frontier>,
    /// Frontier ID counter
    next_frontier_id: usize,
    /// Task assignments per drone
    assignments: HashMap<usize, ExplorationTask>,
}

impl ExplorationManager {
    /// Create new exploration manager
    pub fn new(config: ExplorationConfig, grid: OccupancyGrid) -> Self {
        Self {
            config,
            grid,
            frontiers: Vec::new(),
            next_frontier_id: 0,
            assignments: HashMap::new(),
        }
    }

    /// Update occupancy grid with new observation
    pub fn update_grid(&mut self, free_points: &[Vector3], occupied_points: &[Vector3]) {
        for point in free_points {
            self.grid.mark_free(point);
        }
        for point in occupied_points {
            self.grid.mark_occupied(point);
        }
    }

    /// Detect frontiers in the occupancy grid
    pub fn detect_frontiers(&mut self) {
        self.frontiers.clear();
        let mut visited = HashSet::new();

        for z in 0..self.grid.size_z {
            for y in 0..self.grid.size_y {
                for x in 0..self.grid.size_x {
                    if visited.contains(&(x, y, z)) {
                        continue;
                    }

                    if self.grid.is_frontier(x, y, z) {
                        // Cluster frontier cells
                        let cluster = self.cluster_frontier(x, y, z, &mut visited);
                        
                        if cluster.len() >= self.config.min_frontier_size {
                            // Compute centroid
                            let mut sum_x = 0.0;
                            let mut sum_y = 0.0;
                            let mut sum_z = 0.0;

                            for (cx, cy, cz) in &cluster {
                                let pos = self.grid.grid_to_world(*cx, *cy, *cz);
                                sum_x += pos.x;
                                sum_y += pos.y;
                                sum_z += pos.z;
                            }

                            let n = cluster.len() as f32;
                            let centroid = Vector3::new(sum_x / n, sum_y / n, sum_z / n);

                            let frontier = Frontier::new(self.next_frontier_id, centroid, cluster.len());
                            self.next_frontier_id += 1;
                            self.frontiers.push(frontier);
                        }
                    }
                }
            }
        }
    }

    /// Cluster connected frontier cells using flood fill
    fn cluster_frontier(
        &self,
        start_x: usize,
        start_y: usize,
        start_z: usize,
        visited: &mut HashSet<(usize, usize, usize)>,
    ) -> Vec<(usize, usize, usize)> {
        let mut cluster = Vec::new();
        let mut stack = vec![(start_x, start_y, start_z)];

        while let Some((x, y, z)) = stack.pop() {
            if visited.contains(&(x, y, z)) {
                continue;
            }

            if !self.grid.is_frontier(x, y, z) {
                continue;
            }

            visited.insert((x, y, z));
            cluster.push((x, y, z));

            // Add neighbors
            let neighbors = [
                (x.wrapping_sub(1), y, z),
                (x + 1, y, z),
                (x, y.wrapping_sub(1), z),
                (x, y + 1, z),
                (x, y, z.wrapping_sub(1)),
                (x, y, z + 1),
            ];

            for (nx, ny, nz) in neighbors {
                if nx < self.grid.size_x 
                    && ny < self.grid.size_y 
                    && nz < self.grid.size_z
                    && !visited.contains(&(nx, ny, nz))
                {
                    stack.push((nx, ny, nz));
                }
            }
        }

        cluster
    }

    /// Compute utility of a frontier for a drone
    pub fn compute_utility(&self, drone_position: &Vector3, frontier: &Frontier) -> f32 {
        let distance = drone_position.distance_to(&frontier.centroid);
        
        if distance > self.config.max_range {
            return 0.0;
        }

        let info_gain = frontier.information_gain * self.config.information_gain_weight;
        let dist_cost = distance * self.config.distance_weight;

        // Utility = gain - cost
        (info_gain - dist_cost).max(0.0)
    }

    /// Allocate tasks to drones (greedy assignment)
    pub fn allocate_tasks(&mut self, drone_positions: &[(usize, Vector3)]) {
        self.assignments.clear();

        // Create priority list of (drone_id, frontier_id, utility)
        let mut candidates = Vec::new();

        for (drone_id, position) in drone_positions {
            for frontier in &self.frontiers {
                let utility = self.compute_utility(position, frontier);
                if utility > 0.0 {
                    candidates.push((*drone_id, frontier.id, frontier.centroid, utility));
                }
            }
        }

        // Sort by utility (descending)
        candidates.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap());

        // Greedy assignment: assign best available frontier to each drone
        let mut assigned_drones = HashSet::new();
        let mut assigned_frontiers = HashSet::new();

        for (drone_id, frontier_id, target, utility) in candidates {
            if assigned_drones.contains(&drone_id) || assigned_frontiers.contains(&frontier_id) {
                continue;
            }

            let task = ExplorationTask {
                drone_id,
                frontier_id,
                target_position: target,
                utility,
            };

            self.assignments.insert(drone_id, task);
            assigned_drones.insert(drone_id);
            assigned_frontiers.insert(frontier_id);
        }
    }

    /// Get task for a specific drone
    pub fn get_task(&self, drone_id: usize) -> Option<&ExplorationTask> {
        self.assignments.get(&drone_id)
    }

    /// Get number of detected frontiers
    pub fn num_frontiers(&self) -> usize {
        self.frontiers.len()
    }

    /// Get occupancy grid
    pub fn grid(&self) -> &OccupancyGrid {
        &self.grid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_occupancy_grid_creation() {
        let grid = OccupancyGrid::new(Vector3::zero(), 0.1, 10, 10, 10);
        assert_eq!(grid.size_x, 10);
        assert_eq!(grid.size_y, 10);
        assert_eq!(grid.size_z, 10);
    }

    #[test]
    fn test_world_to_grid() {
        let grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        let idx = grid.world_to_grid(&Vector3::new(5.5, 5.5, 5.5));
        assert_eq!(idx, Some((5, 5, 5)));
    }

    #[test]
    fn test_grid_to_world() {
        let grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        let pos = grid.grid_to_world(5, 5, 5);
        assert!((pos.x - 5.5).abs() < 0.001);
        assert!((pos.y - 5.5).abs() < 0.001);
        assert!((pos.z - 5.5).abs() < 0.001);
    }

    #[test]
    fn test_cell_state() {
        let mut grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        assert_eq!(grid.get_cell(5, 5, 5), CellState::Unknown);
        
        grid.set_cell(5, 5, 5, CellState::Free);
        assert_eq!(grid.get_cell(5, 5, 5), CellState::Free);
        
        grid.set_cell(5, 5, 5, CellState::Occupied);
        assert_eq!(grid.get_cell(5, 5, 5), CellState::Occupied);
    }

    #[test]
    fn test_mark_free_occupied() {
        let mut grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        
        grid.mark_free(&Vector3::new(5.5, 5.5, 5.5));
        assert_eq!(grid.get_cell(5, 5, 5), CellState::Free);
        
        grid.mark_occupied(&Vector3::new(6.5, 6.5, 6.5));
        assert_eq!(grid.get_cell(6, 6, 6), CellState::Occupied);
    }

    #[test]
    fn test_is_frontier() {
        let mut grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        
        // Free cell with unknown neighbors is frontier
        grid.set_cell(5, 5, 5, CellState::Free);
        assert!(grid.is_frontier(5, 5, 5));
        
        // Surrounded by free cells - not frontier
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let x = (5_isize + dx) as usize;
                    let y = (5_isize + dy) as usize;
                    let z = (5_isize + dz) as usize;
                    grid.set_cell(x, y, z, CellState::Free);
                }
            }
        }
        assert!(!grid.is_frontier(5, 5, 5));
    }

    #[test]
    fn test_frontier_creation() {
        let frontier = Frontier::new(0, Vector3::new(1.0, 2.0, 3.0), 10);
        assert_eq!(frontier.id, 0);
        assert_eq!(frontier.size, 10);
        assert_eq!(frontier.information_gain, 10.0);
    }

    #[test]
    fn test_exploration_config_defaults() {
        let config = ExplorationConfig::default();
        assert!(config.min_frontier_size > 0);
        assert!(config.max_range > 0.0);
    }

    #[test]
    fn test_exploration_manager_creation() {
        let grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        let manager = ExplorationManager::new(ExplorationConfig::default(), grid);
        assert_eq!(manager.num_frontiers(), 0);
    }

    #[test]
    fn test_update_grid() {
        let grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        let mut manager = ExplorationManager::new(ExplorationConfig::default(), grid);
        
        let free = vec![Vector3::new(1.5, 1.5, 1.5)];
        let occupied = vec![Vector3::new(2.5, 2.5, 2.5)];
        
        manager.update_grid(&free, &occupied);
        
        assert_eq!(manager.grid().get_cell(1, 1, 1), CellState::Free);
        assert_eq!(manager.grid().get_cell(2, 2, 2), CellState::Occupied);
    }

    #[test]
    fn test_compute_utility() {
        let grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        let manager = ExplorationManager::new(ExplorationConfig::default(), grid);
        
        let drone_pos = Vector3::new(0.0, 0.0, 0.0);
        let frontier = Frontier::new(0, Vector3::new(5.0, 0.0, 0.0), 10);
        
        let utility = manager.compute_utility(&drone_pos, &frontier);
        assert!(utility > 0.0);
    }

    #[test]
    fn test_allocate_tasks() {
        let grid = OccupancyGrid::new(Vector3::zero(), 1.0, 10, 10, 10);
        let mut manager = ExplorationManager::new(ExplorationConfig::default(), grid);
        
        // Manually add a frontier
        manager.frontiers.push(Frontier::new(0, Vector3::new(5.0, 5.0, 5.0), 10));
        
        let drones = vec![(0, Vector3::new(0.0, 0.0, 0.0))];
        manager.allocate_tasks(&drones);
        
        let task = manager.get_task(0);
        assert!(task.is_some());
    }
}
