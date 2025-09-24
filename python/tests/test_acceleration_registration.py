import numpy as np
import pytest

from rko_lio.lio import LIOConfig
from rko_lio.lio_pipeline import LIOPipeline


@pytest.fixture
def identity_extrinsics():
    return np.eye(4)

def test_triangle_acceleration_profile(identity_extrinsics):
    config = LIOConfig()
    pipeline = LIOPipeline(config, identity_extrinsics, identity_extrinsics)
    
    # Time parameters
    imu_freq = 100  # Hz
    lidar_freq = 10  # Hz
    total_time = 2.0  # seconds
    imu_dt = 1.0 / imu_freq
    
    imu_steps = int(total_time * imu_freq)
    
    # Generate triangle acceleration profile for X axis (m/s²)
    # 0->1 m/s² in 1s, then 1->0 m/s² in next 1s
    imu_times = np.linspace(0, total_time, imu_steps)
    
    # 0 to 1 in 0.5s, 1 to -1 in next 1s, -1 back to 0 in last 0.5s
    acceleration_profile = np.piecewise(
    imu_times,
    [imu_times < 0.5,
     (imu_times >= 0.5) & (imu_times < 1.5),
     imu_times >= 1.5],
    [lambda t: 2.0 * t,                # 0 to 1 over 0.5s
     lambda t: 1 - 2.0 * (t - 0.5),   # 1 to -1 over 1s
     lambda t: -1 + 2.0 * (t - 1.5)]  # -1 to 0 over 0.5s
).astype(np.float32)

    __import__('ipdb').set_trace()
    
    # Acceleration vector for each imu sample (X only)
    imu_accelerations = np.zeros((imu_steps, 3), dtype=np.float32)
    imu_accelerations[:, 0] = acceleration_profile
    imu_angular_velocity = np.zeros((imu_steps, 3), dtype=np.float32)
    
    # Integrate acceleration twice to get position (zero initial velocity and position)
    velocities = np.cumsum(imu_accelerations * imu_dt, axis=0)
    positions = np.cumsum(velocities * imu_dt, axis=0)
    
    # Generate 3D cube grid points (-10 to 10 every 1m)
    x = y = z = np.arange(-10, 11, 1)
    X, Y, Z = np.meshgrid(x, y, z, indexing="ij")
    cube_points = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1).astype(np.float32)

    # LiDAR timestamps at 10 Hz (every 10 IMU samples)
    lidar_dt = 1.0 / lidar_freq
    
    for i in range(imu_steps):
        # Add IMU data at 100 Hz
        pipeline.add_imu(time=imu_times[i], angular_velocity=imu_angular_velocity[i], acceleration=imu_accelerations[i])
        
        # Every 10 IMU samples, generate a LiDAR point cloud at 10 Hz
        if i % (imu_freq // lidar_freq) == 0:
            # Corresponding lidar index
            lidar_index = i // (imu_freq // lidar_freq)
            # Compute current position
            pos = positions[i]
            
            # Create pose matrix with translation pos (no rotation)
            pose_mat = np.eye(4, dtype=np.float32)
            pose_mat[:3, 3] = pos
            
            # Transform the cube points from world to sensor frame by inverse of pose
            pose_inv = np.linalg.inv(pose_mat)
            points_transformed = (pose_inv[:3, :3] @ cube_points.T).T + pose_inv[:3, 3]
            
            # Create timestamps array for points (lidar timestamp)
            timestamps = np.full(points_transformed.shape[0], lidar_index * lidar_dt, dtype=np.float32)
            
            pipeline.add_lidar(points_transformed, timestamps)
    
    # Expected final pose from integrated position, identity rotation
    expected_pose = np.eye(4, dtype=np.float32)
    expected_pose[:3, 3] = positions[-1]
    actual_pose = pipeline.lio.pose()
    
    assert np.allclose(actual_pose, expected_pose, atol=1e-3)

