import numpy as np

def read_data(file_path):
    return np.loadtxt(file_path, delimiter=',')

data_predict = read_data('data_predict.txt')
data_camera = read_data('data_camera.txt')
data_imu = read_data('data_imu.txt')
data_update = read_data('data_update.txt')

print("Data loaded successfully.")
import g2o
import open3d as o3d

# Create optimizer
optimizer = g2o.SparseOptimizer()
solver = g2o.BlockSolverSE2(g2o.LinearSolverDenseSE2())
solver = g2o.OptimizationAlgorithmLevenberg(solver)
optimizer.set_algorithm(solver)

# Function to add a vertex (pose) to the graph
def add_vertex(optimizer, id, pose, fixed=False):
    vertex = g2o.VertexSE2()
    vertex.set_id(id)
    vertex.set_estimate(pose)
    vertex.set_fixed(fixed)
    optimizer.add_vertex(vertex)

# Function to add an edge (constraint) to the graph
def add_edge(optimizer, vertices, measurement, information=np.eye(3)):
    edge = g2o.EdgeSE2()
    for i, v in enumerate(vertices):
        edge.set_vertex(i, optimizer.vertex(v))
    edge.set_measurement(measurement)
    edge.set_information(information)
    optimizer.add_edge(edge)

# ICP function to match and find transformation between two point clouds
def icp(source, target):
    source_pcd = o3d.geometry.PointCloud()
    target_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source)
    target_pcd.points = o3d.utility.Vector3dVector(target)

    threshold = 2.0
    trans_init = np.eye(4)

    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    return reg_p2p.transformation

# Initialize the graph with the first pose
initial_pose = g2o.SE2(data_update[0, 0], data_update[0, 1], data_update[0, 2])
add_vertex(optimizer, 0, initial_pose, fixed=True)

# Add the rest of the poses and edges
for i in range(1, len(data_update)):
    current_pose = g2o.SE2(data_update[i, 0], data_update[i, 1], data_update[i, 2])
    add_vertex(optimizer, i, current_pose)

    # Add edge between consecutive poses (odometry)
    odometry = g2o.SE2(data_predict[i, 0], data_predict[i, 1], data_predict[i, 2])
    add_edge(optimizer, [i-1, i], odometry)

    # Loop closure detection with ICP
    for j in range(i):
        if np.linalg.norm(data_update[i, :2] - data_update[j, :2]) < 2.0:  # Larger initial threshold for candidates
            # Prepare point clouds for ICP
            source_points = data_camera[i].reshape(-1, 3)  # Replace with actual points from the camera data
            target_points = data_camera[j].reshape(-1, 3)  # Replace with actual points from the camera data
            
            # Perform ICP
            transformation = icp(source_points, target_points)
            rotation_angle = np.arctan2(transformation[1, 0], transformation[0, 0])
            translation = transformation[:2, 3]

            loop_closure = g2o.SE2(translation[0], translation[1], rotation_angle)
            add_edge(optimizer, [j, i], loop_closure)

# Optimize the pose graph
optimizer.initialize_optimization()
optimizer.optimize(10000)

# Extract the optimized poses
optimized_poses = [optimizer.vertex(i).estimate() for i in range(len(data_update))]

# Convert poses to a numpy array for visualization
optimized_poses_np = np.array([[pose.translation()[0], pose.translation()[1], pose.rotation().angle()] for pose in optimized_poses])

# Visualization
import matplotlib.pyplot as plt

plt.plot(optimized_poses_np[:, 0], optimized_poses_np[:, 1], label='Optimized Path')
plt.scatter(data_update[:, 0], data_update[:, 1], label='Original Path', color='red', s=10)
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Pose Graph Optimization with Loop Closure using ICP')
plt.show()
