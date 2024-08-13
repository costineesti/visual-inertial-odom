import g2o
import numpy as np
import csv
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def normalize_angle(angle):
    angle_norm = angle % 360.0
    if angle_norm < 0:
        angle_norm += 360.0
    return angle_norm

def deg2rad(deg):
    return deg * np.pi / 180

def load_data(filename):
    try:
        with open(filename, 'r') as file:
            reader = csv.reader(file)
            data = []
            for row in reader:
                data.append([float(row[0]), float(row[1]), float(row[2])])
    except FileNotFoundError:
        print(f"File {filename} not found.")
        return None
    except ValueError:
        print(f"Invalid data format in {filename}.")
        return None
    return data

def calculate_transformation(pose1, pose2):
    pose1 = np.array(pose1)
    pose2 = np.array(pose2)
    delta_position = pose2[:2] - pose1[:2]
    delta_theta = pose2[2] - pose1[2]
    
    if abs(delta_theta) > 180:
        if delta_theta > 0:
            delta_theta -= 360
        else:
            delta_theta += 360

    delta_theta = deg2rad(normalize_angle(delta_theta))
    translation = np.array([*delta_position, 0]).reshape(3, 1)
    rotation = R.from_euler('z', delta_theta).as_matrix()
    isometry = g2o.Isometry3d(rotation, translation)
    return g2o.SE3Quat(isometry.rotation(), isometry.translation())

data_predict = load_data('data_predict.txt')

optimizer = g2o.SparseOptimizer()
solver = g2o.BlockSolverSE3(g2o.LinearSolverDenseSE3())
solver = g2o.OptimizationAlgorithmLevenberg(solver)
optimizer.set_algorithm(solver)

# Create vertices
for i in range(len(data_predict)):
    yaw = data_predict[i][2]
    r = R.from_euler('z', yaw, degrees=True)
    quaternion = r.as_quat()
    quaternion = np.roll(quaternion, shift=-1)
    quaternion = g2o.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    translation = np.array([data_predict[i][0], data_predict[i][1], 0]).reshape(3, 1)
    pose = g2o.SE3Quat(quaternion, translation)
    
    v_se3 = g2o.VertexSE3Expmap()
    v_se3.set_estimate(pose)
    v_se3.set_id(i)
    optimizer.add_vertex(v_se3)

# Create edges
for i in range(len(data_predict) - 1):
    edge = g2o.EdgeSE3Expmap()
    edge.set_vertex(0, optimizer.vertex(i))
    edge.set_vertex(1, optimizer.vertex(i+1))
    edge.set_measurement(calculate_transformation(data_predict[i], data_predict[i+1]))
    
    information_matrix = np.identity(6)
    information_matrix[:3, :3] = np.identity(3)  # translation
    information_matrix[3:, 3:] = np.identity(3) * 1e-6  # rotation
    edge.set_information(information_matrix)
    
    optimizer.add_edge(edge)

# Add loop closure edge
edge = g2o.EdgeSE3Expmap()
edge.set_vertex(0, optimizer.vertex(0))
edge.set_vertex(1, optimizer.vertex(len(data_predict) - 1))
edge.set_measurement(calculate_transformation(data_predict[-1], data_predict[0]))
information_matrix = np.identity(6)
#information_matrix[:3, :3] *= 2  # translation
information_matrix[3:, 3:] = np.identity(3) * 1e-6  # rotation
edge.set_information(information_matrix)
optimizer.add_edge(edge)

# Optimize
optimizer.initialize_optimization()
optimizer.optimize(1000)

# Get optimized poses
final_poses = []
for i in range(len(data_predict)):
    pose = optimizer.vertex(i).estimate()
    final_poses.append([pose.translation()[0], pose.translation()[1]])

initial_poses = np.array([[data[0], data[1]] for data in data_predict])
final_poses = np.array(final_poses)

# Plot results
plt.figure()
plt.plot(initial_poses[:, 0], initial_poses[:, 1], 'o-', label='Initial')
plt.plot(final_poses[:, 0], final_poses[:, 1], 'o-', label='Final')
plt.legend()
plt.show()
