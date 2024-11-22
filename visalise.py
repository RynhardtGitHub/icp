import numpy as np
import matplotlib.pyplot as plt

# Function to apply a transformation matrix to a point cloud
def apply_transformation(points, transformation_matrix):
    transformed_points = []
    for point in points:
        # Convert to homogeneous coordinates
        homogeneous_point = np.array([point[0], point[1], 1])
        # Apply transformation
        transformed_homogeneous = transformation_matrix @ homogeneous_point
        # Convert back to 2D coordinates
        transformed_points.append([transformed_homogeneous[0], transformed_homogeneous[1]])
    return np.array(transformed_points)

# Function to plot the point clouds
def plot_point_clouds(source, target, transformed_source=None, title="ICP Alignment"):
    plt.figure(figsize=(8, 6))
    plt.scatter(target[:, 0], target[:, 1], c='blue', label='Target Point Cloud', marker='o')
    plt.scatter(source[:, 0], source[:, 1], c='red', label='Initial Source Point Cloud', marker='x')

    if transformed_source is not None:
        plt.scatter(transformed_source[:, 0], transformed_source[:, 1], c='green', label='Transformed Source Cloud', marker='x')

    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title(title)
    plt.axis('equal')
    plt.grid(True)
    plt.show()

# Define the target point cloud
# target = np.array([
#     [1.0, 1.0],
#     [2.0, 1.0],
#     [3.0, 2.0],
#     [4.0, 3.0]
# ])

target = np.array([
[1.00013,-0.135346],
[0.990301,-0.110976],
[0.983834,-0.0850078],
[0.977504,-0.0625007],
[0.971278,-0.0374527],
[0.974128,-0.0154114],
[0.913988,0.0305934],
[0.904237,0.052334],
[0.895332,0.0723473],
[0.889602,0.093478],
[0.886891,0.114484],
[0.883997,0.135029],
[0.916947,0.182849],
[0.910047,0.204541],
[0.892386,0.223713],
[0.670259,0.298567],
[0.663061,0.314227],
[0.670179,0.336133],
[0.67811,0.357873],
[0.683377,0.38221],
[1.36212,0.86255],
[1.36535,0.908519],
])

# Apply a known transformation to create the source point cloud
theta = np.radians(90)  # Rotation angle
translation = np.array([0.0, 0.0])  # Translation vector

# Known transformation matrix
known_transformation_matrix = np.array([
    [np.cos(theta), -np.sin(theta), translation[0]],
    [np.sin(theta), np.cos(theta), translation[1]],
    [0, 0, 1]
])

# Apply the known transformation to create the source point cloud
# source = apply_transformation(target, known_transformation_matrix)
source = np.array([
[1.00013,-0.135346],
[0.990301,-0.110976],
[0.983834,-0.0850078],
[0.977504,-0.0625007],
[0.971278,-0.0374527],
[0.974128,-0.0154114],
[0.913988,0.0305934],
[0.904237,0.052334],
[0.895332,0.0723473],
[0.889602,0.093478],
[0.886891,0.114484],
[0.883997,0.135029],
[0.916947,0.182849],
[0.910047,0.204541],
[0.892386,0.223713],
[0.670259,0.298567],
[0.663061,0.314227],
[0.670179,0.336133],
[0.67811,0.357873],
[0.683377,0.38221],
[1.36212,0.86255],
[1.36535,0.908519],
[1.36659,0.958549],
[1.36113,1.00424],
[1.35882,1.05207],
[2.21792,2.86572],
[2.14081,2.90455],
[2.07115,2.95402],
[2.00186,3.00471],
[1.92862,3.04691],
[1.85539,3.09351],
[1.7838,3.14252],
[1.56686,3.09358],
[0.903283,2.03777],
[0.843962,2.02924],
[0.77803,2.01179],
[0.71689,1.98794],
[0.656654,1.9731],
[0.602846,1.96223],
[0.54856,1.94876],
[0.494449,1.93301],
[0.443385,1.9269],
[0.391773,1.91203],
[0.342986,1.89877],
[0.296481,1.89115],
[0.250275,1.90009],
[0.211579,1.92642],
[0.168961,1.95546],
[0.123876,1.97913],
[0.078158,2.00197],
[0.0311999,2.03376],
[-0.0162322,2.06469],
[-0.065724,2.09572],
[-0.117447,2.13202],
[-0.168892,2.16241],
[-0.223371,2.19039],
[-0.280009,2.2315],
[-0.338488,2.2699],
[-0.398824,2.29587],
[-0.462252,2.33442],
[-0.530783,2.39156],
[-0.602168,2.43567],
[-0.674128,2.47846],
[-0.246879,0.646463],
[-0.262925,0.594996],
[-0.272881,0.583604],
[-0.291858,0.585824],
[-0.310614,0.593931],
[-0.332702,0.601064],
[-0.354608,0.608169],
[-0.37874,0.614393],
[-0.402629,0.622665],
[-0.429073,0.632133],
[-0.456765,0.639657],
[-0.550374,0.669585],
[-1.11321,0.821484],
[-1.15862,0.813169],
[-1.2013,0.800354],
[-1.24189,0.78792],
[-1.28916,0.777069],
[-1.34237,0.766406],
[-1.39278,0.75289],
[-1.44749,0.739069],
[-1.57798,0.714002],
[-3.08557,0.839577],
[-3.0696,0.75671],
[-3.05085,0.675572],
[-3.02502,0.595084],
[-2.88415,0.42641],
[-2.91268,0.359245],
[-2.92916,0.221151],
[-2.90684,0.150906],
[-2.89762,0.0808623],
[-2.87573,0.0110281],
[-2.85745,-0.055894],
[-2.83958,-0.12313],
[-2.81406,-0.190495],
[-2.80614,-0.255479],
[-2.78993,-0.322401],
[-2.77458,-0.387405],
[-2.75794,-0.450541],
[-2.72989,-0.513413],
[-2.7183,-0.577648],
[-2.70745,-0.642977],
[-1.50969,-0.447076],
[-1.60604,-0.557341],
[-1.64221,-0.657648],
[-1.60862,-0.688462],
[-1.57979,-0.720301],
[-1.56707,-0.757967],
[-0.87041,-0.512334],
[-0.864155,-0.537634],
[-0.790995,-0.548391],
[-0.763871,-0.555925],
[-0.733973,-0.56089],
[-0.703475,-0.567351],
[-0.675479,-0.570249],
[-0.645528,-0.573897],
[-0.625607,-0.582054],
[-0.553331,-0.570136],
[-1.22564,-1.46945],
[-0.402774,-0.588035],
[-0.308338,-0.627319],
[-0.29205,-0.631211],
[-0.276014,-0.634298],
[-0.260831,-0.637991],
[-0.244302,-0.64156],
[-0.226537,-0.644602],
[-0.212223,-0.647614],
[-0.19529,-0.651876],
[-0.179169,-0.651565],
[-0.0965956,-0.644805],
[-0.0456203,-0.998959],
[-0.0225786,-1.00625],
[0.042317,-0.683441],
[0.0593603,-0.678659],
[0.0727814,-0.671568],
[0.0902898,-0.667673],
[0.164946,-0.72446],
[0.180176,-0.712058],
[0.322308,-0.798397],
[0.339528,-0.783333],
[0.607473,-1.13206],
[0.646324,-1.13888],
[0.685718,-1.14281],
[0.722858,-1.14638],
[1.22558,-0.988039],
[1.21215,-0.930727],
[1.18287,-0.817723],
[1.15151,-0.717487],
[1.14379,-0.674283],
[0.807057,-0.214206],
[0.811376,-0.197215],
])
source = apply_transformation(source, known_transformation_matrix)

# Visualize initial alignment
plot_point_clouds(source, target, title="Before ICP Alignment")

# Suppose this is the transformation matrix obtained from ICP (replace this with your ICP output)
icp_transformation_matrix = np.array([
    [0.854518, -0.519422, 82.3747, ],
    [0.519422, 0.854518, 18.0952, ],
    [0, 0, 1, ],
])

# Apply ICP transformation to align source to target
transformed_source = apply_transformation(source, icp_transformation_matrix)

# Visualize the result after applying the ICP transformation
plot_point_clouds(source, target, transformed_source, title="After ICP Alignment")
