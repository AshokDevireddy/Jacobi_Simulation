import pybullet
import pybullet_data
import numpy as np
import time
import trimesh
import pyrender
import matplotlib.pyplot as plt
from py3dbp import Bin, Item, Packer
import random
import os


from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import numpy as np

def pybullet_to_pyrender_quat(pybullet_quat):

    # Convert to Pyrender's convention with flipped x and y
    return [-pybullet_quat[1], -pybullet_quat[2], pybullet_quat[3], pybullet_quat[0]]




def is_simulation_stable(counter_threshold=100, max_time=25):
    # get all body IDs
    bodies = pybullet.getNumBodies()
    counter = 0

    start_time = time.time()  # Record the start time


    while counter < counter_threshold:
        if time.time() - start_time > max_time:
            print("Max time reached, moving on.")
            return True

        for i in range(bodies):
            # get linear and angular velocity
            lin_vel, ang_vel = pybullet.getBaseVelocity(i)
            # if velocity is above a threshold, reset the counter and break
            if np.linalg.norm(lin_vel) > 0.001 or np.linalg.norm(ang_vel) > 0.001:
                counter = 0
                break
        counter += 1
        # step the simulation and sleep

        pybullet.stepSimulation()
        time.sleep(1./240.)
    # return True if counter has reached threshold, otherwise return False
    return counter == counter_threshold

# Set box dimensions
boxes = {
        "Box1" : ["objects/amazon_box/model.urdf", [0.35, 0.35, 0.35], 0.46, "objects/amazon_box/model.obj"],
        "Box2" : ["objects/cardboard_box/cardboard_box.urdf", [3*i/4 for i in [0.76, 0.39, 0.61]], 76, "objects/cardboard_box/cardboard_box.obj"],
        "Box3" : ["objects/sqaure_box/cardboard_box_closed.urdf", [0.4, 0.36, 0.4], 2, "objects/sqaure_box/cardboard_box_closed.obj"],
        "Box4" : ["objects/amazon-prime-shipping-box/source/amazon_prime_shipping_box.urdf", [0.37, 0.37, 0.37], 0.7, "objects/amazon-prime-shipping-box/source/amazon_prime_shipping_box.obj"],
        "Box5" : ["objects/box5/lowpoly_cardboard_box.urdf", [0.3, 0.3, 0.3], 2, "objects/box5/lowpoly_cardboard_box.obj"],
        "Box6" : ["objects/box6/cardboard-box.urdf", [i/12 for i in [3.87, 3.36, 6.00]], 3.87, "objects/box6/cardboard-box.obj"],
        "Box7" : ["objects/box7/closed_cardboard_box_free_download.urdf", [3*i/4 for i in [0.62, 0.63, 0.83]], 0.62, "objects/box7/closed_cardboard_box_free_download.obj"],
        "Box8" : ["objects/box8/cardboard_box.urdf", [i/100 for i in [40.97, 22.66, 62.88]], 40.97, "objects/box8/cardboard_box.obj"],
        "Box9" : ["objects/box9/vintage_cardboard_box.urdf", [i/10 for i in [3.02, 4.05, 2.00]], 3.02, "objects/box9/vintage_cardboard_box.obj"],
        "Box10" : ["objects/box10/toilet_paper_box.urdf", [3*i/4 for i in [0.48, 0.52, 0.72]], 0.48, "objects/box10/toilet_paper_box.obj"]
        }

def grid(box_list):

# for box1_count in range(3, 10):
#     for box2_count in range(3, 10):

    packer = Packer()
    # Define your pallet (the space in which to pack the boxes)
    pallet_width = 1.3
    pallet_depth = 1.8
    pallet_height = 5
    packer.add_bin(Bin('Pallet', pallet_width, pallet_depth, pallet_height, 100.0))  # 100.0 is the weight limit

    # Define your boxes
    box_quantities = {
                    "Box1": box_list[0],
                    "Box2": box_list[1],
                    "Box3": box_list[2],
                    "Box4": box_list[3],
                    "Box5": box_list[4],
                    "Box6": box_list[5],
                    "Box7": box_list[6],
                    "Box8": box_list[7],
                    "Box9": box_list[8],
                    "Box10": box_list[9],
                    }  # This is an example, replace with your box quantities

    for box_name, box_info in boxes.items():
        for _ in range(box_quantities[box_name]):
            packer.add_item(Item(box_name, box_info[1][0], box_info[1][1], box_info[1][2], box_info[1][0] * box_info[1][1] * box_info[1][2]))  # 1.0 is the weight of the box

    # Run the packing algorithm
    print(packer.pack())

    # Now get the results
    rotation_map = {
        0: [0, 0, 0],  # RT_WHD
        1: [0, 0, np.pi/2],  # RT_HWD
        2: [0, np.pi/2, np.pi/2],  # RT_HDW
        3: [0, -np.pi/2, -np.pi/2],  # RT_DHW
        4: [np.pi/2, 0, 0],  # RT_DWH
        5: [-np.pi/2, 0, 0]  # RT_WDH
    }

    #plot calculated configurations
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']  # Add more colors if needed
    idx = 0

    configurations = []
    for bin in packer.bins:
        for item in bin.items:
            configurations.append([item.name, [float(i) for i in item.position], rotation_map.get(item.rotation_type, [0, 0, 0])])

            # plotting data
            box_name = item.name
            center = item.position
            dimensions = boxes[box_name][1]
            euler_angles = rotation_map[item.rotation_type]

            # Create rotation matrix
            rotation = R.from_euler('xyz', euler_angles)

            # Define the box vertices relative to the center
            dx, dy, dz = np.array(dimensions) / 2.0
            box_vertices = np.array([
                [-dx, -dy, -dz],
                [-dx, -dy, dz],
                [-dx, dy, -dz],
                [-dx, dy, dz],
                [dx, -dy, -dz],
                [dx, -dy, dz],
                [dx, dy, -dz],
                [dx, dy, dz]
            ])
            # Rotate and shift vertices
            rotated_vertices = rotation.apply(box_vertices) + np.array(center).astype(float)
            # Create the sides of the box as polygons
            box_sides = [
                [rotated_vertices[i] for i in indices]
                for indices in [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
            ]
            # Create a Poly3DCollection
            poly = Poly3DCollection(box_sides, alpha=0.1, linewidths=1, edgecolors='r', facecolor=colors[idx%len(colors)])
            ax.add_collection3d(poly)
            idx += 1

    print(configurations)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # Add a bit of padding
    ax.set_xlim([0, pallet_width])
    ax.set_ylim([0, pallet_depth])
    ax.set_zlim([0, pallet_height])
    #plt.show()

    return configurations


# for index in range(1):
def simulate(configurations = [], random = random.randint(5, 12)):
    # Setup pybullet
    physicsClient = pybullet.connect(pybullet.GUI)  # GUI (visualize) , DIRECT
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set gravity
    pybullet.setGravity(0, 0, -9.8)
    # Load ground plane
    planeId = pybullet.loadURDF("plane.urdf")
    pybullet.changeDynamics(planeId, -1, lateralFriction=0.7)


    # Define the sizes for the two types of boundaries (horizontal and vertical)
    boundary_width = 0.01  # Thickness of the boundary
    boundary_height = 2  # Height of the boundary

    # Adjust the dimensions of the enclosure
    x_dim = 1.7  # Length along the x-axis
    y_dim = 2.3  # Length along the y-axis

    # Create collision shapes for horizontal (x-axis) and vertical (y-axis) boundaries
    boundary_shape_x = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[boundary_width/2, y_dim/2, boundary_height/2])
    pybullet.changeDynamics(boundary_shape_x, -1, lateralFriction=0.3)
    boundary_shape_y = pybullet.createCollisionShape(pybullet.GEOM_BOX, halfExtents=[x_dim/2, boundary_width/2, boundary_height/2])
    pybullet.changeDynamics(boundary_shape_y, -1, lateralFriction=0.3)


    # Shift to move the origin to the center
    x_shift = x_dim / 2
    y_shift = y_dim / 2

    # Create the boundaries
    pybullet.createMultiBody(0, boundary_shape_x, basePosition=[-x_shift, 0, boundary_height/2])  # Left
    pybullet.createMultiBody(0, boundary_shape_x, basePosition=[x_shift, 0, boundary_height/2])  # Right
    pybullet.createMultiBody(0, boundary_shape_y, basePosition=[0, -y_shift, boundary_height/2])  # Bottom
    pybullet.createMultiBody(0, boundary_shape_y, basePosition=[0, y_shift, boundary_height/2])  # Top



    # Add boxes to simulation
    box_ids = []

    if configurations:
        for box in configurations:
            box_type = box[0]
            position = box[1]
            # shift the positions to fit in the boundaries
            position = list(map(sum, zip(position, [-x_shift + 0.5, -y_shift + 0.5, 0.3])))
            print(box_type, "position:", position)
            orientation = pybullet.getQuaternionFromEuler(box[2])
            print(box_type, "orientation:", orientation)

            box_id = pybullet.loadURDF(boxes[box_type][0], basePosition=position, baseOrientation=orientation, globalScaling=boxes[box_type][1][0]/boxes[box_type][2])
            box_ids.append((box_id, box_type, boxes[box_type][1][0]/boxes[box_type][2]))
            pybullet.changeDynamics(box_id, -1, mass=10, lateralFriction=0.7, spinningFriction=0.5, restitution=0.0)
            # Wait until simulation is stable
            while not is_simulation_stable():
                pybullet.stepSimulation()
                time.sleep(1./240.)
    else:
        for i in range(random):
            # get random position and orientation
            position = [np.random.uniform(-0.6, 0.6), np.random.uniform(-1.0, 1.0), np.random.uniform(1, 2.0)]
            print("pybullet:", position)
            orientation = pybullet.getQuaternionFromEuler([np.random.uniform(0, np.pi / 6) for _ in range(3)]) # np.pi / 12
            box_type = random.choice(list(boxes.keys()))

            box_id = pybullet.loadURDF(boxes[box_type][0], basePosition=position, baseOrientation=orientation, globalScaling=boxes[box_type][1][0]/boxes[box_type][2])
            box_ids.append((box_id, box_type, boxes[box_type][1][0]/boxes[box_type][2]))
            pybullet.changeDynamics(box_id, -1, mass=10, lateralFriction=0.7, spinningFriction=0.5, restitution=0.0)
            # Wait until simulation is stable
            while not is_simulation_stable():
                pybullet.stepSimulation()
                time.sleep(1./240.)



    # Get box positions and orientations
    box_poses = [(pybullet.getBasePositionAndOrientation(box_id), box_type, scaling) for box_id, box_type, scaling in box_ids]


    # Compute the average position of the boxes
    avg_position = np.mean([pos[0][0] for pos in box_poses], axis=0)
    # avg_position[0] -= 3
    # print(box_poses)
    print("avg position:", avg_position)

    # Disconnect
    pybullet.disconnect()

    # Create a pyrender scene
    scene = pyrender.Scene()

    # Load and add cage to the scene
    cage_scene = trimesh.load("objects/cage/model.obj")
    centroids = []

    cage_scale = 2  # Adjust as needed
    for cage in cage_scene.geometry.values():
        cage.apply_scale(cage_scale)

        # centering the cage manually
        cage_pos = [ 0, 0, 0]
        cage_pos[0] -= 2.77674723
        cage_pos[1] += 0.05147725

        cage.apply_translation(cage_pos)  # Position the cage at the average position of the boxes
        centroids.append(cage.centroid)

        mesh = pyrender.Mesh.from_trimesh(cage, smooth=False)
        scene.add(mesh)

    cage_centroid = np.mean(centroids, axis=0)
    print('cage is centered at:', cage_centroid)



    #for (position, orientation), box_type, scaling in box_poses:
    for (position, orientation), box_type, scaling in box_poses:

        box = trimesh.load(boxes[box_type][3])

        position = list(position)
        position[2] += 0.155 * 2
        print("box position", position)
        #print("box vertices", box.vertices)

        box.apply_scale(scaling)  # Scale the box

        orientation = pybullet_to_pyrender_quat(orientation)

        print("box orientation", orientation)
        box.apply_transform(trimesh.transformations.quaternion_matrix(orientation))

        box.apply_translation(position)


        centroids.append(box.centroid)

        mesh = pyrender.Mesh.from_trimesh(box, smooth=False)
        #box.show()

        scene.add(mesh)




    viewer = pyrender.Viewer(scene, use_raymond_lighting=True)


    # Add a camera
    camera = pyrender.IntrinsicsCamera(fx=500, fy=500, cx=600, cy=500)
    camera_pose = np.array([
    [1.0, 0,   0, 0], #avg_position[0]
    [0.0,  1.0, -0.0, 0], #avg_position[1]
    [0.0,  0,   1,   2.5],  # Lower the camera
    [0.0,  0.0, 0.0, 1.0],
    ])
    scene.add(camera, pose=camera_pose)


    # Add a light
    light = pyrender.SpotLight(color=np.ones(3), intensity=5.0, innerConeAngle=np.pi/8.0, outerConeAngle=np.pi/2.0)
    scene.add(light, pose=camera_pose)

    # Render the scene
    r = pyrender.OffscreenRenderer(1200, 1000)  # Adjust to match ground dimensions
    color, depth = r.render(scene)

    return color, depth


if __name__ == "__main__":

    #box_list = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    #configurations = grid(box_list)

    color, depth = simulate(random = random.randint(5, 12))

    # Save the file
    # for index in range(50):
    # plt.imsave(os.path.join('images/rgb', f'rgb{index}.jpg'), color)
    # plt.imsave(os.path.join('images/depth', f'depth{index}.jpg'), depth, cmap=plt.cm.gray_r)

    # Show the renderings
    plt.figure()
    plt.subplot(1,2,1)
    plt.axis('off')
    plt.imshow(color)
    plt.subplot(1,2,2)
    plt.axis('off')
    plt.imshow(depth, cmap=plt.cm.gray_r)
    plt.show()

