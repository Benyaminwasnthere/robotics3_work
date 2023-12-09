import argparse
import numpy as np
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D

dt = 0.1
length = 0.1
width = 0.2
def convert_structure_controls(controls):

    initial_location = controls[0]
    control_sequence = controls[1:]
    q = initial_location
    trajectory = []
    trajectory.append([q[0], q[1], q[2]])


    # Simulate robot movement
    for i in range(len(control_sequence)):
        dq = differential_drive_model(q, control_sequence[i])
        q += dq
        trajectory.append([q[0], q[1], q[2]])

    Odometry = np.array(trajectory, dtype=object)

    return Odometry


#------------------------------------------
def global_landmark_positions(robot_x, robot_y, robot_theta, landmarks_local):
    landmarks_global = []

    for t in landmarks_local:
        distance, angle_rad = t  # Angle is already in radians
        landmark_x = robot_x + distance * np.cos(robot_theta + angle_rad)
        landmark_y = robot_y + distance * np.sin(robot_theta + angle_rad)
        landmarks_global.append([landmark_x, landmark_y])

    return landmarks_global



#------------------------------------------

def differential_drive_model(q, u):
    dq = np.zeros_like(q)
    dq[0] = u[0] * np.cos(q[2]) * dt
    dq[1] = u[0] * np.sin(q[2]) * dt
    dq[2] = u[1] * dt
    return dq

def draw_rotated_rectangle(ax, center, width, height, angle_degrees, color='r'):
    x, y = center
    rect = patches.Rectangle((x - width / 2, y - height / 2), width, height, linewidth=1, edgecolor=color,
                             facecolor='none')
    t = Affine2D().rotate_deg_around(x, y, angle_degrees) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)




def seperate_data(sensing):
    trajectory=[]
    sensor=[]
    data = np.load(sensing, allow_pickle=True)
    for i in range(len(data)):
        if(i % 2 == 0):
          trajectory.append(data[i])
        else:
            sensor.append(data[i])



    return trajectory,sensor
#-----------------------------------------------
def animate_robot_trajectory(trajectory,trajectory_a,s,landmarks_map, length, width):
    fig, ax = plt.subplots(figsize=(6, 6))
    plt.xlim(0, 2)
    plt.ylim(0, 2)

    for i in range(len(trajectory)):
        q = trajectory[i]
        a=trajectory_a[i]
        if(i>1):
            b=global_landmark_positions(trajectory[i][0], trajectory[i][1], trajectory[i][2], s[i-1])
        plt.clf()
        ax = plt.gca()
        plt.xlim(0, 2)
        plt.ylim(0, 2)
        if (i > 1):
            plt.scatter(np.array(b)[:, 0], np.array(b)[:, 1], color='red', marker='x', label='Landmarks')
            # Draw line segments between each point in b and the robot's current position q
            for point in b:
                plt.plot([q[0], point[0]], [q[1], point[1]], color='green', linestyle=':', linewidth=0.75, alpha=0.5)



        # Draw path
        path_array = np.array(trajectory[:i+1])
        plt.plot(path_array[:, 0], path_array[:, 1], color='red', linestyle='--', linewidth=2)

        # Draw path
        path_array_a = np.array(trajectory_a[:i + 1])
        plt.plot(path_array_a[:, 0], path_array_a[:, 1], color='blue', linestyle='--', linewidth=2)

        # Draw robot body
        draw_rotated_rectangle(ax, [q[0], q[1]], length, width, np.degrees(q[2]))
        plt.scatter(landmarks_map[:, 0], landmarks_map[:, 1], marker='o', label='Landmarks Map 1')
        plt.pause(0.05)

    plt.show()



if __name__ == "__main__":
    #--map maps/landmark_0.npy --sensing readings/readings_0_1_H.npy --execution gts/gt_0_1.npy
    # Define command-line arguments
    parser = argparse.ArgumentParser(description="Dead Reckoning Script")
    parser.add_argument("--map", required=True, help="Path to the map file")
    parser.add_argument("--execution", required=True, help="Path to the execution file")
    parser.add_argument("--sensing", required=True, help="Path to the sensing file")

    # Parse the command-line arguments
    args = parser.parse_args()
    landmarks_map = np.load(args.map)
    controls,s_Odometry=seperate_data(args.sensing)
    data = np.load(args.execution, allow_pickle=True)
    control=convert_structure_controls(controls)
    animate_robot_trajectory(control,data,s_Odometry, landmarks_map, length, width)

    print("hi")