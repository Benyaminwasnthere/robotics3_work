import argparse
import numpy as np
import argparse
import numpy as np
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D

dt = 0.1
length = 0.1
width = 0.2
#------------------------------------------
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
    controls=[]
    sensor=[]
    data = np.load(sensing, allow_pickle=True)
    for i in range(len(data)):
        if(i % 2 == 0):
          controls.append(data[i])
        else:
            sensor.append(data[i])



    return controls,sensor
#-----------------------------------------------
if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Particle Filter for Robot Pose Estimation")
    parser.add_argument("--map", type=str, help="Path to the map file")
    parser.add_argument("--sensing", type=str, help="Path to the readings file")
    parser.add_argument("--num_particles", type=int, help="Number of particles")
    parser.add_argument("--estimates", type=str, help="Path to save pose estimates")

    args = parser.parse_args()
    controls, measurements = seperate_data(args.sensing)
    Trajectory=convert_structure_controls(controls)

