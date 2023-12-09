import argparse
import os
import math
import numpy as np
dt = 0.1
length = 0.1
width = 0.2
def differential_drive_model(q, u):
    dq = np.zeros_like(q)
    dq[0] = u[0] * np.cos(q[2]) * dt
    dq[1] = u[0] * np.sin(q[2]) * dt
    dq[2] = u[1] * dt
    return dq





#--------------------------------------------------------------------Create model for Odometery
def Odometery(landmarks_map,measurements):

    print("Odometry Model:")
    initial_location = measurements[0]
    control_sequence = measurements[1:]
    q = initial_location
    trajectory = []
    trajectory.append([q[0], q[1], q[2]])
    print("Odometry initial- ", q)

    # Simulate robot movement
    for i in range(len(control_sequence)):
        dq = differential_drive_model(q, control_sequence[i])
        q += dq
        trajectory.append(landmark_sensor(q[0], q[1], q[2], landmarks_map))
        trajectory.append(control_sequence[i])

        print(f"Odometry Controls-{i}  ", control_sequence[i])
        print(f"Odometry measurements-{i}", landmark_sensor(q[0], q[1], q[2], landmarks_map))

    Odometry = np.array(trajectory, dtype=object)

    return Odometry


#--------------------------------------------------------------------Create model for Actuation

def Actuation(executed_controls):
    print("Actuation Model:")
    initial_location = executed_controls[0]
    control_sequence = executed_controls[1:]
    q = initial_location.copy()
    trajectory = []
    print("Actuation Model initial- ", q)
    trajectory.append([q[0], q[1], q[2]])
    # Simulate robot movement
    for i in range(len(control_sequence)):
        dq = differential_drive_model(q, control_sequence[i])
        q += dq
        trajectory.append([q[0], q[1], q[2]])
        print(f"Actuation Model Trajectory position-{i}  ", q[0], q[1], q[2])



    Poses = np.array(trajectory, dtype=object)
    return Poses





#--------------------------------------------------------------------Sensor measurements calculated here


def landmark_sensor(ground_truth_x, ground_truth_y, ground_truth_theta, landmarks):
    robot_position = np.array([ground_truth_x, ground_truth_y])
    robot_orientation = ground_truth_theta
    t = []
    distances, angles = [], []
    sigma_distance = 0.02
    sigma_direction = 0.02
    for landmark in landmarks:
        delta = landmark - robot_position
        distance = np.linalg.norm(delta)
        angle = np.degrees(np.arctan2(delta[1], delta[0]) - robot_orientation)
        angle = (angle + 360) % 360  # Wrap the angle to the range [0, 360]

        distances.append(distance)
        angles.append(np.radians(angle))
        t.append([distance+ np.random.normal(0, sigma_distance), np.radians(angle)+np.random.normal(0, sigma_direction)])
    return t




#--------------------------------------------------------------------Where i created control sequences for models
# Function to generate odometry measurements
def generate_odometry(controls_file_path, sigma_e_omega, sigma_e_phi):
    control_sequence = controls_file_path

    # Extract initial pose and controls
    initial_pose = control_sequence[0]  # Use [0] directly for the initial pose
    planned_controls = control_sequence[1:]  # Use slicing to get the planned controls



    odometry_measurements = []
    current_pose = np.array(initial_pose)

    for control in planned_controls:
        # Extract control values
        v, phi = control

        # Implement Eq. 5 to generate odometry measurements with noise
        noisy_omega = v + np.random.normal(0, sigma_e_omega)
        noisy_phi = phi + np.random.normal(0, sigma_e_phi)


        # Append the noisy odometry measurement
        odometry_measurements.append([noisy_omega, noisy_phi])
    odometry_measurements.insert(0, control_sequence[0])
    return odometry_measurements

def create_Actuation(controls_file_path):
    # Load control inputs from the file
    control_sequence = np.load(controls_file_path, allow_pickle=True)
    executed_controls=[]
    # Extract initial pose and controls
    initial_pose = control_sequence[0]  # Use [0] directly for the initial pose
    planned_controls = control_sequence[1:]  # Use slicing to get the planned controls
    # Parameters for noise
    sigma_linear = 0.075
    sigma_angular = 0.2
    for i in range(len(planned_controls)):
        # Extract planned linear and angular velocities
        v_planned, omega_planned = planned_controls[i]

        # Generate noise
        noise_linear = np.random.normal(0, sigma_linear)
        noise_angular = np.random.normal(0, sigma_angular)
        v_noisy = v_planned + noise_linear
        omega_noisy = omega_planned + noise_angular
        current_control = np.array([ v_noisy, omega_noisy])
        executed_controls.append(current_control.copy())
    executed_controls.insert(0, initial_pose)
    return executed_controls
#--------------------------------------------------------------------
def simulate(plan_file, map_file, execution_file, sensing_file,V):
    landmarks_map = np.load(map_file)
    Actuation_controls = create_Actuation(plan_file)
    Actuation_Model=Actuation(Actuation_controls)
    np.save(os.path.join(execution_file), Actuation_Model)
    if V=="H":
        sigma_e_omega = 0.1
        sigma_e_phi = 0.3
    else:
        sigma_e_omega= 0.05
        sigma_e_phi= 0.1
    Odometery_controls= generate_odometry(Actuation_controls,sigma_e_omega,sigma_e_phi)
    Odometery_Model = Odometery( landmarks_map,Odometery_controls)
    np.save(os.path.join(sensing_file), Odometery_Model)


if __name__ == "__main__":
    if not os.path.exists("readings"):
        os.makedirs("readings")
    if not os.path.exists("gts"):
        os.makedirs("gts")
    #example : --plan controls/controls_0_1.npy --map maps/landmark_0.npy --sensing readings/readings_X_Y_H.npy



    # Create argument parser
    parser = argparse.ArgumentParser(description="Simulate something.")

    # Add command line arguments
    parser.add_argument("--plan", type=str, help="Path to the plan file")
    parser.add_argument("--map", type=str, help="Path to the map file")
    parser.add_argument("--execution", type=str, help="Path to the execution file")
    parser.add_argument("--sensing", type=str, help="Path to the sensing file")

    # Parse command line arguments
    args = parser.parse_args()
    x = args.plan[18]
    y = args.plan[20]
    execution_file = f"gts/gt_{x}_{y}.npy"
    VARIANCE=args.sensing[22]
    sensing_file=f"readings/readings_{x}_{y}_{VARIANCE}.npy"
    # Call the simulate function with the provided arguments
    simulate(args.plan, args.map, execution_file, sensing_file,VARIANCE)