import numpy as np
#"readings/readings_0_1_H.npy"
#"gts/gt_0_1.npy"
#"controls/controls_0_1.npy"
# Load the control sequence
control_sequence = np.load("readings/readings_0_1_H.npy", allow_pickle=True)

# Print the control sequence
print("Control Sequence:")
for i, control in enumerate(control_sequence):
    print(f"Step {i + 1}: {control}")