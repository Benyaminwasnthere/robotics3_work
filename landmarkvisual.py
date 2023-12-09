import numpy as np
import matplotlib.pyplot as plt

def visualize_map(landmarks, title):
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='o')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(title)
    plt.show()

def load_landmark_map(map_name):
    landmarks = np.load(f"maps/landmark_{map_name}.npy")
    return landmarks

# Load a single landmark map and visualize it
if __name__ == '__main__':
    map_name = 0  # Change this to the desired map number
    landmark_map = load_landmark_map(map_name)
    visualize_map(landmark_map, f'Landmark Map with {len(landmark_map)} Landmarks (MAP{map_name})')
