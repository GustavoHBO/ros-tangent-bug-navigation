import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml

# Settings
map_size_meters = 10  # 10x10 meters map
resolution = 0.05     # 5cm per pixel
pixels = int(map_size_meters / resolution)

# Create a white empty map
map_array = np.full((pixels, pixels), 255, dtype=np.uint8)

# Parse the SDF
tree = ET.parse('world_test.sdf')  # <<< Change your SDF file name here
root = tree.getroot()

# Find models
for model in root.iter('model'):
    pose = model.find('pose')
    if pose is None:
        continue

    pose_values = list(map(float, pose.text.split()))
    x, y, z = pose_values[0], pose_values[1], pose_values[2]

    collision = model.find('link/collision/geometry/box/size')
    if collision is None:
        continue

    size_values = list(map(float, collision.text.split()))
    size_x, size_y, size_z = size_values[0], size_values[1], size_values[2]

    # Convert world coordinates to pixels
    center_px = int((x + map_size_meters / 2) / resolution)
    center_py = int((y + map_size_meters / 2) / resolution)
    half_size_px = int((size_x / 2) / resolution)
    half_size_py = int((size_y / 2) / resolution)

    # Draw a filled rectangle (black = obstacle)
    x_min = max(center_px - half_size_px, 0)
    x_max = min(center_px + half_size_px, pixels)
    y_min = max(center_py - half_size_py, 0)
    y_max = min(center_py + half_size_py, pixels)

    map_array[y_min:y_max, x_min:x_max] = 0  # Black = obstacle

# Save PGM image
img = Image.fromarray(map_array)
img.save('generated_map.pgm')

# Save YAML file
map_metadata = {
    'image': 'generated_map.pgm',
    'resolution': resolution,
    'origin': [-map_size_meters/2, -map_size_meters/2, 0.0],  # bottom-left
    'negate': 0,
    'occupied_thresh': 0.65,
    'free_thresh': 0.196,
}

with open('generated_map.yaml', 'w') as yaml_file:
    yaml.dump(map_metadata, yaml_file, default_flow_style=False)

print("✅ Map and YAML generated!")
