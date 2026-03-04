import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os

# ======================
# PARAMETERS
# ======================

SQUARE_SIZE = 0.5       # meters
SAFETY_DISTANCE = 0.3   # meters

MAP_YAML = "maps/test_est_modified.yaml"


# ======================
# LOAD MAP
# ======================

def load_map(yaml_path):

    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    resolution = meta["resolution"]
    origin = meta["origin"]

    pgm_path = os.path.join(os.path.dirname(yaml_path), meta["image"])

    img = Image.open(pgm_path)
    map_gray = np.array(img)

    return map_gray, resolution, origin


# ======================
# BUILD FREE SPACE
# ======================

def compute_free_space(map_gray, resolution):

    # ROS map interpretation
    free = map_gray >= 250

    # inflate obstacles for safety
    inflate_px = int(SAFETY_DISTANCE / resolution)

    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (2 * inflate_px + 1, 2 * inflate_px + 1)
    )

    obstacles = (~free).astype(np.uint8) * 255
    inflated = cv2.dilate(obstacles, kernel)

    free_safe = inflated == 0

    return free_safe.astype(np.uint8)


# ======================
# KEEP MAIN REGION
# ======================

def keep_largest_region(free_space):

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        free_space,
        connectivity=8
    )

    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])

    return (labels == largest).astype(np.uint8)


# ======================
# GRID GENERATION
# ======================

def generate_grid(free_space, resolution, origin):

    square_px = int(SQUARE_SIZE / resolution)

    h, w = free_space.shape

    points = []

    for y in range(0, h, square_px):
        for x in range(0, w, square_px):

            area = free_space[y:y + square_px, x:x + square_px]

            if area.shape == (square_px, square_px) and np.all(area):

                cx = x + square_px // 2
                cy = y + square_px // 2

                wx = origin[0] + cx * resolution
                wy = origin[1] + (h - cy) * resolution

                points.append((wx, wy))

    return points


# ======================
# DEBUG VISUALIZATION
# ======================

def show_debug(map_gray, free_space, points, resolution, origin):

    h, w = map_gray.shape

    plt.figure(figsize=(10,6))
    plt.imshow(map_gray, cmap="gray")

    xs = [(p[0] - origin[0]) / resolution for p in points]
    ys = [h - (p[1] - origin[1]) / resolution for p in points]

    plt.scatter(xs, ys, c="red", s=10)

    plt.title("Mesh debug")
    plt.axis("off")
    plt.show()


# ======================
# MAIN
# ======================

def main():

    map_gray, resolution, origin = load_map(MAP_YAML)

    free_space = compute_free_space(map_gray, resolution)

    free_space = keep_largest_region(free_space)

    points = generate_grid(free_space, resolution, origin)

    print("Generated", len(points), "points")

    with open("mesh_points.txt", "w") as f:
        for p in points:
            f.write(f"{p[0]:.3f} {p[1]:.3f}\n")

    show_debug(map_gray, free_space, points, resolution, origin)


if __name__ == "__main__":
    main()