import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os
import math
from sklearn.decomposition import PCA
import math

SQUARE_SIZE = 0.1
SAFETY_DISTANCE = 0.1
PATH_TO_MAPS_LIBRARY = "maps/"

ROBOT_SPEED = 0.3
TIME_BY_SQUARE = 30

def pretty_print_hours(total_seconds):
    hours = int(total_seconds // 3600)
    minutes = int((total_seconds % 3600) // 60)
    return f"{hours} hours {minutes} minutes"

def estimate_mission_time(path_coords, robot_speed, time_by_square):
    total_distance = 0.0

    for i in range(len(path_coords) - 1):
        x0, y0 = path_coords[i]
        x1, y1 = path_coords[i + 1]
        dist = math.hypot(x1 - x0, y1 - y0)
        total_distance += dist

    travel_time = total_distance / robot_speed
    pause_time = time_by_square * len(path_coords)

    total_time = travel_time + pause_time
    return total_time, travel_time, pause_time

def link_points_greedy(points, start_idx=0, angle_weight=5.0):
    visited = [False] * len(points)
    path = []
    current_idx = start_idx
    current_heading = None

    for _ in range(len(points)):
        path.append(current_idx)
        visited[current_idx] = True

        current_point = points[current_idx]
        best_score = float('inf')
        next_idx = None

        for idx, point in enumerate(points):
            if visited[idx]:
                continue
            dx = point[0] - current_point[0]
            dy = point[1] - current_point[1]
            distance = math.hypot(dx, dy)
            heading = math.atan2(dy, dx)
            angle_diff = 0.0
            if current_heading is not None:
                angle_diff = abs(heading - current_heading)
                angle_diff = min(angle_diff, 2 * math.pi - angle_diff)
            cost = distance + angle_weight * angle_diff

            if cost < best_score:
                best_score = cost
                next_idx = idx
                best_heading = heading

        if next_idx is None:
            break
        current_idx = next_idx
        current_heading = best_heading

    path.append(start_idx)
    return path

def rotate_image_expand(image, angle_deg):
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))
    M[0, 2] += (new_w / 2) - center[0]
    M[1, 2] += (new_h / 2) - center[1]
    rotated = cv2.warpAffine(image, M, (new_w, new_h), flags=cv2.INTER_NEAREST)
    return rotated, M

def generate_grid_and_path(yaml_path, s2=SQUARE_SIZE, safety_distance=SAFETY_DISTANCE, show=True):
    with open(yaml_path, 'r') as f:
        map_metadata = yaml.safe_load(f)

    resolution = map_metadata['resolution']
    origin = map_metadata['origin']
    pgm_path = os.path.join(os.path.dirname(yaml_path), map_metadata['image'])

    image = Image.open(pgm_path)
    map_gray = np.array(image)
    map_bin = np.where(map_gray < 250, 0, 255).astype(np.uint8)

    inflate_px = int(safety_distance / resolution)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * inflate_px + 1, 2 * inflate_px + 1))
    map_inflated = cv2.dilate(255 - map_bin, kernel)
    free_space = np.where(map_inflated == 0, 1, 0)

    ys, xs = np.where(free_space == 1)
    points = np.column_stack((xs, ys))

    pca = PCA(n_components=2)
    pca.fit(points)
    direction = pca.components_[0]
    angle_rad = np.arctan2(direction[1], direction[0])
    angle_deg = np.degrees(angle_rad)

    rotated_map, M_rot = rotate_image_expand(map_gray, angle_deg)

    map_bin = np.where(rotated_map < 250, 0, 255).astype(np.uint8)
    map_inflated = cv2.dilate(255 - map_bin, kernel)
    free_space = np.where(map_inflated == 0, 1, 0)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(free_space.astype(np.uint8), connectivity=8)
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    free_space = np.where(labels == largest_label, 1, 0).astype(np.uint8)

    grid_points = []
    s2_pix = int(s2 / resolution)
    height, width = free_space.shape

    ys, xs = np.where(free_space == 1)
    min_x, max_x = np.min(xs), np.max(xs)
    min_y, max_y = np.min(ys), np.max(ys)

    free_width = max_x - min_x
    free_height = max_y - min_y

    n_x = free_width // s2_pix
    n_y = free_height // s2_pix

    leftover_x = free_width - n_x * s2_pix
    leftover_y = free_height - n_y * s2_pix

    offset_x = min_x + leftover_x // 2
    offset_y = min_y + leftover_y // 2

    for y in range(offset_y, offset_y + n_y * s2_pix, s2_pix):
        for x in range(offset_x, offset_x + n_x * s2_pix, s2_pix):
            local_area = free_space[y:y + s2_pix, x:x + s2_pix]
            if local_area.shape == (s2_pix, s2_pix) and np.all(local_area == 1):
                center_x_pix = x + s2_pix // 2
                center_y_pix = y + s2_pix // 2
                wx = origin[0] + center_x_pix * resolution
                wy = origin[1] + (height - center_y_pix) * resolution
                grid_points.append((wx, wy))

    path_idx = link_points_greedy(grid_points, start_idx=0, angle_weight=5.0)
    path_coords = [grid_points[i] for i in path_idx]

    total_time, travel_time, pause_time = estimate_mission_time(path_coords, ROBOT_SPEED, TIME_BY_SQUARE)

    print(f"Average robot speed is {ROBOT_SPEED:.2f} m/s and measuring for {TIME_BY_SQUARE}s")
    print(f"Total estimated mission time: {pretty_print_hours(total_time)}.")
    print(f"  - Travel time: {pretty_print_hours(travel_time)}")
    print(f"  - Pause time: {pretty_print_hours(pause_time)}")

    if show:
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(rotated_map, cmap='gray', origin='upper')
        for pt in grid_points:
            px = (pt[0] - origin[0]) / resolution
            py = height - (pt[1] - origin[1]) / resolution
            rect = plt.Rectangle((px - s2_pix/2, py - s2_pix/2), s2_pix, s2_pix, facecolor='blue', alpha=0.3, edgecolor='blue')
            ax.add_patch(rect)
            ax.add_patch(plt.Circle((px, py), radius=0.15, color='blue'))
        for i in range(len(path_coords) - 1):
            x0, y0 = path_coords[i]
            x1, y1 = path_coords[i + 1]
            ax.plot([(x0 - origin[0]) / resolution, (x1 - origin[0]) / resolution],
                    [height - (y0 - origin[1]) / resolution, height - (y1 - origin[1]) / resolution],
                    color='red', linewidth=2)
        ax.set_title(f"Valid grid squares (s2={s2}m, safety={safety_distance}m)")
        plt.axis('off')
        plt.show()


    return grid_points, path_coords

if __name__ == "__main__":
    yaml_file = PATH_TO_MAPS_LIBRARY+'test_est_modified.yaml'  # <<== Change here
    generate_grid_and_path(yaml_file, show=True)