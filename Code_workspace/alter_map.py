import yaml
import cv2
import numpy as np
from PIL import Image
import os

# =========================
# CONFIG
# =========================

MAP_YAML = "maps/test_est_modified.yaml"
OUTPUT_PREFIX = "altered_map"

brush_size = 6


# =========================
# LOAD MAP
# =========================

with open(MAP_YAML, "r") as f:
    meta = yaml.safe_load(f)

resolution = meta["resolution"]
origin = meta["origin"]

pgm_path = os.path.join(os.path.dirname(MAP_YAML), meta["image"])

img = Image.open(pgm_path)
map_gray = np.array(img)

draw_map = map_gray.copy()

drawing = False


# =========================
# MOUSE DRAWING
# =========================

def draw_wall(event, x, y, flags, param):
    global drawing, draw_map

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            cv2.circle(draw_map, (x, y), brush_size, 0, -1)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False


cv2.namedWindow("alter_map")
cv2.setMouseCallback("alter_map", draw_wall)


# =========================
# UI LOOP
# =========================

while True:

    display = cv2.cvtColor(draw_map, cv2.COLOR_GRAY2BGR)

    cv2.putText(
        display,
        f"Brush: {brush_size}px",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2,
    )

    cv2.imshow("alter_map", display)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

    if key == ord("r"):
        draw_map = map_gray.copy()

    if key == ord("["):
        brush_size = max(1, brush_size - 1)

    if key == ord("]"):
        brush_size += 1

    if key == ord("s"):

        out_pgm = OUTPUT_PREFIX + ".pgm"
        out_yaml = OUTPUT_PREFIX + ".yaml"

        Image.fromarray(draw_map).save(out_pgm)

        new_meta = meta.copy()
        new_meta["image"] = out_pgm

        with open(out_yaml, "w") as f:
            yaml.dump(new_meta, f)

        print("Saved:", out_pgm, out_yaml)


cv2.destroyAllWindows()