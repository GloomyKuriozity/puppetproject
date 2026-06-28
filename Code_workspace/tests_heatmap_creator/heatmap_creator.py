import argparse
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from scipy.ndimage import gaussian_filter


COLUMNS = {
    "alpha_cps": 2,
    "beta_cps": 3,
    "alpha_bq": 4,
    "beta_bq": 5,
}


def load_measurements(path):
    data = []

    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.replace(",", ".").split()

            if len(parts) < 6:
                continue

            data.append([float(v) for v in parts[:6]])

    return np.array(data, dtype=float)


def world_to_pixel(x, y, origin_x, origin_y, resolution, map_height):
    px = int((x - origin_x) / resolution)

    # PGM image origin is top-left, map/world origin is usually bottom-left
    py = map_height - int((y - origin_y) / resolution)

    return px, py


def build_heatmap(
    measurements,
    map_width,
    map_height,
    value_column,
    origin_x,
    origin_y,
    resolution,
    blur_sigma_px,
):
    heat = np.zeros((map_height, map_width), dtype=float)
    weight = np.zeros((map_height, map_width), dtype=float)

    for row in measurements:
        x = row[0]
        y = row[1]
        value = row[value_column]

        px, py = world_to_pixel(
            x,
            y,
            origin_x,
            origin_y,
            resolution,
            map_height,
        )

        if 0 <= px < map_width and 0 <= py < map_height:
            heat[py, px] += value
            weight[py, px] += 1.0

    heat = gaussian_filter(heat, sigma=blur_sigma_px)
    weight = gaussian_filter(weight, sigma=blur_sigma_px)

    valid = weight > 0.0001
    heat[valid] /= weight[valid]
    heat[~valid] = np.nan

    return heat


def main():
    parser = argparse.ArgumentParser(description="Generate contamination heatmap from PGM map and measurement TXT.")

    parser.add_argument("--map", required=True, help="Input PGM map")
    parser.add_argument("--data", required=True, help="Input TXT measurement file")
    parser.add_argument("--mode", choices=COLUMNS.keys(), default="alpha_bq")

    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in m/pixel")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)

    parser.add_argument("--min", type=float, default=0.0, help="Minimum contamination scale")
    parser.add_argument("--max", type=float, required=True, help="Maximum contamination scale")

    parser.add_argument("--blur", type=float, default=25.0, help="Cloud blur radius in pixels")
    parser.add_argument("--alpha", type=float, default=0.65, help="Heatmap opacity")
    parser.add_argument("--output", default="heatmap.png")

    args = parser.parse_args()

    map_img = Image.open(args.map).convert("L")
    map_array = np.array(map_img)

    map_height, map_width = map_array.shape

    free_mask = map_array >= 240

    measurements = load_measurements(args.data)

    heat = build_heatmap(
        measurements=measurements,
        map_width=map_width,
        map_height=map_height,
        value_column=COLUMNS[args.mode],
        origin_x=args.origin_x,
        origin_y=args.origin_y,
        resolution=args.resolution,
        blur_sigma_px=args.blur,
    )

    heat[~free_mask] = np.nan

    fig, ax = plt.subplots(figsize=(12, 10))

    ax.imshow(map_array, cmap="gray", origin="upper")

    heat_img = ax.imshow(
        heat,
        cmap="RdYlGn_r",
        origin="upper",
        alpha=args.alpha,
        vmin=args.min,
        vmax=args.max,
    )

    cbar = plt.colorbar(
    heat_img,
    ax=ax,
    orientation="horizontal",
    fraction=0.04,   # thickness
    pad=0.08         # distance from map
)

    cbar.set_label(args.mode)
    cbar.ax.tick_params(labelsize=8)

    ax.set_title(f"Contamination heatmap - {args.mode}")
    ax.set_axis_off()

    plt.tight_layout()
    plt.savefig(args.output, dpi=200)
    plt.show()


if __name__ == "__main__":
    main()