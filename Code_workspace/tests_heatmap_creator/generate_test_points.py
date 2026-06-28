import argparse
import random
import numpy as np
from PIL import Image


def load_pgm(path):
    img = Image.open(path).convert("L")
    return np.array(img)


def is_free(pixel_value, free_threshold):
    # Typical ROS map:
    # white/free ~= 254
    # black/occupied ~= 0
    # gray/unknown ~= 205
    return pixel_value >= free_threshold


def pixel_to_world(px, py, map_height, origin_x, origin_y, resolution):
    x = origin_x + px * resolution
    y = origin_y + (map_height - py) * resolution
    return x, y


def main():
    parser = argparse.ArgumentParser(description="Generate fake contamination test points from free space in a PGM map.")

    parser.add_argument("--map", required=True, help="Input PGM map")
    parser.add_argument("--output", default="test_points.txt", help="Output TXT file")

    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)

    parser.add_argument("--count", type=int, default=500, help="Number of points to generate")
    parser.add_argument("--free-threshold", type=int, default=240)

    parser.add_argument("--region-x", type=float, required=True, help="Contaminated region center X in meters")
    parser.add_argument("--region-y", type=float, required=True, help="Contaminated region center Y in meters")
    parser.add_argument("--region-radius", type=float, default=1.0, help="Contaminated region radius in meters")

    parser.add_argument("--alpha-bq-max", type=float, default=0.15)
    parser.add_argument("--beta-bq-max", type=float, default=0.10)

    args = parser.parse_args()

    map_array = load_pgm(args.map)
    map_height, map_width = map_array.shape

    free_pixels = np.argwhere(map_array >= args.free_threshold)

    if len(free_pixels) == 0:
        raise RuntimeError("No free-space pixels found. Try lowering --free-threshold.")

    selected = free_pixels[
        np.random.choice(len(free_pixels), size=min(args.count, len(free_pixels)), replace=False)
    ]

    with open(args.output, "w", encoding="utf-8") as f:
        f.write("# x y alpha_cps beta_cps alpha_bq_cm2 beta_bq_cm2\n")

        for py, px in selected:
            x, y = pixel_to_world(
                px,
                py,
                map_height,
                args.origin_x,
                args.origin_y,
                args.resolution,
            )

            dx = x - args.region_x
            dy = y - args.region_y
            distance = (dx * dx + dy * dy) ** 0.5

            if distance <= args.region_radius:
                # Smooth contamination: strongest in the center, weaker near edge
                intensity = 1.0 - (distance / args.region_radius)

                alpha_bq = args.alpha_bq_max * intensity
                beta_bq = args.beta_bq_max * intensity

                # Fake CPS roughly proportional to Bq/cm²
                alpha_cps = alpha_bq * 300.0 + random.uniform(-1.0, 1.0)
                beta_cps = beta_bq * 300.0 + random.uniform(-1.0, 1.0)

                alpha_cps = max(0.0, alpha_cps)
                beta_cps = max(0.0, beta_cps)

            else:
                alpha_bq = random.uniform(0.0, 0.003)
                beta_bq = random.uniform(0.0, 0.003)
                alpha_cps = random.uniform(0.0, 1.0)
                beta_cps = random.uniform(0.0, 1.0)

            f.write(
                f"{x:.3f} {y:.3f} "
                f"{alpha_cps:.3f} {beta_cps:.3f} "
                f"{alpha_bq:.6f} {beta_bq:.6f}\n"
            )

    print(f"Generated {len(selected)} test points in {args.output}")


if __name__ == "__main__":
    main()