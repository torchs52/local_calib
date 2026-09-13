import cv2
import numpy as np


def plot_withoffset(val, margin, vmin, vmax, length, offset, coeff):
    return offset + coeff * (
        margin + (val - vmin) / (vmax - vmin) * (length - 2 * margin)
    )


def draw_scatter_plot(
    points,
    resolution=(800, 600),
    point_color=(0, 0, 255),
    point_radius=3,
    grid_color=(200, 200, 200),
    axis_color=(0, 0, 0),
    num_ticks=10,
    swap_xy=False,
    boxlist=None,
):
    width, height = resolution
    margin = 60  # Margin for axes and labels
    canvas = np.ones((height, width, 3), dtype=np.uint8) * 255

    if len(points) == 0:
        return None

    # Extract x and y values
    if swap_xy is False:
        x_vals = np.array([p[0] for p in points])
        y_vals = np.array([p[1] for p in points])
    else:
        x_vals = np.array([p[1] for p in points])
        y_vals = np.array([p[0] for p in points])

    box_x = [x_vals[0]]
    box_y = [y_vals[0]]
    if boxlist is not None:
        for ixt, Lt in enumerate(boxlist):  # ボックス種類ごと
            for ixb, Lb in enumerate(Lt):  # ボックスごと
                box_coordinates_orig = Lb
                coordinate_ord = [1, 0, 3, 2] if swap_xy else [0, 1, 2, 3]
                for ix, (cod, v) in enumerate(
                    zip(coordinate_ord, box_coordinates_orig, strict=False)
                ):
                    if cod % 2 == 0:  # 0,2:X
                        box_x.append(v)
                    else:
                        box_y.append(v)
    box_x = np.array(box_x)
    box_y = np.array(box_y)

    # Determine axis limits with padding
    x_min, x_max = min(x_vals.min(), box_x.min()), max(x_vals.max(), box_x.max())
    y_min, y_max = min(y_vals.min(), box_y.min()), max(y_vals.max(), box_y.max())
    x_range = x_max - x_min if x_max != x_min else 1
    y_range = y_max - y_min if y_max != y_min else 1
    x_min -= 0.05 * x_range
    x_max += 0.05 * x_range
    y_min -= 0.05 * y_range
    y_max += 0.05 * y_range

    # Draw grid and ticks
    for i in range(num_ticks + 1):
        # X axis ticks and grid
        x_val = x_min + i * (x_max - x_min) / num_ticks
        # x_pos = int(margin + (x_val - x_min) / (x_max - x_min) * (width - 2 * margin))
        x_pos = int(plot_withoffset(x_val, margin, x_min, x_max, width, 0, 1))
        cv2.line(canvas, (x_pos, margin), (x_pos, height - margin), grid_color, 1)
        cv2.putText(
            canvas,
            f"{x_val:.2f}",
            (x_pos - 20, height - margin + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            axis_color,
            1,
        )

        # Y axis ticks and grid
        y_val = y_min + i * (y_max - y_min) / num_ticks
        # y_pos = int(height - margin - (y_val - y_min) / (y_max - y_min) * (height - 2 * margin))
        y_pos = int(plot_withoffset(y_val, margin, y_min, y_max, height, height, -1))
        cv2.line(canvas, (margin, y_pos), (width - margin, y_pos), grid_color, 1)
        cv2.putText(
            canvas,
            f"{y_val:.2f}",
            (5, y_pos + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            axis_color,
            1,
        )

    # Draw axes
    cv2.line(
        canvas, (margin, margin), (margin, height - margin), axis_color, 2
    )  # Y axis
    cv2.line(
        canvas,
        (margin, height - margin),
        (width - margin, height - margin),
        axis_color,
        2,
    )  # X axis

    # Plot points
    for x, y in zip(x_vals, y_vals, strict=False):
        # x_pos = int(margin + (x - x_min) / (x_max - x_min) * (width - 2 * margin))
        x_pos = int(plot_withoffset(x, margin, x_min, x_max, width, 0, 1))
        # y_pos = int(height - margin - (y - y_min) / (y_max - y_min) * (height - 2 * margin))
        y_pos = int(plot_withoffset(y, margin, y_min, y_max, height, height, -1))
        cv2.circle(canvas, (x_pos, y_pos), point_radius, point_color, -1)

    boxcolor = [(100, 100, 0), (0, 100, 100)]
    if boxlist is not None:
        for ixt, Lt in enumerate(boxlist):  # ボックス種類ごと
            color = boxcolor[ixt]
            for ixb, Lb in enumerate(Lt):
                box_coordinates_orig = Lb
                box_coordinates_draw = [0, 0, 0, 0]
                coordinate_ord = [1, 0, 3, 2] if swap_xy else [0, 1, 2, 3]

                for ix, (cod, v) in enumerate(
                    zip(coordinate_ord, box_coordinates_orig, strict=False)
                ):
                    if cod % 2 == 0:  # 0,2:X
                        box_coordinates_draw[cod] = int(
                            plot_withoffset(v, margin, x_min, x_max, width, 0, 1)
                        )
                    else:
                        box_coordinates_draw[cod] = int(
                            plot_withoffset(v, margin, y_min, y_max, height, height, -1)
                        )

                canvas_new = np.zeros_like(canvas)
                cv2.rectangle(
                    canvas_new,
                    pt1=tuple(box_coordinates_draw[0:2]),
                    pt2=tuple(box_coordinates_draw[2:4]),
                    color=color,
                    thickness=1,
                    lineType=cv2.LINE_4,
                    shift=0,
                )

                canvas += canvas_new

    return canvas
