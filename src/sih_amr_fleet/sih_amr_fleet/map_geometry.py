"""Pure helpers for deriving the fleet planning grid from its map YAML."""

import math


def map_geometry_from_data(
        data, *, default_resolution=0.5, default_width=90,
        default_height=120, default_origin=(-22.5, -30.0)):
    """Return ``(resolution, width, height, origin_x, origin_y, blocked)``.

    The map publisher, route planner, and LiDAR blockage detector must use the
    exact same static occupancy expansion.  Keeping that calculation here
    prevents a shelf from being a static obstacle in one node and a dynamic
    blockage in another.
    """
    resolution = float(data.get('resolution_m', default_resolution))
    width = int(data.get('width', default_width))
    height = int(data.get('height', default_height))
    origin = data.get('origin', list(default_origin))
    origin_x, origin_y = float(origin[0]), float(origin[1])
    blocked = {tuple(cell) for cell in data.get('blocked_cells', [])}

    layout = data.get('shelf_layout')
    if layout:
        footprint = layout.get('footprint_m', [3.92, 0.90])
        half_x = math.ceil(float(footprint[0]) / resolution / 2.0)
        half_y = math.ceil(float(footprint[1]) / resolution / 2.0)
        excluded = {tuple(pair) for pair in layout.get('excluded_zones', [])}

        for y_zone, rows in layout.get('y_zones', {}).items():
            for x_zone, columns in layout.get('x_zones', {}).items():
                if (y_zone, x_zone) in excluded:
                    continue
                for x in columns:
                    for y in rows:
                        centre_x = round((float(x) - origin_x) / resolution)
                        centre_y = round((float(y) - origin_y) / resolution)
                        for dx in range(-half_x, half_x + 1):
                            for dy in range(-half_y, half_y + 1):
                                cell = (centre_x + dx, centre_y + dy)
                                if 0 <= cell[0] < width and 0 <= cell[1] < height:
                                    blocked.add(cell)

    # Outer perimeter west wall: the physical concrete wall inner surface is at
    # origin_x (x = -22.5m, cell cx = 0). Blocking column cx = 0 prevents AMRs
    # from treating the impassable 0.55m gap between the shelves and the outer wall
    # as a traversable shortcut, ensuring they always use the main corridors.
    for cy in range(height):
        blocked.add((0, cy))

    return resolution, width, height, origin_x, origin_y, blocked
