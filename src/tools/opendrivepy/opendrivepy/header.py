from dataclasses import dataclass


@dataclass
class Header:
    north_bound: float
    south_bound: float
    east_bound: float
    west_bound: float
    lat_0: float
    lon_0: float
    x_0: float
    y_0: float
    grid_width: float = None
    grid_height: float = None
    grid_resolution: float = None
