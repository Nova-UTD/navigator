"""
Package: pedestrian_intent_to_enter_road
   File: pedestrian_geometry.py

Pure geometry helpers for the pedestrian-intent node. No ROS, no ML
dependencies, deterministic — unit-testable in isolation.
"""


# Monocular pinhole constants — tied to the ~672x376 capture resolution.
# These mirror the constants already used inside the producer's
# calculate_horizontal_distance() (focal 470, tan(HFOV/2) = 1.0913).
_FOCAL_PX = 470.0           # focal length (px)
_PERSON_HEIGHT_M = 1.75     # assumed real pedestrian height (m)
_TAN_HALF_HFOV = 1.0913     # tan(HFOV / 2)


def estimate_depth(bbox_pixel_height):
    """Forward range (m) from a monocular bbox pixel height.

    depth = focal * real_height / pixel_height. Returns 0.0 for a non-positive
    pixel height (degenerate bbox) so callers can skip it.

    @param bbox_pixel_height  Bounding-box height in pixels (BRy - TLy).
    @return                   Forward range in meters (float).
    """
    if bbox_pixel_height <= 0:
        return 0.0
    return _FOCAL_PX * _PERSON_HEIGHT_M / float(bbox_pixel_height)


def project_to_base_link(center_x, bbox_pixel_height, img_w,
                         cam_offset_x=0.0, cam_offset_y=0.0):
    """Project a pedestrian bbox to metric base_link coordinates.

    Sign convention: a pedestrian right of image center (center_x > img_w/2)
    maps to NEGATIVE pos_y (the vehicle's right). cam_offset_x/y add a static
    camera->base_link offset (default 0.0 = camera at base_link origin).

    @param center_x          Bbox center x in pixels.
    @param bbox_pixel_height  Bbox height in pixels (BRy - TLy).
    @param img_w             Image width in pixels.
    @param cam_offset_x      Forward camera offset (m).
    @param cam_offset_y      Lateral camera offset (m).
    @return                  Tuple (pos_x, pos_y) in meters, base_link.
    """
    depth = estimate_depth(bbox_pixel_height)
    cx = img_w / 2.0
    lateral = -((center_x - cx) / img_w) * (2.0 * depth * _TAN_HALF_HFOV)
    return depth + cam_offset_x, lateral + cam_offset_y
