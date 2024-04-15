"""
# -*- coding: utf-8 -*-
-----------------------------------------------------------------------------------
# Author: Nguyen Mau Dung
# DoC: 2020.07.20
# email: nguyenmaudung93.kstn@gmail.com
-----------------------------------------------------------------------------------
# Description: This script for iou calculation of rotated boxes (on GPU)

"""

import sys

import torch
from shapely.geometry import Polygon
from scipy.spatial import ConvexHull

sys.path.append('../')

class Line:
    # ax + by + c = 0
    def __init__(self, p1, p2):
        """

        Args:
            p1: (x, y)
            p2: (x, y)
        """
        self.a = p2[1] - p1[1]
        self.b = p1[0] - p2[0]
        self.c = p2[0] * p1[1] - p2[1] * p1[0]  # cross
        self.device = p1.device

    def cal_values(self, pts):
        return self.a * pts[:, 0] + self.b * pts[:, 1] + self.c

    def find_intersection(self, other):
        # See e.g.     https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Using_homogeneous_coordinates
        if not isinstance(other, Line):
            return NotImplemented
        w = self.a * other.b - self.b * other.a
        return torch.tensor([(self.b * other.c - self.c * other.b) / w, (self.c * other.a - self.a * other.c) / w],
                            device=self.device)
    
def PolyArea2D(pts):
    roll_pts = torch.roll(pts, -1, dims=0)
    area = (pts[:, 0] * roll_pts[:, 1] - pts[:, 1] * roll_pts[:, 0]).sum().abs() * 0.5
    return area

def intersection_area(rect1, rect2):
    """Calculate the inter

    Args:
        rect1: vertices of the rectangles (4, 2)
        rect2: vertices of the rectangles (4, 2)

    Returns:

    """

    # Use the vertices of the first rectangle as, starting vertices of the intersection polygon.
    intersection = rect1

    # Loop over the edges of the second rectangle
    roll_rect2 = torch.roll(rect2, -1, dims=0)
    for p, q in zip(rect2, roll_rect2):
        if len(intersection) <= 2:
            break  # No intersection

        line = Line(p, q)

        # Any point p with line(p) <= 0 is on the "inside" (or on the boundary),
        # any point p with line(p) > 0 is on the "outside".
        # Loop over the edges of the intersection polygon,
        # and determine which part is inside and which is outside.
        new_intersection = []
        line_values = line.cal_values(intersection)
        roll_intersection = torch.roll(intersection, -1, dims=0)
        roll_line_values = torch.roll(line_values, -1, dims=0)
        for s, t, s_value, t_value in zip(intersection, roll_intersection, line_values, roll_line_values):
            if s_value <= 0:
                new_intersection.append(s)
            if s_value * t_value < 0:
                # Points are on opposite sides.
                # Add the intersection of the lines to new_intersection.
                intersection_point = line.find_intersection(Line(s, t))
                new_intersection.append(intersection_point)

        if len(new_intersection) > 0:
            intersection = torch.stack(new_intersection)
        else:
            break

    # Calculate area
    if len(intersection) <= 2:
        return 0.

    return PolyArea2D(intersection)


def cvt_box_2_polygon(box):
    """
    :param array: an array of shape [num_conners, 2]
    :return: a shapely.geometry.Polygon object
    """
    # use .buffer(0) to fix a line polygon
    # more infor: https://stackoverflow.com/questions/13062334/polygon-intersection-error-in-shapely-shapely-geos-topologicalerror-the-opera
    return Polygon([(box[i, 0], box[i, 1]) for i in range(len(box))]).buffer(0)


def get_corners_vectorize(x, y, w, l, yaw):
    """bev image coordinates format - vectorization

    :param x, y, w, l, yaw: [num_boxes,]
    :return: num_boxes x (x,y) of 4 conners
    """
    device = x.device
    bbox2 = torch.zeros((x.size(0), 4, 2), device=device, dtype=torch.float)
    cos_yaw = torch.cos(yaw)
    sin_yaw = torch.sin(yaw)

    # front left
    bbox2[:, 0, 0] = x - w / 2 * cos_yaw - l / 2 * sin_yaw
    bbox2[:, 0, 1] = y - w / 2 * sin_yaw + l / 2 * cos_yaw

    # rear left
    bbox2[:, 1, 0] = x - w / 2 * cos_yaw + l / 2 * sin_yaw
    bbox2[:, 1, 1] = y - w / 2 * sin_yaw - l / 2 * cos_yaw

    # rear right
    bbox2[:, 2, 0] = x + w / 2 * cos_yaw + l / 2 * sin_yaw
    bbox2[:, 2, 1] = y + w / 2 * sin_yaw - l / 2 * cos_yaw

    # front right
    bbox2[:, 3, 0] = x + w / 2 * cos_yaw - l / 2 * sin_yaw
    bbox2[:, 3, 1] = y + w / 2 * sin_yaw + l / 2 * cos_yaw

    return bbox2


def get_polygons_areas_fix_xy(boxes, fix_xy=100.):
    """
    Args:
        box: (num_boxes, 4) --> w, l, im, re
    """
    device = boxes.device
    n_boxes = boxes.size(0)
    x = torch.full(size=(n_boxes,), fill_value=fix_xy, device=device, dtype=torch.float)
    y = torch.full(size=(n_boxes,), fill_value=fix_xy, device=device, dtype=torch.float)
    w, l, im, re = boxes.t()
    yaw = torch.atan2(im, re)
    boxes_conners = get_corners_vectorize(x, y, w, l, yaw)
    boxes_polygons = [cvt_box_2_polygon(box_) for box_ in boxes_conners]
    boxes_areas = w * l

    return boxes_polygons, boxes_areas


def iou_rotated_boxes_targets_vs_anchors(anchors_polygons, anchors_areas, targets_polygons, targets_areas):
    device = anchors_areas.device
    num_anchors = len(anchors_areas)
    num_targets_boxes = len(targets_areas)

    ious = torch.zeros(size=(num_anchors, num_targets_boxes), device=device, dtype=torch.float)

    for a_idx in range(num_anchors):
        for tg_idx in range(num_targets_boxes):
            intersection = anchors_polygons[a_idx].intersection(targets_polygons[tg_idx]).area
            iou = intersection / (anchors_areas[a_idx] + targets_areas[tg_idx] - intersection + 1e-16)
            ious[a_idx, tg_idx] = iou

    return ious


def iou_pred_vs_target_boxes(pred_boxes, target_boxes, GIoU=False, DIoU=False, CIoU=False):
    assert pred_boxes.size() == target_boxes.size(), "Unmatch size of pred_boxes and target_boxes"
    device = pred_boxes.device
    n_boxes = pred_boxes.size(0)

    t_x, t_y, t_w, t_l, t_im, t_re = target_boxes.t()
    t_yaw = torch.atan2(t_im, t_re)
    t_conners = get_corners_vectorize(t_x, t_y, t_w, t_l, t_yaw)
    t_areas = t_w * t_l

    p_x, p_y, p_w, p_l, p_im, p_re = pred_boxes.t()
    p_yaw = torch.atan2(p_im, p_re)
    p_conners = get_corners_vectorize(p_x, p_y, p_w, p_l, p_yaw)
    p_areas = p_w * p_l

    ious = []
    giou_loss = torch.tensor([0.], device=device, dtype=torch.float)
    # Thinking to apply vectorization this step
    for box_idx in range(n_boxes):
        p_cons, t_cons = p_conners[box_idx], t_conners[box_idx]
        if not GIoU:
            p_poly, t_poly = cvt_box_2_polygon(p_cons), cvt_box_2_polygon(t_cons)
            intersection = p_poly.intersection(t_poly).area
        else:
            intersection = intersection_area(p_cons, t_cons)

        p_area, t_area = p_areas[box_idx], t_areas[box_idx]
        union = p_area + t_area - intersection
        iou = intersection / (union + 1e-16)

        if GIoU:
            convex_conners = torch.cat((p_cons, t_cons), dim=0)
            hull = ConvexHull(convex_conners.clone().detach().cpu().numpy())  # done on cpu, just need indices output
            convex_conners = convex_conners[hull.vertices]
            convex_area = PolyArea2D(convex_conners)
            giou_loss += 1. - (iou - (convex_area - union) / (convex_area + 1e-16))
        else:
            giou_loss += 1. - iou

        if DIoU or CIoU:
            raise NotImplementedError

        ious.append(iou)

    return torch.tensor(ious, device=device, dtype=torch.float), giou_loss