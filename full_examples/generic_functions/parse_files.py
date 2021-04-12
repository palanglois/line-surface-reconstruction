import json
import csv
import math
import numpy as np
from numpy.linalg import inv, det


class Plane:
    """
    A class to handle the plane type
    """

    def __init__(self, normal, inlier):
        self.normal = normal
        self.inlier = inlier

    def dist_to_point(self, point):
        """
        Compute the distance from the current plane to a point
        :param point: a 3d numpy array
        :return: the unsigned distance between the plane and the point
        """
        return np.abs(np.dot(point - self.inlier, self.normal))


class Line:
    """
    A class to handle the line type
    """

    def __init__(self, plane_index, pt1, pt2, pt_views, residuals_per_pt_view=[]):
        self.plane_index = plane_index
        self.pt1 = pt1
        self.pt2 = pt2
        self.pt_views = pt_views
        self.residuals_per_pt_view = residuals_per_pt_view


class Image:
    """
    A class to handle the image type (colmap context)
    """

    def __init__(self, id, quaternion, translation, cam_id, name):
        self.id = id
        self.quaternion = quaternion
        self.translation = translation
        self.cam_id = cam_id
        self.name = name


class Camera:
    """
    A class to handle the camera type (colmap context)
    """

    def __init__(self, cam_id, fx, fy, cx, cy):
        self.cam_id = cam_id
        self.cx = cx
        self.cy = cy
        self.fx = fx
        self.fy = fy


def parse_colmap_images(path):
    """
    Parsing the images.txt colmap file.
    :param path: path to the images.txt file from colmap
    :return: list of parsed Images
    """

    out_images = []
    with open(path, 'r') as images_file:
        lines_to_keep = [l.split() for l in images_file if l[0] != '#'][::2]
        for line in lines_to_keep:
            out_images.append(Image(int(line[0]), [float(x) for x in line[1:5]], [float(x) for x in line[5:8]],
                                    int(line[8]), line[9]))
    return sorted(out_images, key=lambda x: x.cam_id)


def parse_colmap_cameras(path):
    """
    Parsing the cameras.txt colmap file
    :param path: path to the cameras.txt file from colmap
    :return: line of parsed Cameras
    """

    out_cameras = []
    with open(path, 'r') as cameras_file:
        lines_to_keep = [l.split() for l in cameras_file if l[0] != '#']
        for line in lines_to_keep:
            if line[1] == "SIMPLE_RADIAL":
                out_cameras.append(Camera(int(line[0]), float(line[4]), float(line[4]), float(line[5]), float(line[6])))
            else:
                out_cameras.append(Camera(int(line[0]), float(line[4]), float(line[5]), float(line[6]), float(line[7])))
    return out_cameras


def parse_ransac_output(path):
    """
    Parsing the line and planes contained in the json output
    :param path: path to the json output file of RANSAC on lines
    :return: a list of planes and a list of lines parsed from the json file
    """
    # Parse json file
    with open(path, 'r') as json_file:
        json_obj = json.load(json_file)
        planes = [Plane(o["normal"], o["inlier"]) for o in json_obj["planes"]]
        lines = [Line([int(x) for x in o["plane_index"]], np.array([float(x) for x in o["pt1"]]),
                      np.array([float(x) for x in o["pt2"]]),
                      [np.array([float(x) for x in pt]) for pt in o["pt_views"]],
                      [[float(x) for x in r] for r in o["residual_per_pt_view"]]
                      if "residual_per_pt_view" in o else []) for o in json_obj["lines"]]
    return planes, lines


def parse_lines_input(path):
    """
    Parsing the line and planes contained in the json output
    :param path: path to the json output file of RANSAC on lines
    :return: a list of lines parsed from the json file
    """
    # Parse json file
    with open(path, 'r') as json_file:
        json_obj = json.load(json_file)
        lines = [Line([], np.array([float(x) for x in o["pt1"]]),
                      np.array([float(x) for x in o["pt2"]]),
                      [np.array([float(x) for x in pt]) for pt in o["pt_views"]],
                      [[float(x) for x in r] for r in o["residual_per_pt_view"]]
                      if "residual_per_pt_view" in o else [])
                 for o in json_obj["lines"]]
    return lines


def parse_lines_l3dpp(line_txt_file):
    """
    Parsing the lines contained in the Line3d++ txt output file
    :param line_txt_file: path to the txt output file of Line3d++
    :return: a list of lines parsed from the json file (including the residuals)
    """
    with open(line_txt_file, "r") as line_txt:
        lines_out = []
        lines = csv.reader(line_txt)
        for line in lines:
            line_elems = iter(line[0].split())
            cur_sublines = []
            nb_lines = int(next(line_elems))
            for line_id in range(nb_lines):
                pt1 = np.array([float(next(line_elems)), float(next(line_elems)), float(next(line_elems))])
                pt2 = np.array([float(next(line_elems)), float(next(line_elems)), float(next(line_elems))])
                cur_sublines.append((pt1, pt2))
            nb_pt_views = int(next(line_elems))
            pt_views = []
            residuals_per_pt_view = []
            for pt_view_id in range(nb_pt_views):
                pt_views.append(int(next(line_elems)))
                next(line_elems)  # We drop the segment ID
                residuals_per_pt_view.append(
                    [int(float(next(line_elems))), int(float(next(line_elems))), int(float(next(line_elems))),
                     int(float(next(line_elems)))])
            for subline in cur_sublines:
                cur_line = Line([], subline[0], subline[1], pt_views)
                cur_line.residuals_per_pt_view = residuals_per_pt_view
                lines_out.append(cur_line)

        return lines_out


def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    M = quaternion_matrix([1, 0, 0, 0])
    numpy.allclose(M, numpy.identity(4))
    True
    M = quaternion_matrix([0, 1, 0, 0])
    numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
    True

    """
    _EPS = np.finfo(float).eps * 4.0
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]],
        [q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]],
        [q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]]])


def parse_nvm_file(nvm_path):
    """
    Parse a nvm file
    :param nvm_path: path to the nvm file
    :return: K matrix, and the list of Rotation and Translation [R, T] for each camera
    """
    K = []
    rot_trans = []
    with open(nvm_path, 'r') as nvm_file:
        lines = csv.reader(nvm_file)
        # Reading K
        k_line = [float(x) for x in next(lines)[0].split()[2:]]
        K = np.array([[k_line[0], 0., k_line[1]], [0., k_line[2], k_line[3]], [0., 0., 1.]])
        # Read number of cameras
        curLine = next(lines)
        while len(curLine) == 0 or curLine[0] == " ":
            curLine = next(lines)
        nb_cams = int(curLine[0])
        # Read the camera poses
        for i in range(nb_cams):
            curCam = [float(x) for x in next(lines)[0].split()[2:]]
            curRotation = inv(quaternion_matrix([curCam[0], curCam[1], curCam[2], curCam[3]]))
            curTranslation = [curCam[4], curCam[5], curCam[6]]
            rot_trans.append([curRotation, curTranslation])
    return K, rot_trans


def lines_to_txt(lines, file_stream):
    for line in lines:
        file_stream.write("1 ")
        file_stream.write("%s %s %s " % (str(line.pt1[0]), str(line.pt1[1]), str(line.pt1[2])))
        file_stream.write("%s %s %s " % (str(line.pt2[0]), str(line.pt2[1]), str(line.pt2[2])))
        file_stream.write(str(len(line.pt_views)) + " ")
        for i, pt_view in enumerate(line.pt_views):
            file_stream.write(str(pt_view) + " 0 ")
            res = line.residuals_per_pt_view[i]
            file_stream.write("%s %s %s %s " % (str(res[0]), str(res[1]), str(res[2]), str(res[3])))
        file_stream.write('\n')
