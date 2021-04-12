import argparse
import json
from os.path import join
from generic_functions.parse_files import parse_ransac_output, parse_nvm_file
import numpy as np
from scipy.linalg import inv, norm


def invert_point(x, rot, trans):
    return np.matmul(inv(rot), (x - trans))


def find_pov_index(rot_trans, pt_view):
    # find the point of view index
    pov_index = None
    min_dist = float("inf")
    for k in range(len(rot_trans)):
        cur_dist = norm(rot_trans[k][1] - pt_view)
        if cur_dist < min_dist:
            min_dist = cur_dist
            pov_index = k
            if cur_dist == 0: break
    if pov_index is None:
        print("Debug")
    return pov_index


def update_a_and_b(cur_rot_trans, cur_line, cur_residual, inv_k):
    """
    Compute the sub-segment of a 3D segment corresponding to a given 2D residual in a given pose.
    A 2D residual is a 2D detected line segment in an image which took part in the reconstruction of a
    line segment in the 3D space. The latter 3D line may be longer than the residual because it is the
    result of triangulation with detection from other images.

    :param cur_rot_trans: rotation and translation corresponding to the current pose
    :param cur_line: a Line object defining the 3D line
    :param cur_residual: [x1, y1, x2, y2] where (x1, y1) and (x2, y2) is a 2D residual of cur_line
    :param inv_k: the invert of the K matrix for the current pose
    :return: A, B, the end points of the sub-segment of cur_line corresponding to cur_residual
    """
    cor_A = invert_point(cur_line.pt1, cur_rot_trans[0], cur_rot_trans[1])
    cor_B = invert_point(cur_line.pt2, cur_rot_trans[0], cur_rot_trans[1])
    cor_a = np.matmul(inv_k, np.array([cur_residual[0], cur_residual[1], 1.]))
    cor_b = np.matmul(inv_k, np.array([cur_residual[2], cur_residual[3], 1.]))
    # Point a
    det_a = norm(cor_B - cor_A) ** 2 * norm(cor_a) ** 2 - np.dot(cor_B - cor_A, cor_a) ** 2
    num_t_a = -norm(cor_a) ** 2 * np.dot(cor_B - cor_A, cor_A) \
              + np.dot(cor_B - cor_A, cor_a) * np.dot(cor_A, cor_a)
    # Point b
    det_b = norm(cor_B - cor_A) ** 2 * norm(cor_b) ** 2 - np.dot(cor_B - cor_A, cor_b) ** 2
    num_t_b = -norm(cor_b) ** 2 * np.dot(cor_B - cor_A, cor_A) \
              + np.dot(cor_B - cor_A, cor_b) * np.dot(cor_A, cor_b)

    true_a = cur_line.pt1 + (num_t_a / det_a) * (cur_line.pt2 - cur_line.pt1)
    true_b = cur_line.pt1 + (num_t_b / det_b) * (cur_line.pt2 - cur_line.pt1)

    offset = (true_b - true_a) * 0.05

    return true_a + offset / 2, true_b - offset / 2


def compute_new_lines(lines, inv_k, rot_trans):
    """
    Given a set "lines" of 3D lines for which each line has a set of 2D residuals, return
    the (bigger) set "corrected_lines" of 3D lines such that
    corrected_lines = [un-project(residual) for line in lines for residual in line.residuals]

    :param lines: A set of 3D lines with information on its 2D residuals
    :param inv_k: The invert of the K internal calibration matrix of the used camera
    :param rot_trans: the set of rotation-translation pairs for each camera pose
    :return: a new set of lines as defined above
    """
    lines_out = []
    for i, cur_line in enumerate(lines):
        assert len(cur_line.pt_views) == len(cur_line.residuals_per_pt_view)
        for j, pt_view in enumerate(cur_line.pt_views):
            cur_residual = cur_line.residuals_per_pt_view[j]

            pov_index = find_pov_index(rot_trans, pt_view)
            true_a, true_b = update_a_and_b(rot_trans[pov_index], cur_line, cur_residual, inv_k)

            lines_out.append({"pt1": [x for x in true_a],
                              "pt2": [x for x in true_b],
                              "pt_views": [[x for x in rot_trans[pov_index][1]]],
                              "plane_index": cur_line.plane_index})
        if i % (len(lines) / 100) == 0:
            print("Processed %s lines out of %s" % (str(i), str(len(lines))))
    return lines_out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="json RANSAC output file")
    parser.add_argument("--output", help="Output folder", default="./output_results")
    parser.add_argument("--nvm", help="NVM file containing the SfM structure")
    args = parser.parse_args()

    # lines = parse_lines_l3dpp(args.input)
    planes, lines = parse_ransac_output(args.input)
    K, rot_trans = parse_nvm_file(args.nvm)
    print(K)
    invK = inv(K)
    lines_out = compute_new_lines(lines, invK, rot_trans)

    # Saving the file
    planes_out = []
    for plane in planes:
        planes_out.append({"inlier": plane.inlier, "normal": plane.normal})
    json_output = {"lines": lines_out, "planes": planes_out}
    with open(join(args.output, "input_lines_from_residuals.json"), "w") as out_stream:
        json.dump(json_output, out_stream)


if __name__ == '__main__':
    main()
