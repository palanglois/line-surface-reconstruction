import argparse
from generic_functions.parse_files import parse_ransac_output, parse_colmap_cameras, parse_colmap_images, \
    quaternion_matrix
import numpy as np
from numpy.linalg import inv, norm
from os.path import join
import json


def invertPoint(x, rot, trans):
    return np.matmul(inv(rot), (x - trans))


def find_pov_index(rot_trans, pt_view):
    # find the point of view index
    pov_index = None
    min_dist = float("inf")
    for k in rot_trans.keys():
        cur_dist = norm(rot_trans[k][1] - pt_view)
        if cur_dist < min_dist:
            min_dist = cur_dist
            pov_index = k
            if cur_dist == 0: break
    return pov_index


def updateAandB(rot_trans, cur_line, pov_index, cur_residual, invK):
    corA = invertPoint(cur_line.pt1, rot_trans[pov_index][0], rot_trans[pov_index][1])
    corB = invertPoint(cur_line.pt2, rot_trans[pov_index][0], rot_trans[pov_index][1])
    cora = np.matmul(invK, np.array([cur_residual[0], cur_residual[1], 1.]))
    corb = np.matmul(invK, np.array([cur_residual[2], cur_residual[3], 1.]))
    # Point a
    det_a = norm(corB - corA) ** 2 * norm(cora) ** 2 - np.dot(corB - corA, cora) ** 2
    num_lamb_a = np.dot(cora, corA) * norm(corB - corA) ** 2 \
                 - np.dot(corB - corA, corA) * np.dot(corB - corA, cora)
    num_t_a = -norm(cora) ** 2 * np.dot(corB - corA, corA) \
              + np.dot(corB - corA, cora) * np.dot(corA, cora)
    # Point b
    det_b = norm(corB - corA) ** 2 * norm(corb) ** 2 - np.dot(corB - corA, corb) ** 2
    num_lamb_b = np.dot(corb, corA) * norm(corB - corA) ** 2 \
                 - np.dot(corB - corA, corA) * np.dot(corB - corA, corb)
    num_t_b = -norm(corb) ** 2 * np.dot(corB - corA, corA) \
              + np.dot(corB - corA, corb) * np.dot(corA, corb)

    newA_1 = corA + (num_t_a / det_a) * (corB - corA)
    newA_2 = (num_lamb_a / det_a) * cora
    worked = True
    if norm(newA_1 - newA_2) > 1.:
        print(norm(newA_1 - newA_2))
        worked = False

    newB_1 = corA + (num_t_b / det_b) * (corB - corA)
    newB_2 = (num_lamb_b / det_b) * corb

    if norm(newA_1 - newA_2) > 1.:
        print(norm(newB_1 - newB_2), "\n")

    trueA = cur_line.pt1 + (num_t_a / det_a) * (cur_line.pt2 - cur_line.pt1)
    trueB = cur_line.pt1 + (num_t_b / det_b) * (cur_line.pt2 - cur_line.pt1)

    offset = (trueB - trueA) * 0.05

    return trueA + offset / 2, trueB - offset / 2, worked


def compute_new_lines(lines, invKs, rot_trans):
    lines_out = []
    for i, cur_line in enumerate(lines):
        assert len(cur_line.pt_views) == len(cur_line.residuals_per_pt_view)
        for j, pt_view in enumerate(cur_line.pt_views):
            cur_residual = cur_line.residuals_per_pt_view[j]

            pov_index = find_pov_index(rot_trans, pt_view)

            # Residuals are in 2D. We need to back-project them on the 3D lines to get 3D residuals.
            trueA, trueB, worked = updateAandB(rot_trans, cur_line, pov_index, cur_residual, invKs[pov_index])
            if not worked:
                print(pov_index)

            # Each residual becomes a new standalone 3D line with a single point of view
            lines_out.append({"pt1": [x for x in trueA],
                              "pt2": [x for x in trueB],
                              "pt_views": [[x for x in rot_trans[pov_index][1]]],
                              "plane_index": cur_line.plane_index})
        if i % 100 == 0: print(i)
    return lines_out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="json RANSAC output file")
    parser.add_argument("--output", help="Output folder", default="./output_results")
    parser.add_argument("--colmap", help="Colmap calibrations folder (where images.txt cameras.txt "
                                         "and points3D.txt are)")
    args = parser.parse_args()

    planes, lines = parse_ransac_output(args.input)

    # Loading camera parameters
    images = parse_colmap_images(join(args.colmap, "images.txt"))
    cameras = parse_colmap_cameras(join(args.colmap, "cameras.txt"))

    # Loading the intrinsic camera parameters
    Ks = {}
    rot_trans = {}
    for image in images:
        cam = [c for c in cameras if c.cam_id == image.cam_id][0]
        rotation = -inv(quaternion_matrix(image.quaternion))
        translation = -np.matmul(quaternion_matrix(image.quaternion).transpose(), np.array(image.translation))
        K = np.array([[cam.fx, 0, cam.cx],
                      [0, cam.fy, cam.cy],
                      [0, 0, 1]])
        Ks[cam.cam_id] = K
        rot_trans[cam.cam_id] = [rotation, translation]
    invKs = {}
    for key, item in Ks.items():
        invKs[key] = inv(item)

    # Extract the residuals
    lines_out = compute_new_lines(lines, invKs, rot_trans)

    # Saving the file
    planes_out = []
    for plane in planes:
        planes_out.append({"inlier": plane.inlier, "normal": plane.normal})
    json_output = {"lines": lines_out, "planes": planes_out}
    with open(join(args.output, "input_lines_from_residuals.json"), "w") as out_stream:
        json.dump(json_output, out_stream)


if __name__ == '__main__':
    main()
