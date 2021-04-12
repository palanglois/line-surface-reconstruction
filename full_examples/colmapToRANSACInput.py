from generic_functions.parse_files import parse_lines_l3dpp, parse_colmap_images, quaternion_matrix
import argparse
from os.path import join, basename
import json
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Line3D++ text file")
    parser.add_argument("--colmap", help="Colmap calibrations folder (where images.txt cameras.txt "
                                         "and points3D.txt are)")
    parser.add_argument("--output", help="Output folder")
    args = parser.parse_args()

    # Load liens
    lines = parse_lines_l3dpp(args.input)

    # Load camera parameters
    images = parse_colmap_images(join(args.colmap, "images.txt"))
    centers = {x.id: -np.matmul(quaternion_matrix(x.quaternion).transpose(),
                                    np.array(x.translation)) for x in images}

    # Build the RANSAC Input json file
    json_object = {"original_L3Dpp_file": basename(args.input), "lines": []}
    for line in lines:
        line_obj = {"pt1": [float(x) for x in line.pt1],
                    "pt2": [float(x) for x in line.pt2]}
        pt_views_obj = [[float(x) for x in centers[pt_view]] for pt_view in line.pt_views]
        line_obj["pt_views"] = pt_views_obj
        residuals_obj = [[float(x) for x in residual] for residual in line.residuals_per_pt_view]
        line_obj["residual_per_pt_view"] = residuals_obj
        json_object["lines"].append(line_obj)
    with open(join(args.output, "line3dpp_output.json"), "w") as output_file:
        json.dump(json_object, output_file)


if __name__ == '__main__':
    main()
