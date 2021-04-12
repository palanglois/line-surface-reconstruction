from generic_functions.parse_files import parse_lines_l3dpp, parse_nvm_file
import argparse
import json
from os.path import join, basename


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="Line3D++ text file")
    parser.add_argument("--nvm", help="NVM file for retrieving camera poses")
    parser.add_argument("--output", help="Output folder")
    args = parser.parse_args()

    lines = parse_lines_l3dpp(args.input)
    _, rot_trans = parse_nvm_file(args.nvm)

    json_object = {"original_L3Dpp_file": basename(args.input), "lines": []}
    for line in lines:
        line_obj = {"pt1": [float(x) for x in line.pt1],
                    "pt2": [float(x) for x in line.pt2]}
        pt_views_obj = [[float(x) for x in rot_trans[pt_view][1]] for pt_view in line.pt_views]
        line_obj["pt_views"] = pt_views_obj
        residuals_obj = [[float(x) for x in residual] for residual in line.residuals_per_pt_view]
        line_obj["residual_per_pt_view"] = residuals_obj
        json_object["lines"].append(line_obj)
    with open(join(args.output, "line3dpp_output.json"), "w") as output_file:
        json.dump(json_object, output_file)


if __name__ == '__main__':
    main()

