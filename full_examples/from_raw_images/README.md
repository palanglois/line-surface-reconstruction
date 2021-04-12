# Prerequisites 

* Download and install [colmap](https://colmap.github.io/install.html).
* Download and install [Line3d++](https://github.com/manhofer/Line3Dpp)
* The scripts in this directory require python with numpy

# Workflow

Define the path to the line_based_recons directory (adapt the path to the directory where you cloned the repo): 

    LINE_RECONS=/path/to/line_based_recons

Download the example images (sample from [ETH3D](https://www.eth3d.net/)):

    wget http://imagine.enpc.fr/~langloip/data/Terrains.zip

Uncompress it : 

    unzip Terrains.zip

Prepare a directory for colmap

    mkdir recons

Run the colmap commands to calibrate the cameras and undistort the images

    colmap feature_extractor --database_path recons/database.db --image_path pictures
    colmap exhaustive_matcher --database_path recons/database.db
    mkdir recons/sparse
    colmap mapper --database_path recons/database.db --image_path pictures/ --output_path recons/sparse
    colmap model_converter --input_path recons/sparse/0 --output_path recons/ --output_type TXT
    colmap image_undistorter --image_path pictures/ --input_path recons/sparse/0 --output_path recons/sparse/0 --output_type COLMAP --max_image_size 2000
    
Note: recons/sparse should only contain a directory called `0`. Otherwise, calibration has failed.

Run Line3d++

    runLine3Dpp_colmap -i pictures/ -m recons/ -p 2.5

Note: the `-p` parameter has a huge influence on the line output, so don't hesitate to vary it from 2 to 50.
You can check the output in `recons/Line3D++/`. The `.stl` file can be opened with Meshlab: don't forget 
to uncheck `Unify Duplicated Vertices` when opening the file.

Convert the lines and calibrations into an input file for `ransac_on_lines` thanks to the Python script in this repository:

    python $LINE_RECONS/full_examples/colmapToRANSACInput.py --input recons/Line3D++/Line3D++__W_FULL__N_10__sigmaP_2.5__sigmaA_10__epiOverlap_0.25__kNN_10__OPTIMIZED__vis_3.txt --colmap recons/ --output .

You can now run `ransac_on_lines` to perform plane detection in the 3D line set: 

    $LINE_RECONS/build/ransac_on_lines -i line3dpp_output.json -o . -e 0.02 -n 50000 -v

Extract the residuals thanks to the Python script in this repository:

    python $LINE_RECONS/full_examples/colmapRansacOutToResidualReconsInput.py --input test_input_for_reconstruction.json --output . --colmap recons
    
You can now run the line based reconstruction (if you don't know yet the optimal hyper-parameters for your model, it's better to run `line_based_recons_param` instead): 

    $LINE_RECONS/build/line_based_recons -i input_lines_from_residuals.json -o . -cp 1 -cv 1 -ca 0 -ce 0.01 -cc 0 -vs 4 -int -v
