# Prerequisites 

* Download and install this modified version of [Line3d++](https://github.com/palanglois/Line3d-_with_mlsd)
* The scripts in this directory require python with numpy

# Workflow 

Define the path to the line_based_recons directory (adapt the path to the directory where you cloned the repo): 

    LINE_RECONS=/path/to/line_based_recons
    
Download the example calibrated images: 

    wget http://imagine.enpc.fr/~langloip/data/data-line-surface-reconstruction.zip

Uncompress it : 

    unzip data-line-surface-reconstruction.zip

Run Line3d++

    runLine3Dpp_vsfm -i data-line-surface-reconstruction/HouseInterior/images/ -o . -m data-line-surface-reconstruction/HouseInterior/calibration.nvm -p 2.5

Convert the lines and calibrations into an input file for `ransac_on_lines` thanks to the Python script in this repository:

    python $LINE_RECONS/full_examples/NvmToRansacInput.py --input Line3D++__W_FULL__N_10__sigmaP_2.5__sigmaA_10__epiOverlap_0.25__kNN_10__OPTIMIZED__vis_3.txt --nvm data-line-surface-reconstruction/HouseInterior/calibration.nvm --output .

You can now run `ransac_on_lines` to perform plane detection in the 3D line set: 

    $LINE_RECONS/build/ransac_on_lines -i line3dpp_output.json -o . -e 0.02 -n 50000 -v

Extract the residuals thanks to the Python script in this repository:

    python $LINE_RECONS/full_examples/NvmRansacOutToResidualReconsInput.py --input test_input_for_reconstruction.json --output . --nvm data-line-surface-reconstruction/HouseInterior/calibration.nvm
    
You can now run the line based reconstruction (if you don't know yet the optimal hyper-parameters for your model, it's better to run `line_based_recons_param` instead): 

    $LINE_RECONS/build/line_based_recons -i input_lines_from_residuals.json -o . -cp 10 -cv 1 -ca 0 -ce 0.01 -cc 0.1 -int -v
    
Evaluation is possible thanks to the `mesh_metrics` executable

    $LINE_RECONS/build/mesh_metrics data-line-surface-reconstruction/HouseInterior/ground_truth.ply cp_10_cv_1_ca_0_ce_0.01_cc_0.1.ply 1000000
    

