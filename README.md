# [Surface Reconstruction from 3D Line Segments](http://imagine.enpc.fr/~langloip/index.html?page=line_recons)
**Surface reconstruction from 3d line segments. [[Paper]](https://arxiv.org/pdf/1911.00451.pdf) [[Supplementary Material]](http://imagine.enpc.fr/~langloip/data/3DV-2019-Langlois-et-al_supp.pdf) <br>**
Langlois, P. A., Boulch, A., & Marlet, R. <br>In 2019 International Conference on 3D Vision (3DV) (pp. 553-563). IEEE.
![Project banner](http://imagine.enpc.fr/~langloip/data/banner3DV.png)


 
## Installation
* [IMPORTANT NOTE] The plane arrangement is given as a Linux x64 binary. Please let us know if you need it for an other platform/compiler or if you have issues with it.
* MOSEK 8 : 
    * [Download](https://www.mosek.com/downloads/8.1.0.83/) 
	* [Installation instructions](https://docs.mosek.com/8.1/cxxfusion/install-interface.html). 
	* [Request a license](https://www.mosek.com/products/academic-licenses/) (free for academics), and put it in ~/mosek/mosek.lic. 
	* Set the mosek directory in the MOSEK\_DIR environment variable such that <MOSEK\_DIR>/8/tools/platform/linux64x86/src/fusion\_cxx is a valid path:
	 
	```export MOSEK_DIR=/path/to/mosek```
	* Make sure that the binaries are available at runtime:
	 
	```export LD_LIBRARY_PATH=$MOSEK_DIR/8/tools/platform/linux64x86/bin:$LD_LIBRARY_PATH```
* Clone this repository: `git clone https://github.com/palanglois/line-surface-reconstruction.git`
* Go to the directory: `cd line-surface-reconstruction`

* CGAL : Version 4.11 is required:
 
```
git clone https://github.com/CGAL/cgal.git external/cgal
cd external/cgal
git checkout releases/CGAL-4.11.3
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cd ../../..
```  

* Make a build directory: `mkdir build`
* Go to the build directory: `cd build`
* Prepare the project with cmake: `cmake -DCMAKE_BUILD_TYPE=Release ..`
* Compile the project: `make` 

## Examples

* Out of the box examples are available in [demo.sh](https://github.com/palanglois/line-surface-reconstruction/blob/master/demo.sh)

* An example of a full reconstruction procedure from a simple set of images is available [here](https://github.com/palanglois/line-surface-reconstruction/blob/master/full_examples/from_raw_images)

* A benchmark example for an artificial textureless scene (with quantitative evaluation) is available [here](https://github.com/palanglois/line-surface-reconstruction/blob/master/full_examples/from_nvm). 
	
## Programs

For every program, a simple documentation is available by running `./<program_name> -h`

* `ransac_on_lines` detects planes in a line set.
* `line_based_recons_param` performs reconstruction out of a set of lines and detected planes. Computing the linear program is time consuming, but optimizing is way faster. Therefore, this program 1st computes the linear program and enters a loop in which you can manually set the optimization parameters in order to find the optimal ones for your reconstruction.
* `line_based_recons` does the same as `line_based_recons_param` but the optimization parameters are set directly in the command line. Use it only if you know the optimal parameters for the reconstruction.
* `mesh_metrics` provides evaluation metrics between two meshes.

## Visualization

Reconstruction .ply files can be visualized directly in programs such as Meshlab or CloudCompare.

A simple [OpenGL viewer](https://github.com/palanglois/gl-recons-viewer) is available to directly visualize the json line files.

## Raw data

The raw data for Andalusian and HouseInterior is available [here](http://imagine.enpc.fr/~langloip/data/data-line-surface-reconstruction.zip).
For both examples, it includes the raw images as well as the full calibration in .nvm (VisualSFM) format.

For HouseInterior, a ground truth mesh is also available.

## License

Apart from the code located in the `external` directory, all the code is provided under the [GPL license](https://github.com/palanglois/line-surface-reconstruction/blob/master/LICENSE.txt).

The binaries and code provided in the `external/PolyhedralComplex` directory is provided under the [Creative Commons CC-BY-SA license](https://creativecommons.org/licenses/by-sa/4.0/legalcode.txt).

If these licenses do not suit your needs, please get in touch with us.


## Citing this work

    @inproceedings{langlois:hal-02344362,
    TITLE = {{Surface Reconstruction from 3D Line Segments}},
    AUTHOR = {Langlois, Pierre-Alain and Boulch, Alexandre and Marlet, Renaud},
    URL = {https://hal.archives-ouvertes.fr/hal-02344362},
    BOOKTITLE = {{2019 International Conference on 3D Vision (3DV)}},
    ADDRESS = {Qu{\'e}bec City, Canada},
    PUBLISHER = {{IEEE}},
    PAGES = {553-563},
    YEAR = {2019},
    MONTH = Sep,
    DOI = {10.1109/3DV.2019.00067},
    } 