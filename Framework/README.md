# PCC-HAUS: Point Cloud Completion framework for High-Altitude Urban Scanning

## Getting started

### Requirements

- python >= 3.10
- numpy
- open3d
- scipy

```
pip install -r requirements.txt
```

### Usage
The process is automatic and performed in three steps:
1. Preprocessing of the partial data;
2. Configuring the preinstalled point cloud completion framework and executing its inference commands;
3. Postprocessing the completion results.

At least one point cloud completion framework must be installed in the system in order to perform the second step. In addition, a configuration file must be prepared, based on one of the provided presets. Example usage scenarios are provided at the bottom of this page.

####  Configuration file structure
Each preset file follows the JSON structure. It is recommended to pick one that is preconfigured for the preferred point cloud completion model and only change the root path provided in "fullRootDir".

Parameters related to the point cloud completion framework:
```
"type": name of the framework and the dataset used for training
"fullRootDir": path to the root directory to the framework
"runDir": subdirectory used for running the inference commands
"inferenceCommand": bash command used to start the inferencing process
"directoriesToClear": paths to subdirectories which will be cleared before inferencing
"partialDir": path to the subdirectory with input partial data
"completeDir": path to the subdirectory with reference complete data
"datasetConfigDir": path to subdirectory with configuration files used for ShapeNet-55 models
"outputsDir": path to the subdirectory with the point cloud completion results
```

Parameters related to PCC-HAUS processing:
```
"create_variants": repeat the partitioning process with extra offsets
"prefix": prefix for files with partitioned data
"copiedOutputsDir": directory inside <temp_results_dir> to which the point cloud completion results will be copied
"optimal_points": the exact number of points required for performing point cloud completion
"max_points": check the paper for "maximum value"
"remove_duplicates": remove redundant points which occupy virtually the same coordinates as their neighbors
"max_outlier_distances": values used for outlier removal; each result will be saved to a separate file
```

####  1. Perform preprocessing of the partial data
```
python pcc-haus.py sf_preproc <input_dir> <temp_results_dir> <preset_file> \
[--reference_dir <dir>]
```
Explanation of arguments:
- input_dir: path to directory with partial point clouds
- temp_results_dir: path to directory which will contain partitioned data
- preset_file: path to the preset JSON file
- (optional) reference_dir: path to directory with ground-truth data

####  2. Run the point cloud completion inference command
This step assumes that the preferred point cloud completion framework is already installed in the system. PCC-HAUS will automatically copy all input and output files. If you believe the processing is stuck, try interrupting the process and running the inference command directly to diagnose the problem.
```
python pcc-haus.py sf_inference <temp_results_dir> <preset_file>
```
Explanation of arguments:
- temp_results_dir: path to the directory with partitioned data
- preset_file: path to the preset JSON file

####  3. Perform postprocessing of the completion results
```
python pcc-haus.py sf_postproc <input_dir> <temp_results_dir> <preset_file> \
[--merge]
```
Explanation of arguments:
- input_dir: path to directory with point cloud completion results
- temp_results_dir: path to the directory with partitioned data
- preset_file: path to the preset JSON file
- (optional) merge: path to directory with the partial data, will be used to merge the final results with the input point clouds

### Other useful methods
The framework provides additional helper functions for converting and sampling point clouds. See the `pcc-haus.py` script for the full list of commands.

## Examples
All scenarios assume that the input data is under `partial_data/`, the ground-truth data is under `reference_data/` and all framework results will be saved under `processing_results/`.

### Scenario 1: use the PCC-HAUS framework with SeedFormer model trained on ShapeNet-55
1. Create partitioned data.
```
python pcc-haus.py sf_preproc partial_data processing_results _Presets/SeedFormer-SN55.json --reference_dir reference_data
```
2. Run the inference command.
```
python pcc-haus.py sf_inference processing_results _Presets/SeedFormer-SN55.json
```
3. Perform postprocessing on the point cloud completion results.
```
python pcc-haus.py sf_postproc processing_results/predicted processing_results _Presets/SeedFormer-SN55.json --merge partial_data
```

### Scenario 2: use the PCC-HAUS framework with AdaPoinTr model trained on PCN
1. Create partitioned data.
```
python pcc-haus.py sf_preproc partial_data processing_results _Presets/AdaPoinTr-PCN.json --reference_dir reference_data
```
2. Run the inference command.
```
python pcc-haus.py sf_inference processing_results _Presets/AdaPoinTr-PCN.json
```
3. Perform postprocessing on the point cloud completion results.
```
python pcc-haus.py sf_postproc processing_results/predicted processing_results _Presets/AdaPoinTr-PCN.json --merge partial_data
```
