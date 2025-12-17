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
The process is semi-automatic and is performed in three steps:
1. Preprocessing of the partial data;
2. Executing the inference command of the preferred point cloud completion method;
3. Postprocessing the completion results.

Example scenarios are provided at the bottom of this page.

####  1. Perform preprocessing of the partial data
```
python pcc-haus.py sf_preproc <input_dir> <output_dir> \
[--reference_dir <dir>] \
[--optimal_points] \
[--max_points] \
[--prefix] \
[--create_variants] \
[-pcn]
```
Explanation of optional arguments:
- reference_dir: path to directory with ground-truth data
- optimal_points: optimal number of points per partitioned sector (default is 2048)
- max_points: maximum number of points allowed for each partitioned sector (default is 6144)
- prefix: prefix added to filenames of processed files (default is '00010000-')
- create_variants: repeat the entire partitioning four extra times to create variant sectors with different offsets (this option is recommended for a more uniform point distribution)
- pcn: use PCN format instead of ShapeNet-55

####  2. Run the point cloud completion inference command
You may need to update the configuration files for the preferred point cloud completion method before running the inference commands.

####  3. Perform postprocessing of the completion results
```
python pcc-haus.py sf_postproc <input_dir> <output_dir> \
[--prefix] \
[--create_variants] \
[--remove_duplicates] \
[--results_dirname] \
[--merge] \
[-bpcc]
```
Explanation of optional arguments:
- prefix: prefix used in filenames of processed files (default is '00010000-')
- create_variants: use the extra variant sectors created during preprocessing
- remove_duplicates: remove duplicate points
- results_dirname: name of the output directory with final results (default is 'results')
- merge: path to directory with the partial data, will be used to merge the final results with the input point clouds
- bpcc: use processing steps designated for files generated with Building-Point-Cloud-Completion

#### Other useful methods
The framework provides additional helper functions for converting and sampling point clouds. See the `pcc-haus.py` script for the full list of commands.

### Examples
All scenarios assume that the input data is under `partial_data/`, the ground-truth data is under `reference_data/` and all framework results will be saved under `processing_results/`.

#### Scenario 1: use the PCC-HAUS framework with SeedFormer or SVDFormer model trained on ShapeNet-55
1. Create partitioned data.
```
python pcc-haus.py sf_preproc partial_data processing_results --reference_dir reference_data --create_variants
```
2. Find the preprocessing results generated under `processing_results/data_npy/`. You will need to configure the point cloud completion method to use use this path and manually place the `test.txt` and `train.txt` files in one of its directories.
- For SeedFormer: the dataset paths are in the `train_shapenet55.py` file; the test.txt and train.txt files should be placed in the `codes/datasets/ShapeNet55-34/ShapeNet-55/` directory;
- For SVDFormer: the dataset paths are in the `config_55.py` file; the test.txt and train.txt files should be placed in the `datasets/ShapeNet-55/` directory;
3. Run the inference command. Example for SeedFormer:
```
python3 train_shapenet55.py --test --output 1 --pretrained ../pretrained/shapenet55
```
Example for SVDFormer:
```
python main_55.py --test
```
4. Find the completion results in a subdirectory under `outputs` and provide its path to the postprocessing command.
```
python pcc-haus.py sf_postproc outputs/00010000/ processing_results/ --create_variants --merge partial_data/
```
5. Locate the final results under `processing_results/results/`.

#### Scenario 2: use the PCC-HAUS framework with SeedFormer or SVDFormer model trained on PCN
1. Create partitioned data.
```
python pcc-haus.py sf_preproc partial_data processing_results --reference_dir reference_data --create_variants -pcn
```
2. Find the preprocessing results generated under `processing_results/data_pcd/`. You will need to configure the point cloud completion method to use use this path and use the contents of the generated .json file.
- For SeedFormer: the dataset paths are in the `train_pcn.py` file; the contents of the `codes/datasets/ShapeNet.json` file should be replaced with the contents of the generated .json;
- For SVDFormer: the dataset paths are in the `config_pcn.py` file; the contents of the `datasets/ShapeNet.json` file should be replaced with the contents of the generated .json;
3. Run the inference command. Example for SeedFormer:
```
python3 train_pcn.py --test --output 1 --pretrained ../pretrained/pcn
```
Example for SVDFormer:
```
python main_pcn.py --test
```
4. Find the completion results in a subdirectory under `outputs` and provide its path to the postprocessing command.
```
python pcc-haus.py sf_postproc outputs/00010000/ processing_results/ --create_variants --merge partial_data/
```
5. Locate the final results under `processing_results/results/`.

#### Scenario 3: use the PCC-HAUS framework with PoinTr or AdaPoinTr
1. Create partitioned data.
```
python pcc-haus.py sf_preproc partial_data processing_results --create_variants --prefix ""
```
2. Find the preprocessing results generated under `processing_results/data_npy/` and run the inference command. Examples:
```
python tools/inference.py cfgs/PCN_models/AdaPoinTr.yaml pretrained/AdaPoinTr_PCN.pth --pc_root processing_results/data_npy/ --out_pc_root output/
```
```
python tools/inference.py cfgs/ShapeNet55_models/PoinTr.yaml pretrained/pointr_training_from_scratch_c55_best.pth --pc_root processing_results/data_npy/ --out_pc_root output/
```
```
python tools/inference.py --model_config ./cfgs/BuildingNL_models/AdaPoinTr.yaml --model_checkpoint ./pretrained/AdaPoinTr/BNL_50k_600e/ckpt-best.pth --pc_root processing_results/data_npy/partial/ --out_pc_root output/ --no_subdirs --truth_root processing_results/data_npy/complete/
```
3. Run the postprocessing command.
```
python pcc-haus.py sf_postproc output/ processing_results/ --create_variants --prefix "" -bpcc --merge partial_data/
```
4. Locate the final results under `processing_results/results/`.
