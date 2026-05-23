# Point cloud completion methods

## Introduction
This directory contains a collection of acknowledged state-of-the-art point cloud completion frameworks. Several new features were implemented in order to improve their robustness, with the major contributions listed below:
1. The functionality of SeedFormer and SVDFormer was extended to provide support for processing custom partial point clouds for models trained on ShapeNet-55, as the original code relied on automatic generation of incomplete data from fixed viewpoints;
2. Better exception handling was implemented for all completion methods to prevent situations when a random error interrupts the processing of all remaining files;
3. Point clouds reconstructed by SVDFormer are now saved on the disk, instead of being used only for metrics calculation;
4. Support for exporting .xyz files was added for both PoinTr implementations;
5. Farthest Point Sampling is now automatically performed for SVDFormer if the input data consists of more than 2048 points.

### Original repositories
Addresses of the original repositories for each project are provided below:
- [PoinTr](https://github.com/yuxumin/PoinTr)
- [SeedFormer](https://github.com/hrzhou2/seedformer)
- [SVDFormer](https://github.com/czvvd/SVDFormer_PointSea)

## Python environment
All methods in this collection can be used within a shared Python environment with CUDA support. The following commands were tested with an Anaconda platform installed on EndeavourOS.
```
conda create --name svd python=3.11 -y
conda activate svd

pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu128
conda install -c nvidia cuda-toolkit=12.8.0
pip3 install -r requirements.txt
```
CUDA support can be tested like this:
```
python torchtest.py
nvcc --version
```
If no problems are detected, then you may proceed with the installation of the preferred point cloud completion method and its extensions.
