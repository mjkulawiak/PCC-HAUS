# Point cloud completion methods

## Introduction
This directory contains a collection of acknowledged state-of-the-art point cloud completion methods. Several new features were implemented in order to improve their robustness, with the major contributions listed below:
1. The functionality of SeedFormer and SVDFormer was extended to provide support for processing custom partial point clouds for models trained on ShapeNet-55, as the original code relied on automatic generation of incomplete data from fixed viewpoints;
2. Better exception handling was implemented for all completion methods to prevent situations when a random error interrupts the processing of all remaining files;
3. The user interface for PoinTr, provided by the Building-PCC project, was partially rewritten to improve support for boolean data types and allow for storing results without the need to create separate sub-directories;
4. Point clouds reconstructed by SVDFormer are now saved on the disk, instead of being used only for metrics calculation;
5. Support for exporting .xyz files was added for both PoinTr implementations;
6. Farthest Point Sampling is now automatically performed for SVDFormer if the input data consists of more than 2048 points.

## Python environment
All methods in this collection can be used within a shared Python environment. The following commands were tested with an Anaconda platform installed on Manjaro.
```
conda create --name seed python=3.10 -y
conda activate seed

pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

conda install nvidia/label/cuda-11.8.0::cuda
conda install nvidia/label/cuda-11.8.0::cuda-cudart-dev nvidia/label/cuda-11.8.0::cuda-cudart
conda install conda-forge::gcc_linux-64=11.4.0 gxx_linux-64=11.4.0
conda install nvidia/label/cuda-11.8.0::cuda-nvcc
conda install nvidia/label/cuda-11.8.0::libcusparse-dev nvidia/label/cuda-11.8.0::libcusparse
conda install nvidia/label/cuda-11.8.0::libcublas-dev nvidia/label/cuda-11.8.0::libcublas

pip3 install -r requirements.txt
```
The following command must be called each time the `seed` environment is activated:
```
export CUDA_HOME=$CONDA_PREFIX
```
CUDA support can be tested like this:
```
python torchtest.py
nvcc --version
```
If no problems are detected, then you may proceed with the installation of the preferred point cloud completion method and its extensions.
