# Point cloud completion methods

This directory contains a collection of acknowledged state-of-the-art point cloud completion frameworks. Several new features were implemented in order to improve their robustness, with the major contributions listed below:
1. The functionality of SeedFormer and SVDFormer was extended to provide support for processing custom partial point clouds for models trained on ShapeNet-55, as the original code relied on automatic generation of incomplete data from fixed viewpoints;
2. Better exception handling was implemented for all completion methods to prevent situations when a random error interrupts the processing of all remaining files;
3. The user interface for PoinTr, provided by the Building-PCC project, was partially rewritten to improve support for boolean data types and allow for storing results without the need to create separate sub-directories;
4. Point clouds reconstructed by SVDFormer are now saved on the disk, instead of being used only for metrics calculation;
5. Support for exporting .xyz files was added for both PoinTr implementations;
6. Farthest Point Sampling is now automatically performed for SVDFormer if the input data consists of more than 2048 points.

## Environment Setup
Please refer to the [Wiki](https://github.com/mjkulawiak/PCC-HAUS/wiki/Installation-and-Environment) for the initial setup, then visit one of the original repositories for detailed installation instructions:
- [Building-PCC](https://github.com/tudelft3d/Building-PCC-Building-Point-Cloud-Completion-Benchmarks)
- [PoinTr](https://github.com/yuxumin/PoinTr)
- [SeedFormer](https://github.com/hrzhou2/seedformer)
- [SVDFormer](https://github.com/czvvd/SVDFormer_PointSea)
