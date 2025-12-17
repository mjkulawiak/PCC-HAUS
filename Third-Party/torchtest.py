import torch
print(f'torch.__version__ = {torch.__version__}')
print(f'torch.cuda.is_available() = {torch.cuda.is_available()}')
print(f'torch.version.cuda = {torch.version.cuda}')
print(f'torch.backends.cudnn.version() = {torch.backends.cudnn.version()}')
from torch.utils.cpp_extension import CUDA_HOME
print(f'CUDA_HOME = {CUDA_HOME}')
