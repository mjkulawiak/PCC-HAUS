import numpy as np
import os
import sys
import shutil
import random
import open3d as o3d

def force_print_to_console(message, skip_log:bool=False):
    if (not skip_log):
        print(message)
    temp_stdout = sys.stdout
    temp_stderr = sys.stderr
    sys.stdout = sys.__stdout__
    sys.stderr = sys.__stderr__
    print(message)
    sys.stdout = temp_stdout
    sys.stderr = temp_stderr

def scale_to_new_range(value, old_min,old_max, new_min,new_max):
    old_range = (old_max - old_min)
    new_range = (new_max - new_min)
    if (old_range <= 0.0):
        return (new_min + new_max) * 0.5
    else:
        return ( (value-old_min) * (new_range/old_range) ) + new_min

def fit_in_range(value, oldMin,oldMax, newMin,newMax):
    return int(scale_to_new_range(value, oldMin,oldMax, newMin,newMax))

def clamp(value, minimum, maximum):
    if (value < minimum):
        value = minimum
    elif (value > maximum):
        value = maximum
    return value

def vector3_to_string(vector:list) -> str:
    return '[' + format(vector[0], '.6f') + ', ' + format(vector[1], '.6f') + ', ' + format(vector[2], '.6f') + ']'

def parse_vector3(s:str):
    s = s.strip().replace('[', '')
    s = s.replace(']', '')
    s = s.replace(' ', '')
    s = s.split(',')
    return [float(i) for i in s]

def calculate_min_max(coords) -> list:
    minX = float('inf')
    minY = float('inf')
    minZ = float('inf')
    maxX = float('-inf')
    maxY = float('-inf')
    maxZ = float('-inf')
    for p in coords:
        minX = min(p[0], minX)
        minY = min(p[1], minY)
        minZ = min(p[2], minZ)
        maxX = max(p[0], maxX)
        maxY = max(p[1], maxY)
        maxZ = max(p[2], maxZ)
    return [[minX, minY, minZ], [maxX, maxY, maxZ]]

def count_files(dirPath):
    count = 0
    for path in os.listdir(dirPath):
        fullPath = os.path.join(dirPath, path)
        if os.path.isfile(fullPath):
            if fullPath.lower().endswith('.xyz') or fullPath.lower().endswith('.obj'):
                count += 1
    return count

def clear_directory(dirPath):
    print(f'Clearing {dirPath.name}.')
    if (os.path.isdir(dirPath)):
        for filename in os.listdir(dirPath):
            fullPath = os.path.join(dirPath, filename)
            try:
                if os.path.isfile(fullPath) or os.path.islink(fullPath):
                    os.unlink(fullPath)
                elif os.path.isdir(fullPath):
                    shutil.rmtree(fullPath)
            except Exception as e:
                print(f'Failed to delete {fullPath}: {e}')

def read_xyz(filepath, scale:list=[1.0,1.0,1.0]) -> list:
    coords = []
    try:
        with open(filepath, 'r') as file:
            for line in file:
                #print(line.strip())
                x, y, z = line.strip().split(' ')
                x = float(x) * scale[0]
                y = float(y) * scale[1]
                z = float(z) * scale[2]
                coords.append([x, y, z])
    except Exception as e:
        print(e)
    return coords

def read_obj(filepath, scale:list=[1.0,1.0,1.0]) -> list:
    coords = []
    try:
        with open(filepath, 'r') as file:
            for line in file:
                #print(line.strip())
                if line.startswith('v '):
                    _, x, y, z = line.strip().split(' ')
                    x = float(x) * scale[0]
                    y = float(y) * scale[1]
                    z = float(z) * scale[2]
                    coords.append([x, y, z])
    except Exception as e:
        print(e)
    return coords

def read_pcd(filepath, scale:list=[1.0,1.0,1.0]) -> list:
    coords = []
    try:
        with open(filepath, 'r') as file:
            for line in file:
                #print(line.strip())
                if line[:1].isdigit() or line.startswith('-'):
                    x, y, z = line.strip().split(' ')
                    x = float(x) * scale[0]
                    y = float(y) * scale[1]
                    z = float(z) * scale[2]
                    coords.append([x, y, z])
    except Exception as e:
        print(e)
    return coords

def read_numpy(filepath, scale:list=[1.0,1.0,1.0]) -> list:
    coords = []
    try:
        coords = np.load(filepath)
        coords = coords.tolist()
        #print(str(coords))
        #print(str(coords[0][0]))
    except Exception as e:
        print(e)
    return coords

def read_ply(filepath, scale:list=[1.0,1.0,1.0]) -> list:
    coords = []
    try:
        pc = o3d.io.read_point_cloud(filepath)
        coords = np.asarray(pc.points)
        #coords = np.load(filepath)
        #print(str(coords))
        #print(str(coords[0][0]))
    except Exception as e:
        print(e)
    return coords

def read_point_cloud(filepath, scale:list=[1.0,1.0,1.0]) -> list:
    if filepath.lower().endswith('.xyz'):
        return read_xyz(filepath, scale)
    elif filepath.lower().endswith('.obj'):
        return read_obj(filepath, scale)
    elif filepath.lower().endswith('.pcd'):
        return read_pcd(filepath, scale)
    elif filepath.lower().endswith('.npy'):
        return read_numpy(filepath, scale)
    elif filepath.lower().endswith('.ply'):
        return read_ply(filepath, scale)
    else:
        lastDotIndex = filepath.rindex('.')
        extension = filepath[lastDotIndex:]
        print('Unsupported input extension:', extension)
        return None

def trim_data(pointCloud:list, targetPointCount:int) -> list:
    #print('Trimming:', str(len(pointCloud)), '->', str(targetPointCount))
    numberOfRemoved = len(pointCloud) - targetPointCount
    if (numberOfRemoved <= 0):
        return pointCloud
    toRemove = set( random.sample(range(len(pointCloud)), numberOfRemoved) )
    return [x for i,x in enumerate(pointCloud) if not i in toRemove]

def fps_subsampling(pointCloud:list, targetPointCount:int, keepOriginal:bool=False) -> list:
    coords_pcl = o3d.geometry.PointCloud()
    coords_pcl.points = o3d.utility.Vector3dVector(pointCloud)
    if (not keepOriginal):
        pointCloud.clear()
    pcd_down = coords_pcl.farthest_point_down_sample(targetPointCount)
    return np.asarray(pcd_down.points)

def expand_data(pointCloud:list, targetPointCount:int, randomRadius:float=0.1) -> list:
    result = pointCloud.copy()
    #print('Expanding:', str(len(result)), '->', str(targetPointCount))
    while (targetPointCount / len(result)) >= 2:
        for i in range(0, len(result)):
            newPoint = [result[i][0]+_random_plus_minus(randomRadius),
             result[i][1]+_random_plus_minus(randomRadius),
             result[i][2]+_random_plus_minus(randomRadius)]
            result.append(newPoint)
    numberOfAdded = targetPointCount - len(result)
    toAdd = set( random.sample(range(len(result)), numberOfAdded) )
    for i in toAdd:
        newPoint = [result[i][0]+_random_plus_minus(randomRadius),
         result[i][1]+_random_plus_minus(randomRadius),
         result[i][2]+_random_plus_minus(randomRadius)]
        result.append(newPoint)
    return result

def _random_plus_minus(randomRadius:float) -> float:
    return random.uniform(0.0, randomRadius*2.0) - randomRadius

def sample_data(pointCloud:list, minPointCount:int=2048, maxPointCount:int=4096, keepOriginal:bool=False,
                seed:int=None, randomRadius:float=0.1, useFps:bool=False) -> list:
    if (not keepOriginal):
        result = pointCloud
    else:
        result = pointCloud.copy()
    if (seed is not None):
        random.seed(seed)
    if len(result) > maxPointCount:
        if (not useFps):
            result = trim_data(result, maxPointCount)
        else:
            result = fps_subsampling(result, maxPointCount, keepOriginal)
    elif len(result) < minPointCount:
        result = expand_data(result, minPointCount, randomRadius)
    return result

def downsample_points(coords:list, voxel_size:float=0.000001):
    coords_pcl = o3d.geometry.PointCloud()
    coords_pcl.points = o3d.utility.Vector3dVector(coords)
    downpcd = coords_pcl.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(downpcd.points)

def copytree(src, dst):
    if not os.path.exists(dst):
        os.makedirs(dst)
    for item in os.listdir(src):
        s = os.path.join(src, item)
        d = os.path.join(dst, item)
        if os.path.isdir(s):
            copytree(s, d)
        else:
            if not os.path.exists(d) or os.stat(s).st_mtime - os.stat(d).st_mtime > 1:
                shutil.copy2(s, d)

def bboxCollide2D(min1:list, max1:list, min2:list, max2:list):
    if ( max1[0] < min2[0] or min1[0] > max2[0] or
		 max1[1] < min2[1] or min1[1] > max2[1] ):
        return False
    else:
        return True

def bboxCollide3D(min1:list, max1:list, min2:list, max2:list):
    if ( max1[0] < min2[0] or min1[0] > max2[0] or
		 max1[1] < min2[1] or min1[1] > max2[1] or
		 max1[2] < min2[2] or min1[2] > max2[2] ):
        return False
    else:
        return True

def bboxWithin2D(min1:list, max1:list, min2:list, max2:list):
    if ( min1[0] >= min2[0] and max1[0] <= max2[0] and
		 min1[1] >= min2[1] and max1[1] <= max2[1] ):
        return True
    else:
        return False

def bboxWithin3D(min1:list, max1:list, min2:list, max2:list):
    if ( min1[0] >= min2[0] and max1[0] <= max2[0] and
		 min1[1] >= min2[1] and max1[1] <= max2[1] and
         min1[2] >= min2[2] and max1[2] <= max2[2] ):
        return True
    else:
        return False

def hasPointsOutsideBorders2D(min1:list, max1:list, min2:list, max2:list):
    return (bboxCollide2D(min1,max1, min2,max2) and not bboxWithin2D(min1,max1, min2,max2))

def hasPointsOutsideBorders3D(min1:list, max1:list, min2:list, max2:list):
    return (bboxCollide3D(min1,max1, min2,max2) and not bboxWithin3D(min1,max1, min2,max2))

def path_without_extension(filepath):
    lastDotIndex = filepath.rindex('.')
    pathWithoutExt = filepath[:lastDotIndex]
    return pathWithoutExt
