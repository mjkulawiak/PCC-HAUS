import numpy as np
from pathlib import Path
import os
import shutil
import json
import copy
import open3d as o3d
import traceback
from tools import common_tools
from tools import distance

def coords_to_file(coords:list, outFile, scale:list=[1.0,1.0,1.0]):
    Path(outFile).parent.mkdir(parents=True, exist_ok=True)

    if outFile.lower().endswith('.xyz'):
        f = open(outFile, 'w')
        for xyz in coords:
            f.write(f'{xyz[0]*scale[0]:.6f} {xyz[1]*scale[1]:.6f} {xyz[2]*scale[2]:.6f}\n')
        f.close()
    
    elif outFile.lower().endswith('.obj'):
        f = open(outFile, 'w')
        for xyz in coords:
            f.write(f'v {xyz[0]*scale[0]:.6f} {xyz[1]*scale[1]:.6f} {xyz[2]*scale[2]:.6f}\n')
        f.close()
    
    elif outFile.lower().endswith('.pcd'):
        f = open(outFile, 'w')
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {len(coords)}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(coords)}\n')
        f.write('DATA ascii\n')
        for xyz in coords:
            f.write(f'{xyz[0]*scale[0]:.6f} {xyz[1]*scale[1]:.6f} {xyz[2]*scale[2]:.6f}\n')
        f.close()
    
    elif outFile.lower().endswith('.npy'):
        np.save(outFile, coords)
    
    elif outFile.lower().endswith('.ply'):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(coords)
        o3d.io.write_point_cloud(outFile, pc, write_ascii=ascii)
    
    else:
        lastDotIndex = outFile.rindex('.')
        extension = outFile[lastDotIndex:]
        print('Unsupported output extension:', extension)

def convert(inFile, outFile, scale:list=[1.0,1.0,1.0]):
    coords = common_tools.read_point_cloud(inFile, scale)
    if (coords is not None):
        coords_to_file(coords, outFile)

def batch_convert(inputDir, outputDir, outputExtension:str='.xyz', inputExtension:str='.*', scale:list=[1.0,1.0,1.0]):
    Path(outputDir).mkdir(parents=True, exist_ok=True)
    for path in os.listdir(inputDir):
        fullPath = os.path.join(inputDir, path)
        if os.path.isfile(fullPath) and (inputExtension == '.*' or fullPath.lower().endswith(inputExtension)):
            lastDotIndex = path.rindex('.')
            resultPath = os.path.join(outputDir, path[:lastDotIndex])+outputExtension
            print(f'Converting {path}')
            try:
                convert(fullPath, resultPath, scale)
            except Exception as e:
                print(f'ERROR: Could not process file: {fullPath}')
                print(traceback.format_exc())
                print(e)

def read_categories(filepath) -> list:
    categories = []
    try:
        with open(filepath, 'r') as file:
            for line in file:
                #print(line.strip())
                categories.append(line.strip())
    except Exception as e:
        print(e)
    return categories

def getDirectories(dirpath) -> list:
    return [ item for item in os.listdir(dirpath) if os.path.isdir(os.path.join(dirpath, item)) ]

def getFiles(dirpath, extension:str=None, cutExtension=True) -> list:
    print('Checking directory:', dirpath)
    result = []
    try:
        result = [ item for item in os.listdir(dirpath) if (extension is None or extension == '.*' or item.lower().endswith(extension)) ]
        if (cutExtension):
            for i in range(0, len(result), 1):
                lastDotIndex = result[i].rindex('.')
                result[i] = result[i][:lastDotIndex]
        print(' found', len(result), 'files')
    except Exception as e:
        #print(traceback.format_exc())
        print(f'getFiles: {e}')
    return result

def create_pcd_json(filepath):
    rootDir = Path(filepath).parent
    rootNode = []
    emptyObject = {
          "taxonomy_id": "",
          "taxonomy_name": "",
          "test": [],
          "train": [],
          "val": []
       }
    categories = read_categories(os.path.join(rootDir, 'category.txt'))
    taxonomies = getDirectories(os.path.join(rootDir, 'test', 'complete'))
    #print('categories:', str(categories))
    #print('taxonomies:', str(taxonomies))

    for i in range(0, len(taxonomies), 1):
        tempObject = copy.deepcopy(emptyObject)
        tempObject["taxonomy_id"] = taxonomies[i]
        tempObject["taxonomy_name"] = categories[i]
        tempObject["test"] = getFiles(os.path.join(rootDir, 'test', 'complete', taxonomies[i]), '.pcd')
        tempObject["train"] = getFiles(os.path.join(rootDir, 'train', 'complete', taxonomies[i]), '.pcd')
        tempObject["val"] = getFiles(os.path.join(rootDir, 'val', 'complete', taxonomies[i]), '.pcd')
        rootNode.append(tempObject)
    
    #filepath = os.path.join(rootDir, 'PCN.json')
    f = open(filepath, 'w')
    f.write(json.dumps(rootNode, indent=4)+'\n')
    f.close()

def normalize_by_reference(refGoodFile, refBadFile, fileToFix, outputFile):
    refGoodCoords = common_tools.read_point_cloud(refGoodFile)
    refBadCoords = common_tools.read_point_cloud(refBadFile)
    minRef, maxRef = common_tools.calculate_min_max(refGoodCoords)
    minBad, maxBad = common_tools.calculate_min_max(refBadCoords)

    coordsToFix = common_tools.read_point_cloud(fileToFix)
    fixedCoords = []
    for i in range(0, len(coordsToFix), 1):
        newX = common_tools.scale_to_new_range(coordsToFix[i][0], minBad[0],maxBad[0], minRef[0],maxRef[0])
        newY = common_tools.scale_to_new_range(coordsToFix[i][1], minBad[1],maxBad[1], minRef[1],maxRef[1])
        newZ = common_tools.scale_to_new_range(coordsToFix[i][2], minBad[2],maxBad[2], minRef[2],maxRef[2])
        fixedCoords.append([newX, newY, newZ])

    Path(outputFile).parent.mkdir(parents=True, exist_ok=True)
    coords_to_file(fixedCoords, outputFile)

def normalize_icp(inFile, inRefFile, outFile):
    distance.point_cloud_registration(inFile, inRefFile, outFile, 0.01, True)

def sample_file(inFile, outFile, minPointCount:int=2048, maxPointCount:int=4096, seed:int=None, scale:list=[1.0,1.0,1.0],
                keepOriginal:bool=False, randomRadius:float=0.1, useFps:bool=False):
    coords = common_tools.read_point_cloud(inFile, scale)
    point_cloud = common_tools.sample_data(coords, minPointCount, maxPointCount, keepOriginal, seed, randomRadius, useFps)
    coords_to_file(point_cloud, outFile)

def sample_dir(inputDir, outputDir, outputExtension:str='.xyz', minPointCount:int=2048, maxPointCount:int=4096, inputExtension:str='.*',
               randomRadius:float=0.1, useFps:bool=False):
    for path in os.listdir(inputDir):
        fullPath = os.path.join(inputDir, path)
        if os.path.isfile(fullPath) and (inputExtension == '.*' or fullPath.lower().endswith(inputExtension)):
            lastDotIndex = path.rindex('.')
            resultPath = os.path.join(outputDir, path[:lastDotIndex])
            #print('Processing:', resultPath, '(scale '+str(scale)+')')
            sample_file(inFile=fullPath, outFile=resultPath+outputExtension, minPointCount=minPointCount, maxPointCount=maxPointCount,
                        randomRadius=randomRadius, useFps=useFps)

def downsample_file(inFile, outFile, voxel_size:float=0.000001):
    coords = common_tools.read_point_cloud(inFile)
    point_cloud = common_tools.downsample_points(coords, voxel_size)
    coords_to_file(point_cloud, outFile)
