import numpy as np
from numpy import linalg as LA
import open3d as o3d
import scipy.spatial as spatial
from scipy.spatial import distance
import os
import traceback
from pathlib import Path
from tools import common_tools
from tools import pcd_io

def create_points_geometry(coords, defaultColor:list=None):
    coords_pcl = o3d.geometry.PointCloud()
    coords_pcl.points = o3d.utility.Vector3dVector(coords)
    if (defaultColor is not None):
        coords_pcl.paint_uniform_color(defaultColor)
    return coords_pcl

def select_outliers(inFile, inRefFile, maxDistance:float, deleteGeometry:bool=False):
    coords = common_tools.read_point_cloud(inFile)
    coords_pcl = create_points_geometry(coords, [0.0, 0.0, 1.0])
    refCoords = common_tools.read_point_cloud(inRefFile)
    refCoords_pcl = create_points_geometry(refCoords, [0.0, 1.0, 0.0])
    dists = coords_pcl.compute_point_cloud_distance(refCoords_pcl)
    dists = np.asarray(dists)
    if (not deleteGeometry):
        for i in range(0, len(dists)):
            if (dists[i] > maxDistance):
                coords_pcl.colors[i] = [1.0, 0.0, 1.0]
    else:
        coords_pcl = None
        refCoords_pcl = None
    return dists, coords, coords_pcl, refCoords_pcl

def delete_outliers(dists:list, coords:list, maxDistance:float):
    filteredCoords:list = []
    for i in range(0, len(coords)):
        if (dists[i] <= maxDistance):
            filteredCoords.append(coords[i])
    #distsFiltered:list = [n for n in dists if n > maxDistance]
    percent = 100.0 * (len(filteredCoords) / len(coords))
    print(f'filtered {len(filteredCoords)} out of {len(coords)} points ({percent:.2f}%)')
    return filteredCoords

def select_and_remove_outliers(inFile, inRefFile, outFile, maxDistance:float) -> list:
    dists, coords, _, _ = select_outliers(inFile, inRefFile, maxDistance, True)
    filteredCoords:list = delete_outliers(dists, coords, maxDistance)
    pcd_io.coords_to_file(filteredCoords, outFile)
    return filteredCoords

# Align a point cloud to a reference point cloud.
def point_cloud_registration(inFile, inRefFile, outFile=None, voxel_size=0.01, deleteGeometry:bool=False):
    coords = common_tools.read_point_cloud(inFile)
    refCoords = common_tools.read_point_cloud(inRefFile)
    minRef, maxRef = common_tools.calculate_min_max(refCoords)
    minBad, maxBad = common_tools.calculate_min_max(coords)
    for i in range(0, len(coords), 1):
        coords[i][0] = common_tools.scale_to_new_range(coords[i][0], minBad[0],maxBad[0], minRef[0],maxRef[0])
        coords[i][1] = common_tools.scale_to_new_range(coords[i][1], minBad[1],maxBad[1], minRef[1],maxRef[1])
        coords[i][2] = common_tools.scale_to_new_range(coords[i][2], minBad[2],maxBad[2], minRef[2],maxRef[2])

    coords_pcl = create_points_geometry(coords, [1.0, 1.0, 0.0])
    refCoords_pcl = create_points_geometry(refCoords, [0.0, 1.0, 0.0])
    radius_normal = voxel_size * 2
    coords_pcl.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=60))
    refCoords_pcl.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=60))
    
    threshold = voxel_size * 2
    init_transformation = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])
    
    local_result = o3d.pipelines.registration.registration_icp(
        coords_pcl, refCoords_pcl, threshold, init_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=True),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
    
    #print(f'transformation={local_result.transformation}')
    coords_pcl.transform(local_result.transformation)
    if (outFile is not None):
        coords = np.asarray(coords_pcl.points)
        pcd_io.coords_to_file(coords, outFile)
    if (deleteGeometry):
        coords_pcl = None
        refCoords_pcl = None
    return coords_pcl, refCoords_pcl
