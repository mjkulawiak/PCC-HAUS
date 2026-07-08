import numpy as np
from numpy import linalg as LA
import open3d as o3d
import scipy.spatial as spatial
from scipy.spatial import distance, cKDTree
import os
import traceback
from pathlib import Path
import datetime
import random
from tools import common_tools
from tools import pcd_io

def find_nearest_points(array1:list, array2:list):
    """For each point in array1 find the nearest point in array2."""
    a1 = np.asarray(array1)
    a2 = np.asarray(array2)
    tree = cKDTree(a2)
    _, idx = tree.query(a1, k=1)
    return a2[idx]

def calc_weighted_distances(array1:list, array2:list, z_weight:float=0.5):
    """
    Return distances measured with the following equation:
    dist = x^2 + y^2 + (z^2)*z_weight
    """
    arr1 = np.asarray(array1, dtype=np.float32)
    arr2 = np.asarray(array2, dtype=np.float32)
    diff = (arr2 - arr1)**2
    diff[:, 2] *= z_weight
    return np.sum(diff, axis=1)

def create_points_geometry(coords, defaultColor:list=None):
    coords_pcl = o3d.geometry.PointCloud()
    coords_pcl.points = o3d.utility.Vector3dVector(coords)
    if (defaultColor is not None):
        coords_pcl.paint_uniform_color(defaultColor)
    return coords_pcl

def select_sparse_roofs(coords, roof_dot_threshold:float=0.5, ignore_ratio:float=0.0, normals_radius:float=0.07,
                        normals_max_neighbours:float=50, distance_threshold:float=0.03, ransac_n=3, num_iterations=1000,
                        min_plane_points=200, nb_neighbors=30, std_ratio=2.0, ransac_seed=0) -> tuple[list, list]:
    
    remaining_pcd = create_points_geometry(coords)
    remaining_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=normals_radius,
            max_nn=normals_max_neighbours
        )
    )

    roofs = []
    walls = []
    up_vector = np.array([0.0, 0.0, 1.0])

    # Step 1: Detect planes with RANSAC.
    while True:
        # Note: Open3D random seed is singleton and must be set before each segmentation.
        # Plane segmentation was made deterministic in Open3D v0.19.
        o3d.utility.random.seed(ransac_seed)
        plane_model, inliers = remaining_pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        if len(inliers) < min_plane_points:
            break

        normal = np.array(plane_model[:3])
        normal /= np.linalg.norm(normal)
        dot = abs(np.dot(normal, up_vector))

        if dot > roof_dot_threshold:
            roofs.append(np.asarray(remaining_pcd.points)[inliers])
        else:
            walls.append(np.asarray(remaining_pcd.points)[inliers])

        remaining_pcd = remaining_pcd.select_by_index(inliers, invert=True)
        if len(remaining_pcd.points) < min_plane_points:
            break
    
    roofs = np.vstack(roofs)
    if (len(walls) > 0):
        walls = np.vstack(walls)
    
    # Step 2: Outlier removal.
    temp_pcd = create_points_geometry(roofs)
    filtered_pcd, _ = temp_pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )
    roofs_filtered = np.asarray(filtered_pcd.points)
    #outliers_mask = np.all(~np.isclose(roofs_filtered[:, None], roofs, atol=1e-6), axis=-1).any(axis=1)
    outliers_mask = np.array([~np.any(np.all(np.isclose(point, roofs_filtered, atol=1e-5), axis=1) ) for point in roofs])
    outlier_points = roofs[outliers_mask]

    # Step 3: Filter roofs by normals.
    roofs_pcl = create_points_geometry(roofs_filtered, [0.0, 0.0, 1.0])
    roofs_pcl.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=normals_radius,
            max_nn=normals_max_neighbours
        )
    )
    temp_roofs, temp_walls = _filter_by_normals(roofs_pcl, roof_dot_threshold, up_vector)
    roofs_filtered = np.vstack(temp_roofs)
    if (len(walls) > 0):
        walls = np.vstack( (temp_walls, walls, outlier_points) )
    else:
        walls = np.vstack( (temp_walls, outlier_points) )
    '''
    upside = abs( np.dot(np.asarray(roofs_pcl.normals), np.asarray(up_vector)) )
    for i in range(0, len(roofs_filtered)):
        if (upside[i] >= roof_dot_threshold):
            roofs_pcl.colors[i][1] = 1.0
    return roofs_pcl
    '''

    # Optional step: Remove random roof points for a more uniform spatial distribution.
    if (ignore_ratio > 0):
        numberOfRemoved = int(len(roofs_filtered) * ignore_ratio)
        toRemove = set( random.sample(range(len(roofs_filtered)), numberOfRemoved) )
        temp_roofs = [x for i,x in enumerate(roofs_filtered) if not i in toRemove]
        temp_walls = [x for i,x in enumerate(roofs_filtered) if i in toRemove]
        roofs_filtered = temp_roofs
        walls = np.vstack( (temp_walls, walls) )
    
    print(f'Detected {len(roofs)} roof points and {len(walls)} wall points.')
    return roofs_filtered, walls

def select_outliers(inFile, inRefFile, maxDistance:float, z_weight:float=1.0, deleteGeometry:bool=False):
    coords = common_tools.read_point_cloud(inFile)
    refCoords = common_tools.read_point_cloud(inRefFile)

    if (z_weight < 0.999 or z_weight > 1.001):
        maxDistance *= maxDistance
        nearest = find_nearest_points(coords, refCoords)
        dists = calc_weighted_distances(coords, nearest, z_weight)
    else:
        coords_pcl = create_points_geometry(coords)
        refCoords_pcl = create_points_geometry(refCoords)
        dists = coords_pcl.compute_point_cloud_distance(refCoords_pcl)
    dists = np.asarray(dists)

    if (not deleteGeometry):
        coords_pcl.paint_uniform_color([0.0, 0.0, 1.0])
        refCoords_pcl.paint_uniform_color([0.0, 1.0, 0.0])
        for i in range(0, len(dists)):
            if (dists[i] > maxDistance):
                coords_pcl.colors[i] = [1.0, 0.0, 1.0]
    else:
        coords_pcl = None
        refCoords_pcl = None
    return dists, coords, coords_pcl, refCoords_pcl

def delete_outliers(dists:list, coords:list, maxDistance:float, z_weight:float=1.0):
    filteredCoords:list = []
    if (z_weight < 0.999 or z_weight > 1.001):
        maxDistance *= maxDistance
    for i in range(0, len(coords)):
        if (dists[i] <= maxDistance):
            filteredCoords.append(coords[i])
    #distsFiltered:list = [n for n in dists if n > maxDistance]
    percent = 100.0 * (len(filteredCoords) / len(coords))
    print(f'preserved {len(filteredCoords)} out of {len(coords)} points ({percent:.2f}%)')
    return filteredCoords

def select_and_remove_outliers(inFile, inRefFile, outFile, maxDistance:float, z_weight:float=1.0) -> list:
    #print(f'maxDistance={maxDistance}; z_weight={z_weight};')
    dists, coords, _, _ = select_outliers(inFile, inRefFile, maxDistance, z_weight, True)
    filteredCoords:list = delete_outliers(dists, coords, maxDistance, z_weight)
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
