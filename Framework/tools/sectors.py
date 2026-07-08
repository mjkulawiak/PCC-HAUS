import numpy as np
from pathlib import Path
import shutil
import os, sys
import copy
import traceback
import subprocess
import datetime
import json
from tools import common_tools
from tools import distance
from tools import pcd_io

# Single quadtree node.
class points_sector:
    def __init__(self, minBorders:list, maxBorders:list, refMinBorders:list=None, refMaxBorders:list=None,
                 parent=None, nodeHierarchy:list=[0], isScaled:bool=False):
        self.minBorders:list = minBorders.copy()
        self.maxBorders:list = maxBorders.copy()

        if refMinBorders is None:
            self.refMinBorders:list = minBorders.copy()
        else:
            self.refMinBorders:list = refMinBorders.copy()

        if refMaxBorders is None:
            self.refMaxBorders:list = maxBorders.copy()
        else:
            self.refMaxBorders:list = refMaxBorders.copy()
        
        self._recalculate_centers()
        self.nodeHierarchy:list = nodeHierarchy
        self.childSectors:list[points_sector] = [] # top-left, top-right, bottom-left, bottom-right
        self.parent = parent
        self.points:list = []
        self.descendantPointsNumber:int = 0
        self.isScaled:bool = isScaled
        #print(f'{str(self)} init.')
    
    def _recalculate_centers(self):
        self.centers:list = [(self.maxBorders[0]+self.minBorders[0])*0.5, (self.maxBorders[1]+self.minBorders[1])*0.5, (self.maxBorders[2]+self.minBorders[2])*0.5]

    def replace_data(self, anotherSector):
        self.minBorders = anotherSector.minBorders
        self.maxBorders = anotherSector.maxBorders
        self.refMinBorders = anotherSector.refMinBorders
        self.refMaxBorders = anotherSector.refMaxBorders
        self.centers = anotherSector.centers
        self.points = anotherSector.points
        self.isScaled = anotherSector.isScaled
    
    def _create_merged(self, anotherSector, newParent, newNodeHierarchy:list=[]):
        print(f' merging {self.nodeHierarchy} ({len(self.points)} pts, {self.descendantPointsNumber} dpts) with {anotherSector.nodeHierarchy} ({len(anotherSector.points)} pts, {anotherSector.descendantPointsNumber} dpts)')
        if (self.descendantPointsNumber > 0 or anotherSector.descendantPointsNumber > 0):
            print(f'  WARNING: descendant points detected! ({self.descendantPointsNumber}, {anotherSector.descendantPointsNumber})')
        newMinBorders:list = [min(self.minBorders[0], anotherSector.minBorders[0]),
                              min(self.minBorders[1], anotherSector.minBorders[1]),
                              min(self.minBorders[2], anotherSector.minBorders[2])]
        newMaxBorders:list = [max(self.maxBorders[0], anotherSector.maxBorders[0]),
                              max(self.maxBorders[1], anotherSector.maxBorders[1]),
                              max(self.maxBorders[2], anotherSector.maxBorders[2])]
        newRefMinBorders:list = [min(self.refMinBorders[0], anotherSector.refMinBorders[0]),
                                 min(self.refMinBorders[1], anotherSector.refMinBorders[1]),
                                 min(self.refMinBorders[2], anotherSector.refMinBorders[2])]
        newRefMaxBorders:list = [max(self.refMaxBorders[0], anotherSector.refMaxBorders[0]),
                                 max(self.refMaxBorders[1], anotherSector.refMaxBorders[1]),
                                 max(self.refMaxBorders[2], anotherSector.refMaxBorders[2])]
        resultSector = points_sector(newMinBorders, newMaxBorders, newRefMinBorders, newRefMaxBorders,
                                     newParent, newNodeHierarchy, self.isScaled)
        resultSector.points = self.points + anotherSector.points
        #resultSector.points.extend(self.points)
        #resultSector.points.extend(anotherSector.points)
        resultSector.parent = newParent
        return resultSector
    
    def _merge_inline(self, anotherSector, newNodeHierarchy:list=[]):
        print(f' merging {self.nodeHierarchy} ({len(self.points)} pts, {self.descendantPointsNumber} dpts)'
              + f' with {anotherSector.nodeHierarchy} ({len(anotherSector.points)} pts, {anotherSector.descendantPointsNumber} dpts)'
              + f' creating <{self.hierarchy_to_string(newNodeHierarchy)}>')
        if (self.descendantPointsNumber > 0 or anotherSector.descendantPointsNumber > 0):
            print(f'  WARNING: descendant points detected! ({self.descendantPointsNumber}, {anotherSector.descendantPointsNumber})')
        newMinBorders:list = [min(self.minBorders[0], anotherSector.minBorders[0]),
                              min(self.minBorders[1], anotherSector.minBorders[1]),
                              min(self.minBorders[2], anotherSector.minBorders[2])]
        newMaxBorders:list = [max(self.maxBorders[0], anotherSector.maxBorders[0]),
                              max(self.maxBorders[1], anotherSector.maxBorders[1]),
                              max(self.maxBorders[2], anotherSector.maxBorders[2])]
        newRefMinBorders:list = [min(self.refMinBorders[0], anotherSector.refMinBorders[0]),
                                 min(self.refMinBorders[1], anotherSector.refMinBorders[1]),
                                 min(self.refMinBorders[2], anotherSector.refMinBorders[2])]
        newRefMaxBorders:list = [max(self.refMaxBorders[0], anotherSector.refMaxBorders[0]),
                                 max(self.refMaxBorders[1], anotherSector.refMaxBorders[1]),
                                 max(self.refMaxBorders[2], anotherSector.refMaxBorders[2])]
        self.minBorders = newMinBorders
        self.maxBorders = newMaxBorders
        self.refMinBorders = newRefMinBorders
        self.refMaxBorders = newRefMaxBorders
        self.nodeHierarchy = newNodeHierarchy
        self.points.extend(anotherSector.points)
        return self
    
    def replace_points_from_cloud(self, externalCloud:list, maxPointCount:int, useRefBorders:bool=False):
        self.points.clear()
        for point in externalCloud:
            if (self._is_point_inside(point, useRefBorders)):
                self.points.append(point.copy())
        if (len(self.points) > maxPointCount):
            self.points = common_tools.trim_data(self.points, maxPointCount)
    
    def _is_point_inside(self, point:list, useRefBorders:bool=False):
        if (not useRefBorders):
            return (point[0] >= self.minBorders[0] and point[0] <= self.maxBorders[0]
                and point[1] >= self.minBorders[1] and point[1] <= self.maxBorders[1])
        else:
            return (point[0] >= self.refMinBorders[0] and point[0] <= self.refMaxBorders[0]
                and point[1] >= self.refMinBorders[1] and point[1] <= self.refMaxBorders[1])

    def recalculate_coords(self, newMinBorders:list, newMaxBorders:list, updateBorders:bool=True,
                           oldRecalculation:bool=False):
        if (oldRecalculation):
            for point in self.points:
                for axis in range(0, len(point)):
                    point[axis] = common_tools.scale_to_new_range(point[axis],
                                        self.minBorders[axis], self.maxBorders[axis],
                                        newMinBorders[axis], newMaxBorders[axis])
        else:
            self.refMinBorders, self.refMaxBorders = common_tools.calculate_min_max(self.points)
            if (len(self.points) > 0 and common_tools.hasPointsOutsideBorders3D(self.refMinBorders, self.refMaxBorders,
                                                                                self.minBorders, self.maxBorders)):
                print(f'  WARNING: sector has points outside its borders: {str(self)}')
            for point in self.points:
                for axis in range(0, len(point)):
                    point[axis] = common_tools.scale_to_new_range(point[axis],
                                        self.refMinBorders[axis], self.refMaxBorders[axis],
                                        newMinBorders[axis], newMaxBorders[axis])
        if (updateBorders):
            self.minBorders = newMinBorders.copy()
            self.maxBorders = newMaxBorders.copy()
            self._recalculate_centers()
        for sector in self.childSectors:
            sector.recalculate_coords(newMinBorders, newMaxBorders, updateBorders)
    
    def recalculate_back(self, updateBorders:bool=True, oldRecalculation:bool=False):
        self.recalculate_coords(self.refMinBorders, self.refMaxBorders, updateBorders, oldRecalculation)
        for sector in self.childSectors:
            sector.recalculate_back(updateBorders)
    
    def swap_borders_info(self):
        tempMinBorders = self.minBorders.copy()
        tempMaxBorders = self.maxBorders.copy()
        self.minBorders = self.refMinBorders
        self.maxBorders = self.refMaxBorders
        self.refMinBorders = tempMinBorders
        self.refMaxBorders = tempMaxBorders

    def add_point(self, point:list):
        if (self._is_point_inside(point)):
            self._updateParentPointsCount(1)
            if (len(self.childSectors) <= 0):
                self.points.append(point.copy())
            else:
                for sector in self.childSectors:
                    if (sector._is_point_inside(point)):
                        sector.add_point(point)
                        break
    
    def add_all_points(self, points:list):
        #if (len(self.childSectors) <= 0):
        #    self.points.extend(points)
        #    self._updateParentPointsCount(len(points))
        #else:
            for point in points:
                self.add_point(point)
    
    def clear_list(self):
        self.points.clear()
    
    def _updateParentPointsCount(self, numberToAdd:int):
        if (self.parent is not None):
            self.parent.descendantPointsNumber += numberToAdd
            self.parent._updateParentPointsCount(numberToAdd)
    
    def remove_child_sectors(self):
        for sector in self.childSectors:
            sector.remove_child_sectors()
        self.childSectors.clear()
    
    def get_points_count(self) -> int:
        return len(self.points)
    
    def copy_points(self, extendedList:list):
        for sector in self.childSectors:
            sector.copy_points(extendedList)
        extendedList.extend(self.points)
        return extendedList
    
    def collect_nodes(self, extendedList:list, includeEmpty:bool=False) -> list:
        for sector in self.childSectors:
            sector.collect_nodes(extendedList, includeEmpty)
        if (includeEmpty or len(self.points) > 0):
            extendedList.append(self)
        return extendedList

    def _initialize_children(self, clearExisting:bool=True):
        if (clearExisting):
            self.childSectors.clear()
        self.childSectors.append( points_sector([self.minBorders[0], self.minBorders[1], self.minBorders[2]],
                                                [self.centers[0], self.centers[1], self.maxBorders[2]],
                                                None, None, self, self.nodeHierarchy + [0]) )
        self.childSectors.append( points_sector([self.centers[0], self.minBorders[1], self.minBorders[2]],
                                                [self.maxBorders[0], self.centers[1], self.maxBorders[2]],
                                                None, None, self, self.nodeHierarchy + [1]) )
        self.childSectors.append( points_sector([self.minBorders[0], self.centers[1], self.minBorders[2]],
                                                [self.centers[0], self.maxBorders[1], self.maxBorders[2]],
                                                None, None, self, self.nodeHierarchy + [2]) )
        self.childSectors.append( points_sector([self.centers[0], self.centers[1], self.minBorders[2]],
                                                [self.maxBorders[0], self.maxBorders[1], self.maxBorders[2]],
                                                None, None, self, self.nodeHierarchy + [3]) )

    def divide(self, maxPointCount:int=6144):
        #print(f'[{self.hierarchy_to_string()}] dividing {len(self.points)} points.')
        self.childSectors.clear()
        if (len(self.points) > maxPointCount):
            self._initialize_children(False)
            for point in self.points:
                self.add_point(point)
            self.clear_list()
            for sector in self.childSectors:
                sector.divide(maxPointCount)
    
    def build_from_list(self, nodesList:list):
        # TODO: support merging scenarios
        #print(f'nodeHierarchy: {self.nodeHierarchy}')
        myDepth = len(self.nodeHierarchy)-1
        for node in nodesList:
            if (self.nodeHierarchy == node.nodeHierarchy):
                #print(f' replacing: {node.nodeHierarchy}')
                self.replace_data(node)
                nodesList.remove(node)
                break
            elif (len(node.nodeHierarchy) > len(self.nodeHierarchy) and node.nodeHierarchy[myDepth] == self.nodeHierarchy[myDepth]):
                if (len(self.childSectors) <= 0):
                    self._initialize_children(False)
        for child in self.childSectors:
            child.build_from_list(nodesList)

    def sample_points(self, targetPointCount:int=2048, randomRadius:float=0.001, useFps:bool=True, minPointsForSampling:int=1536):
        if (len(self.points) > minPointsForSampling):
            oldCount:int = len(self.points)
            self.points = common_tools.sample_data(pointCloud=self.points, minPointCount=targetPointCount, maxPointCount=targetPointCount,
                                                   randomRadius=randomRadius, useFps=useFps)
            newCount:int = len(self.points)
            if (newCount != oldCount):
                print(f'Sampled points in node ({self.nodeHierarchy}): {oldCount} -> {newCount}.')
        else:
            print(f'Not enough points for sampling node ({self.nodeHierarchy}): {len(self.points)} < {minPointsForSampling}.')
        for sector in self.childSectors:
            sector.sample_points(targetPointCount, randomRadius, useFps, minPointsForSampling)

    def hierarchy_to_string(self, hierarchy:list=None):
        if (hierarchy is None):
            hierarchy = self.nodeHierarchy
        result = ''
        for i in range(0, len(hierarchy)-1):
            result += str(hierarchy[i])+'.'
        result += str(hierarchy[len(hierarchy)-1])
        return result

    def __str__(self):
        return (
            f'{self.hierarchy_to_string()}: (minBord={common_tools.vector3_to_string(self.minBorders)}; maxBord={common_tools.vector3_to_string(self.maxBorders)};'
            f' rMinBord={common_tools.vector3_to_string(self.refMinBorders)}; rMaxBord={common_tools.vector3_to_string(self.refMaxBorders)};'
            f' length={len(self.points)};)'
        )
    
    def __repr__(self):
        return str(self)
    
    def __eq__(self, other):
        return self.minBorders == other.minBorders and self.maxBorders == other.maxBorders \
            and self.refMinBorders == other.refMinBorders and self.refMaxBorders == other.refMaxBorders \
            and self.centers == other.centers and self.nodeHierarchy == other.nodeHierarchy \
            and len(self.childSectors) == len(other.childSectors) and len(self.points) == len(other.points)


# Quadtree with one or more root nodes.
class sectors_data:
    def __init__(self, minBorders:list, maxBorders:list, createVariants:bool=False, variantsOffset:float=0.33):
        self.rootNodes:list[points_sector] = []
        self.rootNodes.append( points_sector(minBorders, maxBorders) )
        self.numberOfPoints = 0
        if (createVariants):
            self.rootNodes.append( points_sector(minBorders=[minBorders[0]-variantsOffset, minBorders[1]-variantsOffset, minBorders[2]],
                                                 maxBorders=[maxBorders[0]-variantsOffset, maxBorders[1]-variantsOffset, maxBorders[2]],
                                                 nodeHierarchy=[1]) )
            self.rootNodes.append( points_sector(minBorders=[minBorders[0]+variantsOffset, minBorders[1]-variantsOffset, minBorders[2]],
                                                 maxBorders=[maxBorders[0]+variantsOffset, maxBorders[1]-variantsOffset, maxBorders[2]],
                                                 nodeHierarchy=[2]) )
            self.rootNodes.append( points_sector(minBorders=[minBorders[0]-variantsOffset, minBorders[1]+variantsOffset, minBorders[2]],
                                                 maxBorders=[maxBorders[0]-variantsOffset, maxBorders[1]+variantsOffset, maxBorders[2]],
                                                 nodeHierarchy=[3]) )
            self.rootNodes.append( points_sector(minBorders=[minBorders[0]+variantsOffset, minBorders[1]+variantsOffset, minBorders[2]],
                                                 maxBorders=[maxBorders[0]+variantsOffset, maxBorders[1]+variantsOffset, maxBorders[2]],
                                                 nodeHierarchy=[4]) )

    def add_point(self, point:list):
        for rootNode in self.rootNodes:
            rootNode.add_point(point)
        self.numberOfPoints += 1
    
    def add_all_points(self, points:list):
        for rootNode in self.rootNodes:
            rootNode.add_all_points(points)
        self.numberOfPoints += len(points)

    def remove_all_points(self):
        for rootNode in self.rootNodes:
            rootNode.clear_list()
            rootNode.remove_child_sectors()
        self.numberOfPoints = 0
    
    def sectorize_data(self, maxPointCount:int=6144, optimalPointCount:int=2048):
        for rootNode in self.rootNodes:
            rootNode.divide(maxPointCount)
        if (optimalPointCount >= 0):
            self._merge_smaller_children(optimalPointCount)
    
    def _merge_smaller_children(self, optimalPointCount:int=2048):
        print(f'Merging started (optimalPointCount={optimalPointCount}).')
        nodes:list[points_sector] = []
        for rootNode in self.rootNodes:
            rootNode.collect_nodes(nodes, True)
        #nodes.sort(key=lambda x: len(x.childSectors), reverse=True) # leaf nodes first
        #print(f'nodes = {nodes}')
        merger = sector_merger()
        for i in range(0, len(nodes)):
            #print(f'node {nodes[i].nodeHierarchy} has {len(nodes[i].childSectors)} children.')
            #print(f'node {nodes[i].nodeHierarchy} has {nodes[i].descendantPointsNumber} descendant points.')
            #if (nodes[i].descendantPointsNumber > maxPointCount):
            #    print(f'WARNING: node {nodes[i].nodeHierarchy} has {nodes[i].descendantPointsNumber} descendant points; skipping merge attempt.')
            newChildren:list[points_sector] = merger.check_and_merge(nodes[i], optimalPointCount)
            if (newChildren is not None):
                nodes[i].childSectors = newChildren
        print(f'Merging finished.')
    
    def upscale_sectors(self, margin:float=0.05):
        # Neural networks usually work best with values in the range of <-1,1>.
        # When the margin param is > 0, the network will get additional space near the boundaries.
        # This should allow for the generation of new points outside the actual sector boundaries.
        tempMinBorders = [-1.0+margin, -1.0+margin, -1.0+margin]
        tempMaxBorders = [1.0-margin, 1.0-margin, 1.0-margin]
        for rootNode in self.rootNodes:
            rootNode.recalculate_coords(tempMinBorders.copy(), tempMaxBorders.copy(), True)
    
    def downscale_sectors(self):
        for rootNode in self.rootNodes:
            rootNode.recalculate_back(True)
    
    def sample_points(self, targetPointCount:int=2048, randomRadius:float=0.001, useFps:bool=True, minPointsForSampling:int=1536):
        for rootNode in self.rootNodes:
            rootNode.sample_points(targetPointCount, randomRadius, useFps, minPointsForSampling)
    
    def to_point_cloud(self) -> list:
        allPoints = []
        for rootNode in self.rootNodes:
            rootNode.copy_points(allPoints)
        return allPoints
    
    def save_to_disk(self, outputDir, prefix, extension:str='.obj'):
        nodes:list[points_sector] = []
        for rootNode in self.rootNodes:
            rootNode.collect_nodes(nodes, False)
        self._save_nodes_to_disk(nodes, outputDir, prefix, extension)
    
    def _save_nodes_to_disk(self, nodes, outputDir, prefix, extension:str='.obj'):
        Path(outputDir).mkdir(parents=True, exist_ok=True)
        infoPath = os.path.join(outputDir, prefix+'_info.txt')
        f = open(infoPath, 'w')
        for node in nodes:
            self._write_node_to_disk(outputDir, prefix, extension, f, node)
        f.close()

    def _write_node_to_disk(self, outputDir, prefix, extension, f, node):
        try:
            f.write(f'{str(node)}\n')
            tempPath = os.path.join(outputDir, prefix + node.hierarchy_to_string() + extension)
            print(f'Saving {tempPath}')
            pcd_io.coords_to_file(node.points, tempPath)
        except Exception as e:
            print(f'[{prefix}] could not save node: {str(node)} (type:{type(node)})')
            print(traceback.format_exc())
            print(e)
    
    def _read_sector(self, inputDir, prefix, isScaled, extension, line, readPoints:bool=True):
        points:list = []
        minBorders:list = []
        maxBorders:list = []
        refMinBorders:list = []
        refMaxBorders:list = []
        nodeHierarchy:list=[0]

        #print(line.strip())
        nodeHierarchyStr, nodeInfoStr = line.strip().split(':')
        pointsPath = os.path.join(inputDir, prefix+nodeHierarchyStr+extension)
        #print(f'Reading sector: {prefix+nodeHierarchyStr+extension}')

        nodeHierarchyStr = nodeHierarchyStr.split('.')
        nodeHierarchy = [int(i) for i in nodeHierarchyStr]
        nodeInfoStr = nodeInfoStr.replace('(', '')
        nodeInfoStr = nodeInfoStr.replace(')', '')
        nodeInfoStr = nodeInfoStr.replace(' ', '')
        nodeFields = nodeInfoStr.split(';')
        splitNodeFields = [i.split('=') for i in nodeFields]
        #print('splitNodeFields', str(splitNodeFields))
        for field in splitNodeFields:
            if field[0].startswith('minBord'):
                minBorders = common_tools.parse_vector3(field[1])
            elif field[0].startswith('maxBord'):
                maxBorders = common_tools.parse_vector3(field[1])
            elif field[0].startswith('rMinBord'):
                refMinBorders = common_tools.parse_vector3(field[1])
            elif field[0].startswith('rMaxBord'):
                refMaxBorders = common_tools.parse_vector3(field[1])
        node = points_sector(minBorders, maxBorders, refMinBorders, refMaxBorders, None, nodeHierarchy, isScaled)
        if (readPoints):
            points = common_tools.read_point_cloud(pointsPath)
            node.add_all_points(points)
        return node

    def read_from_disk(self, inputDir, prefix, isScaled:bool=True, extension='.obj', immediateRescale:bool=False, returnPoints:bool=False,
                       oldRecalculation:bool=False):
        optionalResult:list = []
        infoPath = os.path.join(inputDir, prefix+'_info.txt')
        try:
            # Create the main root node and 4 variant roots used for different offsets.
            allNodes:list[points_sector] = []
            for i in range(0, 5):
                tempRootNodes:list[points_sector] = []
                allNodes.append(tempRootNodes)
            with open(infoPath, 'r') as file:
                for line in file:
                    node = self._read_sector(inputDir, prefix, isScaled, extension, line)
                    #print(f'nodeHierarchy: {node.nodeHierarchy}')
                    if (immediateRescale):
                        node.recalculate_back(True, oldRecalculation)
                    if (returnPoints):
                        node.copy_points(optionalResult)
                    currentRoot = int(node.nodeHierarchy[0])
                    if (currentRoot < len(allNodes)):
                        allNodes[currentRoot].append(node)
                    else:
                        print(f'WARNING: skipping node with root={currentRoot}, since expected max is {len(allNodes)-1}.')
            self.remove_all_points()
            for i in range(0, len(allNodes)):
                rootMinBorders = [sys.float_info.max, sys.float_info.max, sys.float_info.max]
                rootMaxBorders = [-sys.float_info.max, -sys.float_info.max, -sys.float_info.max]
                rootRefMinBorders = [sys.float_info.max, sys.float_info.max, sys.float_info.max]
                rootRefMaxBorders = [-sys.float_info.max, -sys.float_info.max, -sys.float_info.max]
                node:points_sector
                for node in allNodes[i]:
                    #node.recalculate_back(True)
                    rootMinBorders = min(rootMinBorders, node.minBorders)
                    rootMaxBorders = max(rootMaxBorders, node.maxBorders)
                    rootRefMinBorders = min(rootRefMinBorders, node.refMinBorders)
                    rootRefMaxBorders = max(rootRefMaxBorders, node.refMaxBorders)
                if (i < len(self.rootNodes)):
                    self.rootNodes[i] = points_sector(rootMinBorders, rootMaxBorders, rootRefMinBorders, rootRefMaxBorders)
                    self.rootNodes[i].childSectors.clear()
                    self.rootNodes[i].build_from_list(allNodes[i])
                    if (len(allNodes[i]) > 0):
                        print(f'WARNING: could not include {len(allNodes[i])} nodes ({prefix}[{i}])')
                        #print(f'WARNING: could not include the following nodes: {allNodes}')
                else:
                    print(f'WARNING: skipping root node {i} since the sectors data were initialized without variants.')
        except Exception as e:
            print(traceback.format_exc())
            print(e)
        return optionalResult
    
    def resectorize(self, inputFile, outputDir, isScaled:bool=True, extension='.obj', oldRecalculation:bool=False):
        tempName = Path(inputFile).name
        suffixIndex = tempName.rindex('_info.txt')
        prefix = tempName[:suffixIndex]
        inputDir = Path(inputFile).parent
        try:
            Path(outputDir).mkdir(parents=True, exist_ok=True)
            outInfoPath = os.path.join(outputDir, prefix+'_info.txt')
            outFile = open(outInfoPath, 'w')
            with open(inputFile, 'r') as inFile:
                for line in inFile:
                    node = self._read_sector(inputDir, prefix, isScaled, extension, line)
                    node.recalculate_back(True, oldRecalculation)
                    self._write_node_to_disk(outputDir, prefix, extension, outFile, node)
            outFile.close()
        except Exception as e:
            print(traceback.format_exc())
            print(e)

    def replace_points(self, externalCloud:list, maxPointCount:int):
        for rootNode in self.rootNodes:
            nodes:list = []
            rootNode.collect_nodes(nodes, False)
            for node in nodes:
                node.replace_points_from_cloud(externalCloud, maxPointCount)

    def __str__(self):
        result = '('
        for i in range(0, len(self.rootNodes)):
            nodes:list = []
            self.rootNodes[i].collect_nodes(nodes, False)
            result += f'minBord={common_tools.vector3_to_string(self.rootNodes[i].minBorders)}; maxBord={common_tools.vector3_to_string(self.rootNodes[i].maxBorders)};'
            f' length={self.numberOfPoints};'
            f' nodes={str(nodes)};'
        result += ')'
        return result
    
    def __repr__(self):
        return str(self)


class sector_merger:
    def check_and_merge(self, node:points_sector, optimalPointCount:int=2048) -> list:
        if (len(node.childSectors) < 4): # no children or already merged
            return None
        
        sectorSizes:list[int] = [len(node.childSectors[0].points), len(node.childSectors[1].points),
                                 len(node.childSectors[2].points), len(node.childSectors[3].points)]
        mergables:list = [bool]
        mergablesCount:int = 0
        for i in range(0, len(sectorSizes)):
            # If a sector is empty or is a parent to any other sectors, then it is not suitable for merging.
            if self._isMergable(node.childSectors[i]):
                mergables.append(False)
            else:
                mergables.append(True)
                mergablesCount += 1

        if (mergablesCount <= 1):
            return None # not enough sectors for merging
        
        elif (mergablesCount == 2):
            if ( (mergables[0] and mergables[3]) or (mergables[1] and mergables[2]) ):
                 return None # diagonal merging is not supported
            else:
                return self._attemptMerging(node, sectorSizes, mergablesCount, optimalPointCount)
        else:
            return self._attemptMerging(node, sectorSizes, mergablesCount, optimalPointCount)
        
    def _attemptMerging(self, node:points_sector, sectorSizes:list, mergablesCount:int, optimalPointCount:int=2048) -> list:
        smallestSectorIndex:int = self._findSmallestMergableSector(node, sectorSizes)
        if (smallestSectorIndex < 0): # sectors are too large to merge
            return None
        
        neighbourType = self._getBetterNeighbour(node, sectorSizes, smallestSectorIndex, optimalPointCount)
        if (neighbourType == 0): # neighbours have too many points or descendants
            return None
        
        elif (neighbourType == 1): # horizontal neighbour
            newChildSectors:list = []
            if (smallestSectorIndex <= 1):
                print(f'Merging top sectors for node {node.nodeHierarchy} ({sectorSizes[0]}+{sectorSizes[1]} points).')
                newChildSectors.append( node.childSectors[0]._merge_inline(node.childSectors[1], node.nodeHierarchy+[0]) )
                tempScore = abs(sectorSizes[2] + sectorSizes[3] - optimalPointCount)
                if (mergablesCount > 3 and self._isMergable(sectorSizes[2]) and tempScore < abs(sectorSizes[2] - optimalPointCount)
                    and self._isMergable(sectorSizes[3]) and tempScore < abs(sectorSizes[3] - optimalPointCount)):
                        print(f'also merging bottom sectors for node {node.nodeHierarchy}'
                              + f'({sectorSizes[0]}+{sectorSizes[1]})({sectorSizes[2]}+{sectorSizes[3]} points).')
                        newChildSectors.append( node.childSectors[2]._merge_inline(node.childSectors[3], node.nodeHierarchy+[1]) )
                else:
                    self._reassign_children(newChildSectors, node, node.childSectors[2], node.childSectors[3])
            else:
                print(f'Merging bottom sectors for node {node.nodeHierarchy} ({sectorSizes[2]}+{sectorSizes[3]} points).')
                newChildSectors.append( node.childSectors[2]._merge_inline(node.childSectors[3], node.nodeHierarchy+[0]) )
                tempScore = abs(sectorSizes[0] + sectorSizes[1] - optimalPointCount)
                if (mergablesCount > 3 and self._isMergable(sectorSizes[0]) and tempScore < abs(sectorSizes[0] - optimalPointCount)
                    and self._isMergable(sectorSizes[1]) and tempScore < abs(sectorSizes[1] - optimalPointCount)):
                        print(f'also merging top sectors for node {node.nodeHierarchy}'
                              + f'({sectorSizes[2]}+{sectorSizes[3]})({sectorSizes[0]}+{sectorSizes[1]} points).')
                        newChildSectors.append( node.childSectors[0]._merge_inline(node.childSectors[1], node.nodeHierarchy+[1]) )
                else:
                    self._reassign_children(newChildSectors, node, node.childSectors[0], node.childSectors[1])
            return newChildSectors
        
        elif (neighbourType == 2): # verticalNeighbour
            newChildSectors:list = []
            if (smallestSectorIndex == 0 or smallestSectorIndex == 2):
                print(f'Merging left sectors for node {node.nodeHierarchy} ({sectorSizes[0]}+{sectorSizes[2]} points).')
                newChildSectors.append( node.childSectors[0]._merge_inline(node.childSectors[2], node.nodeHierarchy+[0]) )
                tempScore = abs(sectorSizes[1] + sectorSizes[3] - optimalPointCount)
                if (mergablesCount > 3 and self._isMergable(sectorSizes[1]) and tempScore < abs(sectorSizes[1] - optimalPointCount)
                    and self._isMergable(sectorSizes[3]) and tempScore < abs(sectorSizes[3] - optimalPointCount)):
                        print(f'also merging right sectors for node {node.nodeHierarchy}'
                              + f'({sectorSizes[0]}+{sectorSizes[2]})({sectorSizes[1]}+{sectorSizes[3]} points).')
                        newChildSectors.append( node.childSectors[1]._merge_inline(node.childSectors[3], node.nodeHierarchy+[1]) )
                else:
                    self._reassign_children(newChildSectors, node, node.childSectors[1], node.childSectors[3])
            else: # merge right sectors vertically
                print(f'Merging right sectors for node {node.nodeHierarchy} ({sectorSizes[1]}+{sectorSizes[3]} points).')
                newChildSectors.append( node.childSectors[1]._merge_inline(node.childSectors[3], node.nodeHierarchy+[0]) )
                tempScore = abs(sectorSizes[0] + sectorSizes[2] - optimalPointCount)
                if (mergablesCount > 3 and self._isMergable(sectorSizes[0]) and tempScore < abs(sectorSizes[0] - optimalPointCount)
                    and self._isMergable(sectorSizes[2]) and tempScore < abs(sectorSizes[2] - optimalPointCount)):
                        print(f'also merging left sectors for node {node.nodeHierarchy}'
                              + f'({sectorSizes[1]}+{sectorSizes[3]})({sectorSizes[0]}+{sectorSizes[2]} points).')
                        newChildSectors.append( node.childSectors[0]._merge_inline(node.childSectors[2], node.nodeHierarchy+[1]) )
                else:
                    self._reassign_children(newChildSectors, node, node.childSectors[0], node.childSectors[2])
            return newChildSectors
        
        else:
            print(f'WARNING: unknown neighbour type: {neighbourType}')
            return None

    def _getBetterNeighbour(self, node:points_sector, sectorSizes:list, smallestSectorIndex:int, optimalPointCount:int=2048) -> int:
        #bestIndex:int = smallestSectorIndex
        smallestSectorScore:int = abs(sectorSizes[smallestSectorIndex] - optimalPointCount)
        bestScore:int = smallestSectorScore
        neighbourType = 0

        horizontalNeighbourIndex = self._getHorizontalNeighbour(smallestSectorIndex)
        if (horizontalNeighbourIndex >= 0 and self._isMergable(node.childSectors[horizontalNeighbourIndex])):
            tempScore:int = abs(sectorSizes[smallestSectorIndex] + sectorSizes[horizontalNeighbourIndex] - optimalPointCount)
            if (tempScore < bestScore):
                bestScore = tempScore
                #bestIndex = horizontalNeighbourIndex
                neighbourType = 1

        verticalNeighbourIndex = self._getVerticalNeighbour(smallestSectorIndex)
        if (verticalNeighbourIndex >= 0 and self._isMergable(node.childSectors[verticalNeighbourIndex])):
            tempScore:int = abs(sectorSizes[smallestSectorIndex] + sectorSizes[verticalNeighbourIndex] - optimalPointCount)
            if (tempScore < bestScore):
                bestScore = tempScore
                #bestIndex = verticalNeighbourIndex
                neighbourType = 2
        
        return neighbourType
    
    def _findSmallestMergableSector(self, node:points_sector, sectorSizes:list, optimalPointCount:int=2048):
        result:int = -1
        minValue:int = sys.maxsize
        for i in range(0, len(sectorSizes)):
            if (self._isMergable(node.childSectors[i]) and (sectorSizes[i] < minValue)):
                minValue = sectorSizes[i]
                result = i
        if (minValue >= optimalPointCount): # sectors are too large to merge
            return -1
        return result
    
    def _isMergable(self, sector:points_sector):
        return not (len(sector.points) <= 0 or len(sector.childSectors) > 0 or sector.descendantPointsNumber > 0)

    def _getHorizontalNeighbour(self, currentIndex) -> int:
        if (currentIndex == 0):
            return 1
        elif (currentIndex == 1):
            return 0
        elif (currentIndex == 2):
            return 3
        elif (currentIndex == 3):
            return 2
        else:
            return -1
    
    def _getVerticalNeighbour(self, currentIndex) -> int:
        if (currentIndex == 0):
            return 2
        elif (currentIndex == 2):
            return 0
        elif (currentIndex == 1):
            return 3
        elif (currentIndex == 3):
            return 1
        else:
            return -1
    
    def _reassign_children(self, newChildSectors:list, node, sector1:points_sector, sector2:points_sector):
        sector1.nodeHierarchy = node.nodeHierarchy+[1]
        sector2.nodeHierarchy = node.nodeHierarchy+[2]
        newChildSectors.append(sector1)
        newChildSectors.append(sector2)

def sectorize(inFile, outDir, prefix, maxPointCount:int=6144, optimalPointCount:int=2048, rescale:bool=True, extension:str='.obj',
              createVariants:bool=False, variantsOffset:float=0.33, minPointsForSampling:int=1536) -> int:
    #print(f'Sectorizing {Path(inFile).name}.')
    inCoords = common_tools.read_point_cloud(inFile)
    if (len(inCoords) < maxPointCount):
        print(f' File {Path(inFile).name} has too few points to sectorize ({len(inCoords)} < {maxPointCount}).')
        return 1
    else:
        sectors = sectors_data([-1.0,-1.0,-1.0], [1.0,1.0,1.0], createVariants, variantsOffset)
        sectors.add_all_points(inCoords)
        sectors.sectorize_data(maxPointCount, optimalPointCount)
        sectors.sample_points(targetPointCount=optimalPointCount, minPointsForSampling=minPointsForSampling)
        if (rescale):
            sectors.upscale_sectors()
        sectors.save_to_disk(outDir, prefix, extension)
        return 0

def resectorize(inFile, outDir, isScaled:bool=True, extension:str='.obj'):
    sectors = sectors_data([-1.0,-1.0,-1.0], [1.0,1.0,1.0])
    sectors.resectorize(inFile, outDir, isScaled, extension)

def combine_sectors(inDir, outFile, prefix, extension, rescale:bool=True, returnPointsDirectly:bool=True,
                    createVariants:bool=False, variantsOffset:float=0.33, oldRecalculation:bool=False,
                    removeDuplicates:bool=True):
    sectors = sectors_data([-1.0,-1.0,-1.0], [1.0,1.0,1.0], createVariants, variantsOffset)
    restoredPoints = sectors.read_from_disk(inDir, prefix, rescale, extension, rescale, returnPointsDirectly, oldRecalculation)
    if (removeDuplicates):
        print(f'DEBUG: removing duplicate points.')
        restoredPoints = common_tools.downsample_points(restoredPoints)
    print(f'Saving {Path(outFile).name}')
    pcd_io.coords_to_file(restoredPoints, outFile)

def transfer_sectors(inDir, inPrefix, pointsFile, outDir, outPrefix, maxPointCount:int, isScaled:bool=False, extension='.obj',
                     createVariants:bool=False, variantsOffset:float=0.33, oldRecalculation:bool=False):
    sectors = sectors_data([-1.0,-1.0,-1.0], [1.0,1.0,1.0], createVariants, variantsOffset)
    infoPath = os.path.join(inDir, inPrefix+'_info.txt')
    try:
        inCoords = common_tools.read_point_cloud(pointsFile)
        allNodes:list = []
        with open(infoPath, 'r') as file:
            for line in file:
                node = sectors._read_sector(inDir, inPrefix, False, extension, line)
                node.replace_points_from_cloud(inCoords, maxPointCount, isScaled)
                if (isScaled):
                    node.swap_borders_info()
                    node.recalculate_coords(node.refMinBorders, node.refMaxBorders, False, oldRecalculation)
                    node.swap_borders_info()
                allNodes.append(node)
        sectors._save_nodes_to_disk(allNodes, outDir, outPrefix, extension)
    except Exception as e:
        print(f'Could not transfer some sectors for {outPrefix}.')
        print(traceback.format_exc())
        print(e)

def _extract_dots_area(filename:str, isDirectory:bool=False) -> str:
    startIndex = filename.rindex("_") + 1
    if (not isDirectory):
        endIndex = filename.rindex(".")
    else:
        endIndex = len(filename)-1
    substring = filename[startIndex:endIndex]
    return substring

def _replace_last_occurrence(filename:str, substring:str, replacement:str):
    index = filename.rfind(substring)
    return filename[:index] + replacement + filename[index+len(substring):]

def remove_dots(filename:str, isDirectory:bool=False):
    try:
        substring = _extract_dots_area(filename, isDirectory)
        noDots = substring.replace(".", "")
        return _replace_last_occurrence(filename, substring, noDots)
    except Exception as e:
        #print(traceback.format_exc())
        #print(e)
        return filename

def restore_dots(filename:str, isDirectory:bool=False):
    try:
        substring = _extract_dots_area(filename, isDirectory)
        restoredDots = ''
        for i in range(0,len(substring)-1):
            restoredDots += substring[i] + '.'
        restoredDots += substring[len(substring)-1]
        return _replace_last_occurrence(filename, substring, restoredDots)
    except Exception as e:
        #print(traceback.format_exc())
        #print(e)
        return filename

def fix_dots_in_filenames(dirPath, removeMode:bool, inputExtension:str='.*', isDirectory:bool=False):
    if (os.path.isdir(dirPath)):
        for filename in os.listdir(dirPath):
            if (inputExtension == '.*' or filename.lower().endswith(inputExtension)):
                if (removeMode):
                    newFilename = remove_dots(filename, isDirectory)
                else:
                    newFilename = restore_dots(filename, isDirectory)
                if (newFilename != filename):
                    try:
                        inputFullPath = os.path.join(dirPath, filename)
                        outputFullPath = os.path.join(dirPath, newFilename)
                        os.rename(inputFullPath, outputFullPath)
                    except Exception as e:
                        print(f'Could not rename file: {filename}')
                        #print(traceback.format_exc())
                        #print(e)

def _sf_preprocess_file(partialFile, outDir, completeFile=None, optimalPointCount:int=2048, maxSectPoints:int=6144,
                        numberPrefix='00010000-', createVariants:bool=False, variantsOffset:float=0.33,
                        oldRecalculation:bool=False, maxRefSectPoints=8192, minPointsForSampling:int=1536,
                        roof_thinning_ratio:float=-1.0):
    lastDotIndex = Path(partialFile).name.rindex('.')
    sectPrefix = Path(partialFile).name[:lastDotIndex] + '_'
    destFilePrefix = ''
    if (numberPrefix != '' and not sectPrefix.startswith(numberPrefix)):
        sectPrefix = numberPrefix + sectPrefix
        destFilePrefix = numberPrefix
    dataObjDir = os.path.join(outDir, 'data_obj')
    partialSectorsDir = os.path.join(dataObjDir, 'partial_sectors')
    #partialSectorsUnscaledDir = os.path.join(dataObjDir, 'partial_sectors_unscaled')
    partialSinglesDir = os.path.join(dataObjDir, 'partial_singles')
    completeSectorsDir = os.path.join(dataObjDir, 'complete_sectors')
    completeSinglesDir = os.path.join(dataObjDir, 'complete_singles')

    if (roof_thinning_ratio >= 0 and roof_thinning_ratio <= 1):
        thinnedRoofsDir = os.path.join(outDir, 'thinned_roofs')
        inCoords = common_tools.read_point_cloud(partialFile)
        _, walls = distance.select_sparse_roofs(inCoords, 0.5, 1.0-roof_thinning_ratio)
        partialFile = os.path.join(thinnedRoofsDir, Path(partialFile).name)
        pcd_io.coords_to_file(walls, partialFile)

    sectResult = sectorize(partialFile, partialSectorsDir, sectPrefix, maxSectPoints, optimalPointCount, True, '.obj',
                           createVariants, variantsOffset, minPointsForSampling)
    #if (sectResult == 0):
    #    # Point cloud was sectorized, also create an unscaled version.
    #    sectorize(partialFile, partialSectorsUnscaledDir, sectPrefix, maxSectPoints, optimalPointCount, False, '.obj',
    #              createVariants, variantsOffset, minPointsForSampling)
    #else:
    if (sectResult != 0):
        # Point cloud was not sectorized and will be treated as a standalone "single" file.
        partialSingleFile = os.path.join(partialSinglesDir, destFilePrefix+Path(partialFile).name)
        os.makedirs(os.path.dirname(partialSingleFile), exist_ok=True)
        shutil.copy(partialFile, partialSingleFile)
        inCoords = common_tools.read_point_cloud(partialSingleFile)
        # Upsample the single partial point cloud if it is too small.
        if (len(inCoords) < optimalPointCount):
            inCoords = common_tools.sample_data(pointCloud=inCoords, minPointCount=optimalPointCount, maxPointCount=optimalPointCount,
                                                randomRadius=0.001)
            pcd_io.coords_to_file(inCoords, partialSingleFile)
    
    if (completeFile is not None and os.path.isfile(completeFile)):
        completeSingleFile = os.path.join(completeSinglesDir, destFilePrefix+Path(completeFile).name)
        if (sectResult != 0):
            # Create a single, non-sectorized reference point cloud.
            #os.makedirs(os.path.dirname(completeSingleFile), exist_ok=True)
            #shutil.copy(completeFile, completeSingleFile)
            pcd_io.sample_file(inFile=completeFile, outFile=completeSingleFile, minPointCount=maxRefSectPoints,
                               maxPointCount=maxRefSectPoints, randomRadius=0.001, useFps=False)
        else:
            # Create a sectorized reference point cloud.
            transfer_sectors(partialSectorsDir, sectPrefix, completeFile, completeSectorsDir, sectPrefix, maxRefSectPoints, True, '.obj',
                             createVariants, variantsOffset, oldRecalculation)

def _sf_convert_partial_pcn(partialSectorsDir, partialPcdDir):
    for path in os.listdir(partialSectorsDir):
        pathWithoutExt = common_tools.path_without_extension(path)
        srcPath = os.path.join(partialSectorsDir, path)
        dstPath = os.path.join(partialPcdDir, pathWithoutExt, '00.pcd')
        pcd_io.convert(srcPath, dstPath)

def _sf_preprocess_finalize(outDir, pcn:bool=False, optimalPointCount:int=2048):
    dataObjDir = os.path.join(outDir, 'data_obj')
    partialSectorsDir = os.path.join(dataObjDir, 'partial_sectors')
    partialSinglesDir = os.path.join(dataObjDir, 'partial_singles')
    completeSectorsDir = os.path.join(dataObjDir, 'complete_sectors')
    completeSinglesDir = os.path.join(dataObjDir, 'complete_singles')
    os.makedirs(partialSinglesDir, exist_ok=True)
    os.makedirs(completeSinglesDir, exist_ok=True)
    taxonomyId = '04460130'
    taxonomyName = 'tower'

    # If the reference file does not exist, use a duplicated input file in its place.
    for path in os.listdir(partialSectorsDir):
        srcPath = os.path.join(partialSectorsDir, path)
        dstPath = os.path.join(completeSectorsDir, path)
        if not os.path.exists(dstPath):
            os.makedirs(os.path.dirname(dstPath), exist_ok=True)
            shutil.copy(srcPath, dstPath)
    for path in os.listdir(partialSinglesDir):
        srcPath = os.path.join(partialSinglesDir, path)
        dstPath = os.path.join(completeSinglesDir, path)
        if not os.path.exists(dstPath):
            os.makedirs(os.path.dirname(dstPath), exist_ok=True)
            shutil.copy(srcPath, dstPath)
    #common_tools.copytree(partialSectorsDir, completeSectorsDir)
    #common_tools.copytree(partialSinglesDir, completeSinglesDir)
    
    # Prepare the "partial" and "complete" data dirs which will be used as input for SeedFormer.
    if (not pcn):
        dataNpyDir = os.path.join(outDir, 'data_npy')
        partialNpyDir = os.path.join(dataNpyDir, 'partial')
        completeNpyDir = os.path.join(dataNpyDir, 'complete')
        pcd_io.batch_convert(partialSectorsDir, partialNpyDir, '.npy', '.obj')
        pcd_io.batch_convert(completeSectorsDir, completeNpyDir, '.npy', '.obj')
        fix_dots_in_filenames(partialNpyDir, True, '.npy')
        fix_dots_in_filenames(completeNpyDir, True, '.npy')

        pcd_io.batch_convert(partialSinglesDir, partialNpyDir, '.npy', '.obj')
        pcd_io.batch_convert(completeSinglesDir, completeNpyDir, '.npy', '.obj')

        trainPath = os.path.join(dataNpyDir, 'train.txt')
        f = open(trainPath, 'w')
        f.close()

        testPath = os.path.join(dataNpyDir, 'test.txt')
        testPathFull = os.path.join(dataNpyDir, '_test_full.txt')
        f = open(testPath, 'w')
        f_full = open(testPathFull, 'w')
        for path in os.listdir(partialNpyDir):
            fullPath = os.path.join(partialNpyDir, path)
            if os.path.isfile(fullPath) and (fullPath.lower().endswith('.npy')):
                f_full.write(f'{path}\n')
                tempCoords = common_tools.read_numpy(fullPath)
                if (len(tempCoords) >= optimalPointCount):
                    f.write(f'{path}\n')
        f.close()
        f_full.close()
    else:
        dataPcdDir = os.path.join(outDir, 'data_pcd')
        completePcdDir = os.path.join(dataPcdDir, 'test', 'complete', taxonomyId)
        partialPcdDir = os.path.join(dataPcdDir, 'test', 'partial', taxonomyId)
        pcd_io.batch_convert(completeSectorsDir, completePcdDir, '.pcd', '.obj')
        fix_dots_in_filenames(completePcdDir, True, '.pcd')
        #_sf_convert_partial_pcn(partialSectorsDir, partialPcdDir)
        pcd_io.batch_convert(partialSectorsDir, partialPcdDir, '.pcd', '.obj')
        #fix_dots_in_filenames(dirPath=partialPcdDir, removeMode=True, isDirectory=True)
        fix_dots_in_filenames(dirPath=partialPcdDir, removeMode=True)

        pcd_io.batch_convert(completeSinglesDir, completePcdDir, '.pcd', '.obj')
        #_sf_convert_partial_pcn(partialSinglesDir, partialPcdDir)
        pcd_io.batch_convert(partialSinglesDir, partialPcdDir, '.pcd', '.obj')

        categoryFile = os.path.join(dataPcdDir, 'category.txt')
        f = open(categoryFile, 'w')
        f.write(f'{taxonomyName}\n')
        f.close()
        jsonFile = os.path.join(dataPcdDir, 'ShapeNet.json')
        pcd_io.create_pcd_json(jsonFile)

def sf_preprocess(inDirPartial, outDir, inDirComplete=None, optimalPointCount:int=2048, maxSectPoints:int=6144, numberPrefix='00010000-',
                  createVariants:bool=False, variantsOffset:float=0.33, oldRecalculation:bool=False, pcn:bool=False, minPointsForSampling:int=1536,
                  roof_thinning_ratio:float=-1.0):
    for path in os.listdir(inDirPartial):
        partialPath = os.path.join(inDirPartial, path)
        if (os.path.isfile(partialPath)):
            if (inDirComplete is None):
                completePath = None
            else:
                completePath = os.path.join(inDirComplete, path)
            #print(f'partialPath: {partialPath}')
            #print(f'completePath: {completePath}')
            maxRefSectPoints:int = 8192
            if (pcn):
                maxRefSectPoints *= 2
            _sf_preprocess_file(partialPath, outDir, completePath, optimalPointCount, maxSectPoints, numberPrefix,
                                createVariants, variantsOffset, oldRecalculation, maxRefSectPoints, minPointsForSampling,
                                roof_thinning_ratio)
    _sf_preprocess_finalize(outDir, pcn)

def sf_preprocess_preset(inDirPartial, outDir, presetPath, inDirComplete=None, variantsOffset:float=0.33, oldRecalculation:bool=False,
                         minPointsForSampling:int=1536, roof_thinning_ratio:float=-1.0):
    jsonFile = open(presetPath, 'r')
    preset = json.load(jsonFile)
    pcn:bool = False
    pccType = preset["pccConfig"]["type"]
    if ("former" in pccType[0].lower() and "pcn" in pccType[1].lower()):
        pcn = True
    sf_preprocess(inDirPartial=inDirPartial, outDir=outDir, inDirComplete=inDirComplete, optimalPointCount=preset["preprocessing"]["optimal_points"],
                  maxSectPoints=preset["preprocessing"]["max_points"], numberPrefix=preset["prefix"], createVariants=preset["create_variants"],
                  variantsOffset=variantsOffset, oldRecalculation=oldRecalculation, pcn=pcn, minPointsForSampling=minPointsForSampling,
                  roof_thinning_ratio=roof_thinning_ratio)
    jsonFile.close()

def sf_inference(preprocessDir, presetPath):
    jsonFile = open(presetPath, 'r')
    preset = json.load(jsonFile)
    pccType = preset["pccConfig"]["type"]
    pccRootDir = preset["pccConfig"]["fullRootDir"]
    pccRunDir = Path(pccRootDir, preset["pccConfig"]["runDir"])
    inferenceCommand = preset["pccConfig"]["inferenceCommand"].split()
    pccPartialDir = Path(pccRootDir, preset["pccConfig"]["partialDir"])

    pccCompleteDir = None
    if ("completeDir" in preset["pccConfig"]):
        pccCompleteDir = Path(pccRootDir, preset["pccConfig"]["completeDir"])
    datasetConfigDir = None
    if ("datasetConfigDir" in preset["pccConfig"]):
        datasetConfigDir = Path(pccRootDir, preset["pccConfig"]["datasetConfigDir"])
    
    outputsDir = Path(pccRootDir, preset["pccConfig"]["outputsDir"])
    predictedDir = Path(preprocessDir, preset["copiedOutputsDir"])
    jsonFile.close()

    # Clear existing input and output files.
    directoriesToClear = preset["pccConfig"]["directoriesToClear"]
    for directory in directoriesToClear:
        fullDirPath = Path(pccRootDir, directory)
        common_tools.clear_directory(fullDirPath)
    
    # Copy the input files.
    if ("former" in pccType[0].lower() and "pcn" in pccType[1].lower()):
        preprocPartialDir = Path(preprocessDir, "data_pcd", "test", "partial")
        preprocCompleteDir = Path(preprocessDir, "data_pcd", "test", "complete")
    else:
        preprocPartialDir = Path(preprocessDir, "data_npy", "partial")
        preprocCompleteDir = Path(preprocessDir, "data_npy", "complete")
    shutil.copytree(preprocPartialDir, pccPartialDir, dirs_exist_ok=True)
    if (pccCompleteDir is not None):
        shutil.copytree(preprocCompleteDir, pccCompleteDir, dirs_exist_ok=True)
    if (datasetConfigDir is not None and os.path.isdir(datasetConfigDir)):
        if ("former" in pccType[0].lower() and "pcn" in pccType[1].lower()):
            inShapeNet = Path(preprocessDir, "data_pcd", "ShapeNet.json")
            #inCategory = Path(preprocessDir, "data_pcd", "category.txt")
            outShapeNet = Path(datasetConfigDir, "ShapeNet.json")
            #outCategory = Path(datasetConfigDir, "category.txt")
            shutil.copy(inShapeNet, outShapeNet)
            #shutil.copy(inCategory, outCategory)
        else:
            inTest = Path(preprocessDir, "data_npy", "test.txt")
            inTrain = Path(preprocessDir, "data_npy", "train.txt")
            outTest = Path(datasetConfigDir, "test.txt")
            outTrain = Path(datasetConfigDir, "train.txt")
            shutil.copy(inTest, outTest)
            shutil.copy(inTrain, outTrain)

    print(f'INFERENCE START: {datetime.datetime.now().strftime("%Y.%m.%d %H:%M:%S.%f")}\n')
    pccResult = subprocess.run(inferenceCommand, cwd=pccRunDir, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print("PCC output:", pccResult.stdout)
    print("PCC errors:", pccResult.stderr)
    print("PCC return code:", pccResult.returncode)
    print(f'\nINFERENCE END: {datetime.datetime.now().strftime("%Y.%m.%d %H:%M:%S.%f")}')

    # Copy results.
    if (os.path.isdir(predictedDir)):
        common_tools.clear_directory(predictedDir)
    print(f'Copying results to: {predictedDir}')
    shutil.copytree(outputsDir, predictedDir, dirs_exist_ok=True)

def sf_postprocess(predictedDir, preprocessDir, numberPrefix='00010000-', createVariants:bool=False, variantsOffset:float=0.33,
                   oldRecalculation:bool=False, removeDuplicates:bool=True, bpcc:bool=False, resultsDirName:str='results',
                   inDirPartial=None, maxDistances:list=[0.005, 0.006, 0.007, 0.008], z_weight:float=1.0):
    if (numberPrefix is None):
        numberPrefix = ''
    
    resultsDir = os.path.join(preprocessDir, resultsDirName)
    tempDir = os.path.join(resultsDir, 'temp')
    Path(tempDir).mkdir(parents=True, exist_ok=True)
    if not bpcc:
        suffixes = ['_pred.ply', '_partial.ply']
        predExtension = '.ply'
    else:
        suffixes = ['_predict.npy', '_predict.npy']
        predExtension = '.npy'

    dataObjDir = os.path.join(preprocessDir, 'data_obj')
    partialSinglesDir = os.path.join(dataObjDir, 'partial_singles')
    #partialSectorsUnscaledDir = os.path.join(dataObjDir, 'partial_sectors_unscaled')

    number_of_copied:int = 0
    for path in os.listdir(predictedDir):
        srcPath = os.path.join(predictedDir, path)
        if os.path.isfile(srcPath) and (srcPath.lower().endswith(suffixes[0]) or srcPath.lower().endswith(suffixes[1])):
            if (numberPrefix != '' and not path.startswith(numberPrefix)):
                dstFilename = numberPrefix + path
            else:
                dstFilename = path
            if not bpcc:
                # Just copy the file to the temp directory.
                dstPath = os.path.join(tempDir, dstFilename)
                os.makedirs(os.path.dirname(dstPath), exist_ok=True)
                shutil.copy(srcPath, dstPath)
                number_of_copied += 1
            else:
                # Check if it's one of non-sectorized files and copy it to the final dir instead.
                suffixIndex = path.rindex(suffixes[0])
                baseName = path[:suffixIndex]
                copyToTemp = True
                for tempPath in os.listdir(partialSinglesDir):
                    tempLastDotIndex = tempPath.rindex('.')
                    tempBaseName = tempPath[:tempLastDotIndex]
                    if (baseName == tempBaseName):
                        finalSuffixIndex = dstFilename.rindex(suffixes[0])
                        finalBaseName = dstFilename[:finalSuffixIndex]
                        dstPath = os.path.join(resultsDir, finalBaseName + '.obj')
                        pcd_io.convert(srcPath, dstPath)
                        copyToTemp = False
                        number_of_copied += 1
                        break
                if copyToTemp:
                    dstPath = os.path.join(tempDir, dstFilename)
                    os.makedirs(os.path.dirname(dstPath), exist_ok=True)
                    shutil.copy(srcPath, dstPath)
                    number_of_copied += 1
    
    if (number_of_copied <= 0):
        common_tools.force_print_to_console(f'ERROR: no result files were found in the following path: {predictedDir}')
        return

    if not bpcc:
        # Normalize the non-sectorized data and move it to the results dir.
        for path in os.listdir(partialSinglesDir):
            srcPath = os.path.join(partialSinglesDir, path)
            lastDotIndex = path.rindex('.')
            baseName = path[:lastDotIndex]
            '''
            tempPredPath = os.path.join(tempDir, baseName+suffixes[0])
            if (not os.path.exists(tempPredPath)):
                print(f'WARNING: missing file in the temp directory: {baseName+suffixes[0]}')
                continue
            dstPath = os.path.join(resultsDir, path[len(numberPrefix) : lastDotIndex])+'.obj'
            pcd_io.normalize_icp(tempPredPath, srcPath, dstPath)
            os.remove(tempPredPath)
            '''
            tempPredPath = os.path.join(tempDir, baseName+suffixes[0])
            tempPartialPath = os.path.join(tempDir, baseName+suffixes[1])
            if (not os.path.exists(tempPredPath) or not os.path.exists(tempPartialPath)):
                print(f'WARNING: missing files in the temp directory ({baseName+suffixes[0]}, {baseName+suffixes[1]})')
                continue
            dstPath = os.path.join(resultsDir, path[len(numberPrefix) : lastDotIndex])+'.obj'
            pcd_io.normalize_by_reference(refGoodFile=srcPath, refBadFile=tempPartialPath,
                                        fileToFix=tempPredPath, outputFile=dstPath)
            os.remove(tempPartialPath)
            os.remove(tempPredPath)
    
    # Restore original sector names so the full point clouds can be restored.
    for path in os.listdir(tempDir):
        srcPath = os.path.join(tempDir, path)
        if os.path.isfile(srcPath) and path.lower().endswith(suffixes[0]):
            suffixIndex = path.rindex(suffixes[0])
            dstName = restore_dots( path[:suffixIndex]+'.obj' )
            dstPath = os.path.join(tempDir, dstName)
            pcd_io.convert(srcPath, dstPath)
            #print(f'DEBUG: {path} -> {dstName}')
    
    # Remove all unneeded files from the temp directory.
    for path in os.listdir(tempDir):
        fullPath = os.path.join(tempDir, path)
        if os.path.isfile(fullPath) and path.lower().endswith(predExtension):
            #print(f'Removing temp file: {path}')
            os.remove(fullPath)

    # Copy original sectors, without replacing the completed ones.
    partialSectorsDir = os.path.join(dataObjDir, 'partial_sectors')
    for path in os.listdir(partialSectorsDir):
        srcPath = os.path.join(partialSectorsDir, path)
        dstPath = os.path.join(tempDir, path)
        if not os.path.exists(dstPath):
            os.makedirs(os.path.dirname(dstPath), exist_ok=True)
            print(f'Restoring unmodified file: {path}')
            shutil.copy(srcPath, dstPath)
        else:
            # Align the reconstructed sector to the input partial point cloud.
            pcd_io.normalize_icp(dstPath, srcPath, dstPath)

    suffix = '_info.txt'
    for path in os.listdir(tempDir):
        if (path.lower().endswith(suffix)):
            suffixIndex = path.rindex(suffix)
            prefix = path[:suffixIndex]
            dstName = path[len(numberPrefix) : suffixIndex-1] + '.obj'
            dstPath = os.path.join(resultsDir, dstName)
            print(f'Combining sectors: {Path(dstPath).name}')
            combine_sectors(inDir=tempDir, outFile=dstPath, prefix=prefix, extension='.obj', rescale=True,
                            createVariants=createVariants, variantsOffset=variantsOffset,
                            oldRecalculation=oldRecalculation, removeDuplicates=removeDuplicates)
    
    # Merge results with input files.
    if (inDirPartial is not None):
        inputNames = os.listdir(inDirPartial)
        for path in os.listdir(resultsDir):
            if (path.lower().endswith('.obj')):
                lastDotIndex = path.rindex('.')
                baseName = path[:lastDotIndex]
                tempNames = list(filter(lambda x: baseName in x, inputNames))
                print(f'tempNames: {tempNames}')
                
                if (len(tempNames) > 0):
                    inputName = tempNames[0]
                    inPath = os.path.join(inDirPartial, inputName)
                    inCoords = common_tools.read_point_cloud(inPath)
                    for dist in maxDistances:
                        resPath = os.path.join(resultsDir, path)
                        #resCoords = common_tools.read_point_cloud(resPath)
                        filteredPath = os.path.join(resultsDir, baseName+'_filter-'+str(dist)+'.obj')
                        print(f'Saving {Path(filteredPath).name}')
                        resCoords = distance.select_and_remove_outliers(resPath, inPath, filteredPath, dist, z_weight)

                        resCoords.extend(inCoords)
                        if (removeDuplicates):
                            oldCount = len(resCoords)
                            resCoords = common_tools.downsample_points(resCoords)
                            newCount = len(resCoords)
                            print(f'Downsampled merged point cloud from {oldCount} to {newCount} points.')
                        outPath = os.path.join(resultsDir, baseName+'_filter-'+str(dist)+'_merge.obj')
                        print(f'Saving {Path(outPath).name}')
                        pcd_io.coords_to_file(resCoords, outPath)

def sf_postprocess_preset(predictedDir, preprocessDir, presetPath, variantsOffset:float=0.33, oldRecalculation:bool=False,
                          resultsDirName:str='results', inDirPartial=None, z_weight:float=1.0):
    jsonFile = open(presetPath, 'r')
    preset = json.load(jsonFile)
    bpcc:bool = False
    pccType = preset["pccConfig"]["type"]
    if ("pointr" in pccType[0].lower()):
        bpcc = True
    sf_postprocess(predictedDir=predictedDir, preprocessDir=preprocessDir, numberPrefix=preset["prefix"], createVariants=preset["create_variants"],
                   variantsOffset=variantsOffset, oldRecalculation=oldRecalculation, removeDuplicates=preset["postprocessing"]["remove_duplicates"],
                   bpcc=bpcc, resultsDirName=resultsDirName, inDirPartial=inDirPartial, maxDistances=preset["postprocessing"]["max_outlier_distances"], z_weight=z_weight)
    jsonFile.close()
