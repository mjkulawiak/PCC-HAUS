import argparse
import os
import sys
import logging
import datetime
from pathlib import Path
from tools import pcd_io
from tools import distance
from tools import sectors

def configure_log():
    currDir = Path(__file__).resolve().parent
    logsDir = Path(currDir, '_Logs')
    logsDir.mkdir(parents=True, exist_ok=True)
    dt = datetime.datetime.now()
    currLogPath = Path(logsDir, dt.strftime("log_%Y.%m.%d_%H.%M.%S")+'.txt')
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    file_handler = logging.FileHandler(currLogPath)
    file_handler.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    sys.stdout = file_handler.stream
    sys.stderr = file_handler.stream

if __name__ == '__main__':
    configure_log()
    print(f'pc_tools start: {datetime.datetime.now().strftime("%Y.%m.%d %H:%M:%S.%f")}\n')
    print(os.getcwd())
    mainParser = argparse.ArgumentParser(description='Convert point clouds between different formats.')
    subParser = mainParser.add_subparsers(dest='command')

    convert_parser = subParser.add_parser('convert')
    convert_parser.add_argument('input_file', type=str, help='path to input file')
    convert_parser.add_argument('output_file', type=str, help='path to output file')
    convert_parser.add_argument('-s', '--scale', type=float, default=1.0, help='target scale')

    batch_convert_parser = subParser.add_parser('batch_convert')
    batch_convert_parser.add_argument('input_dir', type=str, help='path to input dir')
    batch_convert_parser.add_argument('output_dir', type=str, help='path to output dir')
    batch_convert_parser.add_argument('outputExtension', type=str, help='output extension (.xyz, .obj, .pcd, .npy, .ply)')
    batch_convert_parser.add_argument('-ie', '--input_extension', type=str, default='.*', help='input file extension')
    batch_convert_parser.add_argument('-s', '--scale', type=float, default=1.0, help='target scale')

    sample_parser = subParser.add_parser('sample')
    sample_parser.add_argument('input_file', type=str, help='path to input file')
    sample_parser.add_argument('output_file', type=str, help='path to output file')
    sample_parser.add_argument('-pc', '--point_count', type=int, default=2048, help='target number of points')
    sample_parser.add_argument('-rr', '--random_radius', type=float, default=0.1, help='maximum distance for new points')
    sample_parser.add_argument('-fps', '--farthest_point_sampling', default=False, action="store_true", help='use FPS instead of random sampling')

    sample_dir_parser = subParser.add_parser('sample_dir')
    sample_dir_parser.add_argument('input_dir', type=str, help='path to input dir')
    sample_dir_parser.add_argument('output_dir', type=str, help='path to output dir')
    sample_dir_parser.add_argument('outputExtension', type=str, help='output extension (.xyz, .obj, .pcd, .npy, .ply)')
    sample_dir_parser.add_argument('-pc', '--point_count', type=int, default=2048, help='target number of points')
    sample_dir_parser.add_argument('-ie', '--input_extension', type=str, default='.*', help='input file extension')
    sample_dir_parser.add_argument('-rr', '--random_radius', type=float, default=0.1, help='maximum distance for new points')
    sample_dir_parser.add_argument('-fps', '--farthest_point_sampling', default=False, action="store_true", help='use FPS instead of random sampling')

    downsample_parser = subParser.add_parser('downsample')
    downsample_parser.add_argument('input_file', type=str, help='path to input file')
    downsample_parser.add_argument('output_file', type=str, help='path to output file')
    downsample_parser.add_argument('-vs', '--voxel_size', type=float, default=0.000001, help='voxel size used for downsampling')

    sf_preproc_parser = subParser.add_parser('sf_preproc')
    sf_preproc_parser.add_argument('input_dir', type=str, help='path to input dir')
    sf_preproc_parser.add_argument('output_dir', type=str, help='path to output dir')
    sf_preproc_parser.add_argument('-rf', '-rd', '--reference_dir', type=str, default=None, help='path to dir with reference complete points')
    sf_preproc_parser.add_argument('-opt', '--optimal_points', type=int, default=2048, help='optimal number of points per sector')
    sf_preproc_parser.add_argument('-max', '--max_points', type=int, default=6144, help='max number of points per sector, also min required for sectorization')
    sf_preproc_parser.add_argument('-p', '--prefix', type=str, default='00010000-', help='number prefix used for buildings')
    sf_preproc_parser.add_argument('-cv', '--create_variants', default=False, action="store_true", help='create variant sectors with different offsets')
    sf_preproc_parser.add_argument('-pcn', default=False, action="store_true", help='use PCN format instead of ShapeNet-55')

    sf_postproc_parser = subParser.add_parser('sf_postproc')
    sf_postproc_parser.add_argument('input_dir', type=str, help='path to dir with SeedFormer results (containing .ply files)')
    sf_postproc_parser.add_argument('output_dir', type=str, help='path to dir used for preprocessing')
    sf_postproc_parser.add_argument('-p', '--prefix', type=str, default='00010000-', help='number prefix used for buildings')
    sf_postproc_parser.add_argument('-cv', '--create_variants', default=False, action="store_true", help='include variant sectors with different offsets')
    sf_postproc_parser.add_argument('-rd', '--remove_duplicates', type=int, default=1, help='remove duplicate points')
    sf_postproc_parser.add_argument('-bpcc', default=False, action="store_true", help='Building-Point-Cloud-Completion version')
    sf_postproc_parser.add_argument('--results_dirname', type=str, default='results', help='name of the output directory with final results')
    sf_postproc_parser.add_argument('-m', '--merge', type=str, default=None, help='path to input dir, results will be merged with input point clouds')

    args = mainParser.parse_args()

    if args.command == 'convert':
        pcd_io.convert(args.input_file, args.output_file, [args.scale,args.scale,args.scale])
    
    elif args.command == 'batch_convert':
        pcd_io.batch_convert(args.input_dir, args.output_dir, args.outputExtension, args.input_extension, [args.scale,args.scale,args.scale])
    
    elif args.command == 'sample':
        pcd_io.sample_file(inFile=args.input_file, outFile=args.output_file, minPointCount=args.point_count, maxPointCount=args.point_count,
                           randomRadius=args.random_radius, useFps=args.farthest_point_sampling)
    
    elif args.command == 'sample_dir':
        pcd_io.sample_dir(inputDir=args.input_dir, outputDir=args.output_dir, outputExtension=args.outputExtension, minPointCount=args.point_count,
                          maxPointCount=args.point_count, inputExtension=args.input_extension, randomRadius=args.random_radius, useFps=args.farthest_point_sampling)
    
    elif args.command == 'downsample':
        pcd_io.downsample_file(args.input_file, args.output_file, args.voxel_size)
    
    elif args.command == 'sf_preproc':
        sectors.sf_preprocess(inDirPartial=args.input_dir, outDir=args.output_dir, inDirComplete=args.reference_dir,
                              optimalPointCount=args.optimal_points, maxSectPoints=args.max_points, numberPrefix=args.prefix,
                              createVariants=args.create_variants, oldRecalculation=False, pcn=args.pcn,
                              minPointsForSampling=1536)
    
    elif args.command == 'sf_postproc':
        sectors.sf_postprocess(predictedDir=args.input_dir, preprocessDir=args.output_dir, numberPrefix=args.prefix, createVariants=args.create_variants,
                               oldRecalculation=False, removeDuplicates=bool(args.remove_duplicates), bpcc=args.bpcc,
                               resultsDirName=args.results_dirname, inDirPartial=args.merge)
    
    else:
        print('Supported commands:  convert  batch_convert  sample  sample_dir  downsample  sf_preproc  sf_postproc')

    print(f'\npc_tools end: {datetime.datetime.now().strftime("%Y.%m.%d %H:%M:%S.%f")}')
    