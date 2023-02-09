#! /usr/bin/env python3

''' 
Calculate 3D Features of a point cloud.
The script using the following module: https://pypi.org/project/pyCloudCompareCLI/

Usage: pc_3DfeaturesCC.py filename -j jsonconfig.json -r radius(def=0.3) -c class number (def=0) -output filename(.LAS) -d (optional)

positional arguments:
  pc_file_name          point cloud (.LAS, .PLY or other CloudCompare supported formats)

optional arguments:
  -h, --help            show this help message and exit
  -j JCONFIG, --jconfig JCONFIG
                        Add feature names in a json file to calculate
  -r RADIUS, --radius RADIUS
                        search radius [m]
  -c CLASS_NUM, --class_num CLASS_NUM
                        Add a class label to the LAS file. Default is unclassified(=0)
  -o OUTPUT, --output OUTPUT
                        Custom output filename - default: input filename + features.las
  -d, --debug           Switch on debug mode
 
A sample json file with all of the features:
pc_features.json
{
    "eigen_features": ["SUM_OF_EIGENVALUES",
                    "OMNIVARIANCE",
                    "EIGENTROPY", 
                    "ANISOTROPY", 
                    "PLANARITY", 
                    "LINEARITY",
                    "PCA1",
                    "PCA2",
                    "SURFACE_VARIATION",
                    "SPHERICITY",
                    "VERTICALITY",
                    "EIGENVALUE1",
                    "EIGENVALUE2",
                    "EIGENVALUE3"],
    "density_features": ["KNN",
                        "SURFACE",
                        "VOLUME"],  
    "curvature_features":["GAUSS",
                        "NORMAL_CHANGE"]
}

List of the 3D features:
    - Geometry-based features:
        - X, Y, Z,
        - A(rea)
        - Point density
    - Spectral-based features
        - R,G,B
        - mean R, mean G, mean B                    - TODO: based on sphere
        - std R, std G, std B                       - TODO: based on sphere
    - Eigen-based features
        - Eigenvalues (l1, l2, l3)
        - Sum of eigen values
        - Omnivariance
        - Eigentropy
        - Anisotropy
        - Linearity
        - Planarity
        - Surface variation
        - Speherecity
        ...
'''

import argparse
import pyCloudCompare as cc
import json
import sys

if __name__ == "__main__":

    # Command windows parameters
    parser = argparse.ArgumentParser()
    parser.add_argument('name', metavar='pc_file_name', type=str, nargs=1,
                    help='point cloud (.LAS)')
    parser.add_argument('-j', '--jconfig', type=str, default=None,
                        help='Add feature names in a json file to calculate') 
    parser.add_argument('-r', '--radius', type=float, default=0.3,
                        help='search radius [m]')
    parser.add_argument('-c', '--class_num', type=int, default=0,
                        help='Add a class label to the LAS file. Default is unclassified(=0)')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='Custom output filename - default: input filename + features.las')
    parser.add_argument('-d', '--debug', action='store_true',
                        help=' Switch on debug mode')
    args = parser.parse_args()

    # Parameters into variables
    pc_fnm = args.name[0]
    jname = args.jconfig
    rad = args.radius
    class_num = args.class_num
    pc_fnm_out = args.output
    debug_val = args.debug

    if pc_fnm_out is None:
        pc_fnm_out = pc_fnm[:-4]+'_features.las'
                        
    # Initalize CC
    cli = cc.CloudCompareCLI()
    cmd = cli.new_command()

    # Silent mode
    cmd.silent(is_silent=True)
    # Debug mode
    if debug_val:
        cmd.silent(is_silent=False)             
    

    # Switch auto save off
    cmd.auto_save(on_off=0)                                   

    # Read file
    cmd.open(pc_fnm)

    # Remove features
    cmd.remove_all_sfs()

    # Define feature names in lists
    # Calculate added features
    if jname is not None:
        try:
            with open(jname) as jfile:
                jdata = json.load(jfile)
                if "eigen_features" in jdata:
                    eigen_features = jdata["eigen_features"]
                if "density_features" in jdata:
                    density_features = jdata["density_features"]
                if "curvature_features" in jdata:
                    curvature_features = jdata["curvature_features"]
        except:
            print(f'File not found or invalid: {jname}')
            sys.exit(2)
    
    else:
        # Calculate all of the features
        eigen_features = ['SUM_OF_EIGENVALUES', 'OMNIVARIANCE', 'EIGENTROPY', 'ANISOTROPY', 'PLANARITY', 'LINEARITY',
        'PCA1', 'PCA2','SURFACE_VARIATION','SPHERICITY', 'VERTICALITY', 'EIGENVALUE1', 'EIGENVALUE2', 'EIGENVALUE3']
        density_features = ['KNN','SURFACE','VOLUME']
        curvature_features = ['GAUSS','MEAN','NORMAL_CHANGE']


    # Create command for each feature
    for eigen_feature in eigen_features:
        cmd.feature(eigen_feature,rad)
        if debug_val:
            print(eigen_feature)

    # Calculate density-based features 
    for density_feature in density_features:
        cmd.density(rad, density_feature)
        if debug_val:
            print(density_feature)


    # Calculate curvature-based features
    for curvature_feature in curvature_features:
        cmd.curvature(curvature_feature, rad)
        if debug_val:
            print(curvature_feature) 

    # TODO: add class number as SF value
    #cmd.sf_arithmetic()
    #-SF_ADD_CONST {SF name} {value}
    #cmd.sf_operation('LAST', 'SET', class_num)

    # Add export settings
    cmd.cloud_export_format(cc.CLOUD_EXPORT_FORMAT.LAS, extension='LAS')
    #cmd.cloud_export_format(cc.CLOUD_EXPORT_FORMAT.ASCII, add_header=1, precision=3, separator=';', extension='TXT')
    cmd.save_clouds(pc_fnm_out)
    
    # Execute script
    cmd.execute()
    if debug_val:   
        print(cmd)
 
