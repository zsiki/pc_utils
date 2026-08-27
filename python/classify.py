"""
    use neural network model to classify point cloud points
    the model should be saved by make_classifier.py, a scaler is also saved
    processing is made in chunks of the point cloud, chunk size is a parameter
    the different classes are saved into separate las files,
    the name of the files contains class name and saved into the point cloud's folder

    Some parameters are given in a json config file (it can be the same as
    the config file of make_classifier.py. Configuration parameters used by
    classifier.py: categories, model_name, pc_name, custom_extra_dim_names

"""
import time
import pickle
import json
import os.path
import argparse
import numpy as np
import laspy
from make_classifier import pc_features2np

if __name__ == "__main__":
    start = time.time()
    CHUNK_SIZE = 2_000_000
    LIMIT = None
    parser = argparse.ArgumentParser()
    parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                        help='config file')
    parser.add_argument('-c', '--chunk_size', type=int, default=CHUNK_SIZE,
                        help=f'chunk size for processing large point cloud, default: {CHUNK_SIZE}')
    parser.add_argument('-l', '--limit', type=float, default=LIMIT,
                        help='probability limit to accep point in class, only for neural networks, default: no limit')
    parser.add_argument('-p', '--point_cloud', type=str, default=None,
                        help='point cloud to process overwrites pc_name from config, default: pc_name param from config')
    args = parser.parse_args()
    # read json config
    with open(args.name[0], 'r', encoding="utf-8") as f:
        conf = json.load(f)

    CATEGORIES = conf['categories']
    MODEL_NAME = conf["model_name"]
    if args.point_cloud is None:
        PC_NAME = conf["pc_name"]
    else:
        PC_NAME = args.point_cloud
    print(PC_NAME)
    TARGET_NAME = os.path.splitext(PC_NAME)[0]
    CUSTOM_EXTRA_DIM_NAMES = conf["custom_extra_dim_names"]

    data = pickle.load(open(MODEL_NAME, 'rb'))   # load pre-trained model
    model = data["model"]
    scaler = data.get("scaler", None)
    with_colors = data.get("with_colors", False)

    lasf = laspy.open(PC_NAME, mode="r")
    num_classes = len(CATEGORIES) # model.output_shape[-1]
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.scales = np.array([0.001, 0.001, 0.001])   # millimeter precision
    # create and open output las files
    outputs = []
    for i in range(num_classes):
        out_name = TARGET_NAME + "_" + CATEGORIES[i] +".las"
        outputs.append(laspy.open(out_name, mode="w", header=header))
    if with_colors:
        ind = 3
    else:
        ind = 6     # skip colors
    # process las file in chunks
    for pnts in lasf.chunk_iterator(args.chunk_size):
        # save colors for output
        pc2np = pc_features2np(pnts, CUSTOM_EXTRA_DIM_NAMES) # convert to numpy array
        X_features = pc2np[:,ind:pc2np.shape[1]]      # exclude coordinates
        if scaler is not None:
            X_features = scaler.transform(X_features)   # scale data
        y_predict = model.predict(X_features)       # predict labels
        y_val = None
        if len(y_predict.shape) > 1:
            y_val = np.max(y_predict, axis=1)           # preserve probability
            y_predict = np.argmax(y_predict, axis=1)    # predict classes from MLP model
        # build output
        xyz = pc2np[:,0:3]                          # get coordinates
        colors = pc2np[:,3:6]                        # get colors
        classes = np.unique(y_predict)              # different class labels (int)
        # point count per class
        unique_labels, unique_label_counts = np.unique(y_predict, return_counts=True)
        for label, count in zip(unique_labels, unique_label_counts):
            print(f'Points in {label} class ({CATEGORIES[label]}): {count}')

        for class_n in classes:
            if args.limit is not None and y_val is not None:
                row_ix = np.where((y_predict == class_n) & (y_val >= args.limit))
            else:
                row_ix = np.where(y_predict == class_n)
            xyz_class = xyz[row_ix[0],:]        # select class points
            colors_class = colors[row_ix[0],:]
            out_points = laspy.ScaleAwarePointRecord.zeros(len(xyz_class),
                header=header)
            out_points.x = xyz_class[:, 0]     # add coordinates
            out_points.y = xyz_class[:, 1]
            out_points.z = xyz_class[:, 2]
            out_points.red = colors_class[:, 0] * 256
            out_points.green = colors_class[:, 1] * 256
            out_points.blue = colors_class[:, 2] * 256
            outputs[class_n].write_points(out_points)
    for i in range(num_classes):
        outputs[i].close()
    print(f"execution time {time.time() - start} seconds")
