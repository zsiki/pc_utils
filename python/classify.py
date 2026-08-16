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
import pickle
import json
import os.path
import argparse
import numpy as np
import laspy
from make_classifier import pc_features2np

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                        help='config file')
    parser.add_argument('-c', '--chunk_size', type=int, default=1_000_000)
    args = parser.parse_args()
    # read json config
    with open(args.name[0], 'r', encoding="utf-8") as f:
        conf = json.load(f)

    CATEGORIES = conf['categories']
    MODEL_NAME = conf["model_name"]
    PC_NAME = conf["pc_name"]
    TARGET_NAME = os.path.splitext(PC_NAME)[0]
    CUSTOM_EXTRA_DIM_NAMES = conf["custom_extra_dim_names"]

    data = pickle.load(open(MODEL_NAME, 'rb'))   # load pre-trained model
    model = data["model"]
    scaler = data["scaler"]
    with_colors = data["with_colors"]

    lasf = laspy.open(PC_NAME, mode="r")
    num_classes = model.output_shape[-1]
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.scales = np.array([0.001, 0.001, 0.001])   # millimeter precision
    # create and open output las files
    outputs = []
    for i in range(num_classes):
        out_name = TARGET_NAME + "_" + CATEGORIES[i] +".las"
        outputs.append(laspy.open(out_name, mode="w", header=header))
    #las = laspy.read(PC_NAME)   # load point cloud to classify
    # process las file in chunks
    for pnts in lasf.chunk_iterator(args.chunk_size):
        pc2np = pc_features2np(pnts, CUSTOM_EXTRA_DIM_NAMES, with_colors) # convert to numpy array
        X_features = pc2np[:,3:pc2np.shape[1]]      # exclude coordinates
        X_features = scaler.transform(X_features)   # scale data
        #print("X_feature", X_features.shape)
        y_predict = model.predict(X_features)       # predict labels
        y_predict = np.argmax(y_predict, axis=1)    # predict classes from model
        # build output
        xyz = pc2np[:,0:3]                          # get coordinates
        colors = pc2np[:,3:6]                       # get colors
        classes = np.unique(y_predict)              # different class labels (int)
        #labels = y_predict
        # point count per class
        unique_labels, unique_label_counts = np.unique(y_predict, return_counts=True)
        for label, count in zip(unique_labels, unique_label_counts):
            print(f'Points in {label} class ({CATEGORIES[label]}): {count}')

        for class_n in classes:
            row_ix = np.where(y_predict == class_n)
            xyz_class = xyz[row_ix[0],:]        # select class points
            color_class = colors[row_ix[0],:]   # select class colors
            out_points = laspy.ScaleAwarePointRecord.zeros(len(xyz_class),
                header=header)
            out_points.x = xyz_class[:, 0]     # add coordinates
            out_points.y = xyz_class[:, 1]
            out_points.z = xyz_class[:, 2]
            out_points.red = color_class[:, 0] # add colors
            out_points.green = color_class[:, 1]
            out_points.blue = color_class[:, 2]
            #las.write(TARGET_NAME + "_" + CATEGORIES[class_n] +".las")
            outputs[class_n].write_points(out_points)
    for i in range(num_classes):
        outputs[i].close()
