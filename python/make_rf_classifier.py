"""
    Generate a random forest classifier for point clouds using extra parameters and
    save it into a file with the scaler
    JSON config parameters:
        datadir: root folder for training data, subfolder names have to match categories
        categories: categories for classification
        custom_extra_dim_names: features to consider in classification
        model_name: file name for saved model

    Sample json config:

    {
    "datadir": "2020/barnag_classes",
    "categories": ["epulet", "fa", "oszlop", "vezetek"],
    "custom_extra_dim_names": ["Omnivariance (0.25)",
                              "Anisotropy (0.25)",
                              "Planarity (0.25)",
                              "Sphericity (0.25)",
                              "Linearity (0.25)",
                              "Omnivariance (0.5)",
                              "Anisotropy (0.5)",
                              "Planarity (0.5)",
                              "Sphericity (0.5)",
                              "Linearity (0.5)",
                              "Omnivariance (1)",
                              "Anisotropy (1)",
                              "Planarity (1)",
                              "Sphericity (1)",
                              "Linearity (1)"
                              ],
    "model_name": "barnag_modell.pickle"
}
"""
import time
import sys
import pickle
import json
import argparse
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier

from make_classifier import load_training_data

if __name__ == "__main__":
    start = time.time()
    parser = argparse.ArgumentParser()
    parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                        help='config file')
    parser.add_argument('-c', '--with_colors', action="store_true",
                        help='add colors to features')
    parser.add_argument('-i', '--importance', action="store_true",
                        help='show importance of parameters')
    args = parser.parse_args()
    # read json config
    try:
        with open(args.name[0], 'r', encoding="utf-8") as f:
            conf = json.load(f)
    except FileNotFoundError:
        print(f"{args.name[0]} config file not found")
        sys.exit(1)
    except json.decoder.JSONDecodeError as e:
        print(f"JSON decode error: {e}")
        sys.exit(1)
    DATADIR = conf["datadir"]
    CATEGORIES = conf["categories"]
    CUSTOM_EXTRA_DIM_NAMES = conf["custom_extra_dim_names"]
    MODEL_NAME = conf["model_name"]
    # load training and test data
    X_features, y_labels = load_training_data(CATEGORIES, DATADIR,
                                              CUSTOM_EXTRA_DIM_NAMES)
    unique_labels, unique_label_counts = np.unique(y_labels, return_counts=True)
    for label, count, name in zip(unique_labels, unique_label_counts, CATEGORIES):
        print(f'{label}/{name} címkéhez tartozó elemek száma: {count}')
    # split data to train and test set
    X_train, X_test, y_train, y_test = train_test_split(
         X_features, y_labels, test_size=0.3, shuffle=True)

    num_classes = len(CATEGORIES) # size of output layer

    model = RandomForestClassifier(
        n_estimators=300,
        max_features="sqrt",
        max_depth=None,
        min_samples_split=2,
        min_samples_leaf=4,
        class_weight="balanced",
        n_jobs=-1,
        random_state=42
    )

    # train modell
    if args.with_colors:
        ind = 3
    else:
        ind = 6
    model.fit(X_train[:,ind:], y_train)
    print(model.get_params())
    print(f"Score for train data {model.score(X_train[:,ind:], y_train)}")
    print(f"Score for test data {model.score(X_test[:,ind:], y_test)}")
    if args.importance:
        imp = model.feature_importances_
        if args.with_colors:
            feature_names = ['Red', 'Green', 'Blue'] + CUSTOM_EXTRA_DIM_NAMES
        else:
            feature_names = CUSTOM_EXTRA_DIM_NAMES
        if len(imp.tolist()) != len(feature_names):
            print("*** ERROR in feature count in importance {len(imp.tolist())} vs. feature_names")
        res = sorted(zip(imp.tolist(), feature_names), reverse=True)
        for i, name in res:
            print(f"{name:25}: {i:8.4f}")
    # save model
    data = { "model": model, "with_colors": args.with_colors}
    with open(MODEL_NAME, 'wb') as f:
        pickle.dump(data, f)
    print(f"execution time {time.time() - start} seconds")
