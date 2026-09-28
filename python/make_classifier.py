"""
    Generate a neural network based on point cloud extra parameters and
    save it into a file with the scaler
    JSON config parameters:
        datadir: root folder for training data, subfolder names have to match categories
        categories: categories for classification
        custom_extra_dim_names: features to consider in classification
        epochs: number of epochs for training the neural network
        batch_size: batch size for training
        early_stop: number of epochs with no improvement after which training will be stopped
        weight_decay: weight decay parameter for adamw optimizer
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
    "epochs": 30,
    "batch_size": 32,
    "early_stop": 3,
    "weight_decay": 1e-4,
    "model_name": "barnag_modell.pickle"
}
"""
import time
import sys
import os.path
import glob
import pickle
import json
import argparse
import numpy as np
import laspy
import matplotlib.pyplot as plt
import matplotlib.colors as mcolor
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report
from sklearn.inspection import permutation_importance
from sklearn.preprocessing import MinMaxScaler, StandardScaler, RobustScaler
from sklearn.base import BaseEstimator
from keras.models import Sequential
from keras.layers import Dense, Dropout, Input
from keras.callbacks import EarlyStopping
from keras.optimizers import AdamW
from tensorflow import convert_to_tensor, GradientTape, float32

def feature_importance(model, x, class_id, batch_size=4096):
    """ get importance of input params for a class

        :param model: the neural network model
        :param x: imput to neural network
        :param class_id: output class to examine
    """
    all_gradients = []
    for start in range(0, len(x), batch_size):
        end = min(start + batch_size, len(x))
        x_batch = convert_to_tensor(x[start:end], dtype=float32)

        with GradientTape() as tape:
            tape.watch(x_batch)
            y = model(x_batch, training=False)
            score = y[:, class_id]

        gradients = tape.gradient(score, x_batch)
        all_gradients.append(gradients.numpy())
        del x_batch, y, score, gradients

    return np.concatenate(all_gradients, axis=0)

def importance_matrix(model, x, num_classes):
    """ get importance of input params for given classes

        :param model: the neural network model
        :param x: imput to neural network
        :param class_ids: output classes to examine (list)
    """
    im = np.empty((x.shape[1], num_classes)) # create output matrix
    for class_id in range(num_classes):
        importance = feature_importance(model, x, class_id)
        mean_importance = np.mean(np.abs(importance), axis=0)
        im[:,class_id] = mean_importance
    return im

def read_las_file_scalarfields(las):
    """ Get list of extra scalar fields

        :param las: laspy.lasdata.LasData loaded las file
        :returns: list of extra scalar field names
    """
    return list(las.point_format.extra_dimension_names)

def validate_extra_dim_names(custom_names, las_names):
    """ Remove missing custom scalar field names 

        :param custom_names: requered scalar field names
        :param las_names: available scalar field names in las file
        :returns: list of names in custom_names but not in las_names
                  e.g. the missing values in point cloud
    """
    return list(set(custom_names) - set(las_names))

def pc_features2np(pnts, custom_extra_dim_names):
    """ Get scalar field data from point cloud

        :param pnts: laspy.point.record from loaded las file
        :param custom_extra_dim_names: required scalar fields
        :returns: required scalar fileds + xyz and colors in a numpy array
    """
    pc_xyz_features = None

    for extra_dim in custom_extra_dim_names:
        col = pnts[extra_dim].reshape(-1,1)  # single column
        if pc_xyz_features is None:
            pc_xyz_features = col
        else:
            pc_xyz_features = np.concatenate([pc_xyz_features, col], axis=1)
    # add XYZ and color data
    xyz = np.column_stack((pnts.x, pnts.y, pnts.z))
    r = (pnts.red // 256).astype(np.uint8).reshape((-1,1))
    g = (pnts.green // 256).astype(np.uint8).reshape((-1,1))
    b = (pnts.blue // 256).astype(np.uint8).reshape((-1,1))
    colors = np.concatenate([r, g, b], axis=1).reshape(-1,3)

    # put data together
    if pc_xyz_features is None:
        pc_xyz_colors_features = np.concatenate([xyz, colors], axis=1)
    else:
        pc_xyz_colors_features = np.concatenate([xyz, colors, pc_xyz_features], axis=1)

    # remove rows with NAN values
    pc_xyz_colors_features_filt = (pc_xyz_colors_features[~np.isnan(pc_xyz_colors_features).any(axis=1), :])

    return pc_xyz_colors_features_filt

def load_training_data(categories, datadir, custom_names):
    """ load training data from categorised folders

        :param categories: category names, same as folder name with laballed data
        :param datadir: root directory for category data
        :param custom_names: necessary feature names 
        :returns X. y (features and numeric labels
    """
    X_features = []
    y_labels = []

    # process categories
    for category in categories:
        print(f"*** CATEGORY: {category}")
        path = os.path.join(datadir, category)  # path to category data
        class_num = categories.index(category)  # numeric label is index
        for pc in glob.glob(os.path.join(path, "*.las")):     # process las files in dir
            print(f"    {pc}")
            las = laspy.read(pc)
            # check presence of extra dims
            missing_names = validate_extra_dim_names(custom_names, list(las.point_format.dimension_names))
            if len(missing_names) > 0:
                print(f"Missing scalars {missing_names} from {pc}")
                sys.exit(1)
            if len(X_features) == 0:    # add first feature and label
                X_features = pc_features2np(las.points, custom_names)
                y_labels = np.full(X_features.shape[0], class_num)
            else:   # following features and labels
                features = pc_features2np(las.points, custom_names)
                labels = np.full(features.shape[0], class_num)
                X_features = np.concatenate((X_features, features), axis=0)
                y_labels = np.concatenate((y_labels, labels), axis=0)
    return X_features, y_labels

def training_plot(model, epochs):
    """ plot training accuracy and loss curves
    """
    acc = model.history.history['accuracy']
    val_acc = model.history.history['val_accuracy']
    loss = model.history.history['loss']
    val_loss = model.history.history['val_loss']

    plt.figure(figsize=(15, 15))
    plt.subplot(2, 2, 1)
    plt.plot(range(len(acc)), acc, label='Training Accuracy')
    plt.plot(range(len(val_acc)), val_acc, label='Validation Accuracy')
    plt.legend(loc='lower right')
    plt.title('Training and Validation Accuracy')

    plt.subplot(2, 2, 2)
    plt.plot(range(len(loss)), loss, label='Training Loss')
    plt.plot(range(len(val_loss)), val_loss, label='Validation Loss')
    plt.legend(loc='upper right')
    plt.title('Training and Validation Loss')
    plt.show()

class KerasEstimator(BaseEstimator):
    """ Keras - scikit-learn compability
    """
    def __init__(self, model):
        self.model = model

    def fit(self, X, y=None):
        return self

    def predict(self, X):
        preds = self.model.predict(X)
        return np.argmax(preds, axis=1)

    def score(self, X, y):
        return np.mean(self.predict(X) == y)

def parameter_importance(model, feature_names, X_test, y_test):
    """ Estimate parameter importance for all classes """
    wrapper = KerasEstimator(model) # estimator from trained model

    # calculate importance of parameters on test data
    result = permutation_importance(
        wrapper, # the model
        X_test,
        y_test,
        n_repeats=10,
        #random_state=42,
        scoring='accuracy' # evaluation mode
    )

    # results
    imp = result.importances_mean
    res = sorted(zip(imp.tolist(), feature_names), reverse=True)
    return res

if __name__ == "__main__":
    start = time.time()
    parser = argparse.ArgumentParser()
    parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                        help='config file')
    parser.add_argument('-s', '--scaler', choices=['standard', 'minmax', 'robust'], default='standard',
                        help='scaler for feature data, default: minmax')
    parser.add_argument('-i', '--importance', action="store_true",
                        help='show importance of parameters')
    parser.add_argument('-m', '--importance_matrix', action="store_true",
                        help='show importance matrix of parameters/classes')
    parser.add_argument('-a', '--accuracy', action="store_true",
                        help='draw accuracy and loss curve')
    parser.add_argument('-c', '--with_colors', action="store_true",
                        help='add colors to features')
    parser.add_argument('-v', '--hsv_colors', action="store_true",
                        help=' convert colors to HSV, use together --with_colors')
    parser.add_argument('-l', '--large_net', action="store_true",
                        help='Use large net 5 hidden layer')
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
    EPOCHS = conf["epochs"]
    BATCH_SIZE = conf['batch_size']
    EARLY_STOP = conf['early_stop']
    WEIGHT_DECAY = conf['weight_decay']
    MODEL_NAME = conf["model_name"]
    # check output name
    try:
        with open(MODEL_NAME, "w") as f:
            pass
    except FileNotFoundError as e:
        print(f"File creation error: {MODEL_NAME}, {e}")
        sys.exit(1)
    # load training and test data
    X_features, y_labels = load_training_data(CATEGORIES, DATADIR,
                                              CUSTOM_EXTRA_DIM_NAMES)
    unique_labels, unique_label_counts = np.unique(y_labels, return_counts=True)
    for label, count, name in zip(unique_labels, unique_label_counts, CATEGORIES):
        print(f'{count:8} samples for label {label}/{name}')
    # scale features
    if args.scaler == 'standard':
        scaler = StandardScaler()
    if args.scaler == 'minmax':
        scaler = MinMaxScaler()
    elif args.scaler == 'robust':
        scaler = RobustScaler()
    else:
        scaler = MinMaxScaler()
    # skip coordinates and optionally colors in scaling
    if args.with_colors:
        ind = 3
        if args.hsv_colors:
            feature_names = ['Hue', 'Saturation', 'Value'] + CUSTOM_EXTRA_DIM_NAMES
            X_features[:,ind:ind+3] = mcolor.rgb_to_hsv(X_features[:,ind:ind+3] / 255.)
        else:
            feature_names = ['Red', 'Green', 'Blue'] + CUSTOM_EXTRA_DIM_NAMES
    else:
        ind = 6
        feature_names = CUSTOM_EXTRA_DIM_NAMES
    X_features_scaled = np.concatenate((X_features[:,0:ind], scaler.fit_transform(X_features[:,ind:X_features.shape[1]])), axis=1)
    # split data to train and test set
    X_train, X_test, y_train, y_test = train_test_split(
         X_features_scaled, y_labels, test_size=0.3, shuffle=True)
    # split test data to test and validation
    X_test, X_valid, y_test, y_valid = train_test_split(
         X_test, y_test, test_size=0.33, shuffle=True)

    num_classes = len(CATEGORIES) # size of output layer

    # build neural network
    callbacks = []
    if EARLY_STOP > 0:
        callbacks.append(EarlyStopping(monitor='loss', patience=EARLY_STOP))
    model = Sequential()
    model.add(Input(shape=(X_train.shape[1]-ind,)))   # xyz and/or rgb not used (-3/-6)
    if args.large_net:
        model.add(Dense(256, activation='relu'))
        model.add(Dropout(0.2))
        model.add(Dense(128, activation='relu'))
        model.add(Dropout(0.2))

    model.add(Dense(64, activation='relu'))
    model.add(Dropout(0.2))
    model.add(Dense(32, activation='relu'))
    #model.add(Dropout(0.15))
    model.add(Dense(16, activation='relu'))
    model.add(Dense(num_classes, activation='softmax'))
    model.compile(optimizer=AdamW(weight_decay=WEIGHT_DECAY),
                  loss='sparse_categorical_crossentropy',
                  metrics=['accuracy'])

    # train modell
    model.fit(X_train[:, ind:], y_train, batch_size=BATCH_SIZE, epochs=EPOCHS,
              callbacks = callbacks,
              validation_data=(X_valid[:, ind:], y_valid), verbose=2)
    print(model.summary())
    # save model, scaler & with_colors
    data = {"model": model, "scaler": scaler,
            "with_colors": args.with_colors,
            "hsv:colors": args.hsv_colors}
    with open(MODEL_NAME, 'wb') as f:
        pickle.dump(data, f)

    if args.accuracy:
        # show training and loss tendencies
        training_plot(model, EPOCHS)
    # accuracy on test data
    y_predictions = model.predict(X_test[:,ind:X_test.shape[1]])
    y_predictions = np.argmax(y_predictions, axis=1)
    print(f'Model accuracy on test data: {accuracy_score(y_test, y_predictions):.4f}')
    print("Accuracy on test data \n",
          classification_report(y_test,y_predictions, target_names = CATEGORIES))
    # accuracy on valditation data
    y_predictions = model.predict(X_valid[:,ind:X_valid.shape[1]])
    y_predictions = np.argmax(y_predictions, axis=1)
    print(f'Model accuracy on validation data: {accuracy_score(y_valid, y_predictions):.4f}')
    print(f"Accuracy on validation data \n {classification_report(y_valid,y_predictions, target_names = CATEGORIES)}")

    if args.importance:
        for val, name in parameter_importance(model, feature_names, X_test[:, ind:], y_test):
            print(f"{name:20} {val:10.5f}")
    if args.importance_matrix:
        i_m = importance_matrix(model, X_train[:, ind:], len(CATEGORIES))
        print(" "*21 + " ".join([ f"{name:^10s}" for name in CATEGORIES]))
        for i, row in enumerate(i_m):
            row_text = f"{feature_names[i]:21s}" + \
                        " ".join([f"{val:10.3f}" for val in row])
            print(row_text)

    print(f"execution time {(time.time() - start):.1f} seconds")
