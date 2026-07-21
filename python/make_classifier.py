"""
    Generate a neural network based on point cloud extra parameters

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
    "model_name": "barnag_modell.pickle"
}
"""
import os.path
import sys
import glob
import pickle
import json
import numpy as np
import pandas as pd     # TODO try to avoid, used only ones
import laspy
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report
from sklearn.inspection import permutation_importance
from sklearn.base import BaseEstimator
from keras.models import Sequential
from keras.layers import Dense, Dropout

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
        :returns: commom set of the two lists
    """
    return list(set(custom_names) & set(las_names))

def pc_features2np(las, custom_extra_dim_names):
    """ Get scalar field data from point cloud

        :param las: laspy.lasdata.LasData loaded las file
        :param custom_extra_dim_names: required scalar fields
        :returns: required scalar fileds + xyz and colors in a numpy array
    """
    pc_xyz_features = []

    #pc_dim_names = read_las_file_scalarfields(las)
    for extra_dim in custom_extra_dim_names:
        #indx = pc_dim_names.index(extra_dim)
        col = las[extra_dim].reshape(-1,1)
        if len(pc_xyz_features) == 0:
            pc_xyz_features = col
        else:
            pc_xyz_features = np.concatenate([pc_xyz_features, col], axis=1)
    # add XYZ and color data
    xyz = las.xyz.reshape((-1,3))
    r = (las.red // 256).astype(np.uint8).reshape((-1,1))
    g = (las.green // 256).astype(np.uint8).reshape((-1,1))
    b = (las.blue // 256).astype(np.uint8).reshape((-1,1))
    colors = np.concatenate([r, g, b], axis=1).reshape(-1,3)

    # put data together
    if len(pc_xyz_features) == 0:
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
        path = os.path.join(datadir, category)  # path to category data
        class_num = categories.index(category)  # numeric label is index
        for pc in glob.glob(os.path.join(path, "*.las")):     # process las files in dir
            las = laspy.read(pc)
            if len(X_features) == 0:    # add first feature and label
                X_features = pc_features2np(las, custom_names)
                y_labels = np.full(X_features.shape[0], class_num)
            else:   # following features and labels
                features = pc_features2np(las, custom_names)
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

    epochs_range = range(epochs)

    plt.figure(figsize=(15, 15))
    plt.subplot(2, 2, 1)
    plt.plot(epochs_range, acc, label='Training Accuracy')
    plt.plot(epochs_range, val_acc, label='Validation Accuracy')
    plt.legend(loc='lower right')
    plt.title('Training and Validation Accuracy')

    plt.subplot(2, 2, 2)
    plt.plot(epochs_range, loss, label='Training Loss')
    plt.plot(epochs_range, val_loss, label='Validation Loss')
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

def parameter_importance(model, custom_extra_dim_names, X_test, y_test):
    # add colors
    feature_names = ['Red', 'Green', 'Blue'] + custom_extra_dim_names
    wrapper = KerasEstimator(model) # estimator from trained model

    # calculate importance of parameters on test data
    result = permutation_importance(
        wrapper, # the model
        X_test[:, 3:],
        y_test,
        n_repeats=10,
        #random_state=42,
        scoring='accuracy' # evaluation mode
    )

    # results
    importance = pd.Series(result.importances_mean, index=feature_names)
    importance = importance.sort_values(ascending=False)
    return importance

if __name__ == "__main__":
    # read json config
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} json_config_file")
        sys.exit()
    with open(sys.argv[1], 'r', encoding="utf-8") as f:
        conf = json.load(f)
    DATADIR = conf["datadir"]
    CATEGORIES = conf["categories"]
    CUSTOM_EXTRA_DIM_NAMES = conf["custom_extra_dim_names"]
    EPOCHS = conf["epochs"]
    MODEL_NAME = conf["model_name"]
    # load training and test data
    X_features, y_labels = load_training_data(CATEGORIES, DATADIR, CUSTOM_EXTRA_DIM_NAMES)
    unique_labels, unique_label_counts = np.unique(y_labels, return_counts=True)
    for label, count in zip(unique_labels, unique_label_counts):
        print(f'Az {label}-s címkéhez tartozó elemek száma: {count}')
    # TODO scale features
    # split data to train and test set
    X_train, X_test, y_train, y_test = train_test_split(
         X_features, y_labels, test_size=0.3, shuffle=True)
    # split test data to test and validation
    X_test, X_valid, y_test, y_valid = train_test_split(
         X_test, y_test, test_size=0.33, shuffle=True)

    num_classes = len(CATEGORIES) # size of output layer

    # build neural network
    model = Sequential()
    model.add(Dense(64, activation='relu', input_dim=X_train.shape[1]-3))   # xyz not used TODO relative Z can be used
    model.add(Dropout(0.25))
    model.add(Dense(32, activation='relu'))
    model.add(Dropout(0.15))
    model.add(Dense(16, activation='relu'))
    model.add(Dense(num_classes, activation='softmax'))
    model.compile(optimizer='adam', loss='sparse_categorical_crossentropy',
                  metrics=['accuracy'])

    # train modell
    model.fit(X_train[:, 3:], y_train, batch_size=1024, epochs=EPOCHS,
              validation_data=(X_valid[:, 3:], y_valid), verbose=2)

    with open(MODEL_NAME, 'wb') as f:
        pickle.dump(model, f)   # save model

    # show training and loss tendencies
    training_plot(model, EPOCHS)
    # accuracy on test data
    y_predictions = model.predict(X_test[:,3:X_test.shape[1]])
    y_predictions = np.argmax(y_predictions, axis=1)
    print(f'Model accuracy on test data: {accuracy_score(y_test, y_predictions):.4f}')
    print("Accuracy on test data \n",classification_report(y_test,y_predictions, target_names = CATEGORIES))
    # accuracy on valditation data
    y_predictions = model.predict(X_valid[:,3:X_valid.shape[1]])
    y_predictions = np.argmax(y_predictions, axis=1)
    print(f'Model accuracy on validation data: {accuracy_score(y_valid, y_predictions):.4f}')
    print(f"Accuracy on validation data \n {classification_report(y_valid,y_predictions, target_names = CATEGORIES)}")

    print(parameter_importance(model, CUSTOM_EXTRA_DIM_NAMES, X_test, y_test))
