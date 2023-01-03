#!/usr/bin/env python3

"""
    Generates neural network estimator for voxels based on
    statistical parameters calculated from eigen values of variancia-cova0riance
    matrix of points in the voxel
    Generate input CSV data using eigen.py
    Model is saved into a binary file

"""
# TODO make statistics of different eigen value expressions based on
# range parameter to see which are the significant values
from os import path
import sys
import pickle
import argparse
import pandas as pd
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import confusion_matrix, classification_report

def accuracy(conf_matrix):
    """ calculate accuracy from confusion matrix """
    diagonal_sum = conf_matrix.trace()
    sum_of_all_elements = conf_matrix.sum()
    return diagonal_sum / sum_of_all_elements

# columns used in model
data_columns = ['anisotropy', 'linearity', 'planarity', 'surf_vari',
                'omni_vari', 'spher', 'entropy']
# process command line parameters
parser = argparse.ArgumentParser()
parser.add_argument('-o', '--output_path', type=str, required=True,
                    help='path to save model, required')
parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                    help='CSV file to process')
args = parser.parse_args()

# load input file (space separated with header line)
csv_file = args.name[0]
try:
    orig_data = pd.read_csv(csv_file, sep=" ", index_col=False)
except:
    print(f"Error opening/reading file: {csv_file}")
    sys.exit()
# remove unneccesary columns
data = orig_data[data_columns]

# scale data
data_scaler = StandardScaler()
scaled_data = data_scaler.fit_transform(data)
data = pd.DataFrame(data=scaled_data, index=list(range(scaled_data.shape[0])),
                    columns=data_columns)
print(data_scaler.mean_)
print(data_scaler.scale_)
# add labels to data plane +- 5 cm
data['label'] = 0   # not a plane
data.loc[orig_data["range"] <= 0.05, 'label'] = 1   # plane

# Splitting the dataset into training (80%) and validation (20%) sets
training_set, validation_set = train_test_split(data, test_size = 0.2, random_state = 21)

# classifying the predictors and target variables as X and Y
X_train = training_set.iloc[:,0:-1].values
Y_train = training_set.iloc[:,-1].values
X_val = validation_set.iloc[:,0:-1].values
y_val = validation_set.iloc[:,-1].values

# Initializing the MLPClassifier
# TODO hidden_layer_sizes? and other parameters
classifier = MLPClassifier(hidden_layer_sizes=(150, 100, 50), max_iter=300,
                           activation = 'relu', solver='adam', random_state=1)

# Fitting the training data to the network
classifier.fit(X_train, Y_train)

# Predicting y for X_val (the validation set)
y_pred = classifier.predict(X_val)

print(f"Test set score: {classifier.score(X_val, y_val)}")

# Comparing the predictions against the actual observations in y_val
cm = confusion_matrix(y_pred, y_val)
print(cm)
print(classification_report(y_val, y_pred))

#Printing the accuracy
print("Accuracy of MLPClassifier : ", accuracy(cm))
print("weights:")
print(classifier.coefs_)
# save model
with open(args.output_path, 'wb') as fp:
    pickle.dump(classifier, fp)
# save scale
w = path.splitext(args.output_path)
with open(w[0]+"_scale"+w[1], 'wb') as fps:
    pickle.dump(data_scaler, fps)
# load saved model and scale with pickle.load(open(name, 'rb')
