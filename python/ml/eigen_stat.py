#! /usr/bin/env python3

from sys import argv
import pandas as pd
from matplotlib import pyplot as plt

if len(argv) < 2:
    print(f"Usage {argv[0]} csv_file")
    exit()
csv_file = argv[1]
try:
    data = pd.read_csv(csv_file, sep=" ", index_col=False)
except:
    print(f"Error opening/reading file: {csv_file}")
    exit()

mean_val = data[['range', 'anisotropy', 'linearity', 'planarity', 'surf_vari', 'omni_vari', 'spher', 'entropy']].groupby('range').mean()
std_val = data[['range', 'anisotropy', 'linearity', 'planarity', 'surf_vari', 'omni_vari', 'spher', 'entropy']].groupby('range').std()
min_val = data[['range', 'anisotropy', 'linearity', 'planarity', 'surf_vari', 'omni_vari', 'spher', 'entropy']].groupby('range').min()
max_val = data[['range', 'anisotropy', 'linearity', 'planarity', 'surf_vari', 'omni_vari', 'spher', 'entropy']].groupby('range').max()
print(80  * "-")
print("Means:")
print(mean_val)
print(80  * "-")
print("Mean errors:")
print(std_val)
print(80  * "-")
print("Min values:")
print(min_val)
print(80  * "-")
print("Max values:")
print(max_val)
fig, ax = plt.subplots(4, 7, figsize=(15, 10))
fig.tight_layout()
for i, name in enumerate(['anisotropy', 'linearity', 'planarity', 'surf_vari', 'omni_vari', 'spher', 'entropy']):
    ax[0, i].plot(mean_val.index, mean_val[name])
    ax[0, i].set_title(f'Mean {name}')
    ax[1, i].plot(std_val.index, std_val[name])
    ax[1, i].set_title(f'Std {name}')
    ax[2, i].plot(min_val.index, min_val[name])
    ax[2, i].set_title(f'Min {name}')
    ax[3, i].plot(max_val.index, max_val[name])
    ax[3, i].set_title(f'Max {name}')
plt.show()
