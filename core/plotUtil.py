# plotUtil.py
# Plot utility file - Helps plot sumo data from csv
# Written by Mike Yannuzzi

import csv
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Initialize the lists for X and Y
data = pd.read_csv('/home/mike/RTOS/Selfless-Traffic-Routing-Testbed/djikstraData.csv')  
df = pd.DataFrame(data)
df.plot()
# df.plot(kind='scatter', x='time', y='updated_Mean_Deadline', linestyle='solid')
plt.show()