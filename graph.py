import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

path = "raw-imu-solution/2024_06_05_13_02_32_885595.csv"
#path = "raw-imu-solution/2024_05_22_13_58_13_626548.csv"
data = pd.read_csv(path)
#data.yaw.plot()
data.pitch.plot()
#data.roll.plot()
plt.show()