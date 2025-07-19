import os
import numpy as np



#
#
#
# with open(f"{str('new')}.csv", "r") as file:
#     data = np.array([])
#     for row in file:
#         row_values = list(map(float, row.strip().split(';')))
#         data = np.append(data, row_values)
#

#
# print(data)

data1 = np.array([0.432, 2.4123])
data2 = np.array([0.9432, 5.2124])

print(np.tan(data1/data2))