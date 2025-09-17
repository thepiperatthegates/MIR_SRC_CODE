import numpy as np

ex = np.arange(12)
ex1 = ex.reshape(-1, 4)

col1 = ex1[:, 0]

print(col1.ndim)