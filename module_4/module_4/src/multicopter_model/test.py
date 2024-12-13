import numpy as np

rot_mat = np.array([[[1], [2], [3]],
					[[4], [5], [6]],
				    [[7], [8], [9]]])

m = 0.5
a = m * rot_mat
print(a)