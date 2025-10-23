import numpy as np

q = np.ones(16)

q_ref = np.zeros((2,16))

print(q_ref)

q_ref[0,:] = q

print(q_ref)