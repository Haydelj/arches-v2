import numpy as np

# Example input
y = np.array([1, 2, 3, 4, 5])
kernel = np.array([1, 0, -1, 4, 6, 7])

# Convolution
result = np.convolve(y, kernel, mode='same')

print("Size of y:", y.size)
print("Size of kernel:", kernel.size)
print("Size of result:", result.size)  # This will be the same as size of y
