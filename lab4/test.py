import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load an example image (replace with your image path)
image = cv2.imread('image.png', cv2.IMREAD_GRAYSCALE)

# Get the dimensions of the image
height, width = image.shape

# Generate random noise with normal distribution (mean=0, variance=1)
noise = np.random.normal(0, 1, (height, width))

# Normalize noise to range 0-255 and convert to uint8
normalized_noise = cv2.normalize(noise, None, 0, 255, cv2.NORM_MINMAX)
noise_image = normalized_noise.astype(np.uint8)

# Display the noise image
plt.imshow(noise_image, cmap='gray')
plt.title('Random Noise Image')
plt.axis('off')
plt.show()
