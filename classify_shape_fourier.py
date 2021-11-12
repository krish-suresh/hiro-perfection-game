import cv2
import numpy as np
import matplotlib.pyplot as plt

def get_largest_contour(input):
    input_blur = cv2.GaussianBlur(input,(5,5),cv2.BORDER_DEFAULT)
    hsv = cv2.cvtColor(input_blur, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([0,50,50])
    upper_yellow = np.array([50, 255, 255])
    mask_all = cv2.inRange(hsv, lower_yellow, upper_yellow)
    ret,thresh = cv2.threshold(mask_all, 40, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    mask = np.zeros(input.shape[0:2], np.uint8)
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        cv2.fillPoly(mask, pts =[c], color=255)
    return mask, c
def get_distances_to_centroid(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    dists = []
    for point in contour.squeeze():
        dists.append(cv2.norm(point-(cx,cy)))
    return dists

input = cv2.imread('shapes/piece_5.png')
input_mask,contour = get_largest_contour(input)
contour_array = contour[:, 0, :]
contour_complex = np.empty(contour_array.shape[:-1], dtype=complex)
contour_complex.real = contour_array[:, 0]
contour_complex.imag = contour_array[:, 1]
fourier_result = np.fft.fft(contour_complex)
centered_fourier = np.fft.fftshift(fourier_result)
center_index = int(len(centered_fourier) / 2)
degree = 100
center_range = centered_fourier[int(center_index - degree / 2):int(center_index + degree / 2)]
truncated_fourier = np.fft.ifftshift(center_range)
contour_reconstruct = np.fft.ifft(truncated_fourier)
contour_reconstruct = np.array(
    [contour_reconstruct.real, contour_reconstruct.imag])
contour_reconstruct = np.transpose(contour_reconstruct)
contour_reconstruct = np.expand_dims(contour_reconstruct, axis=1)
# make positive
if contour_reconstruct.min() < 0:
    contour_reconstruct -= contour_reconstruct.min()
# normalization
contour_reconstruct *= 800 / contour_reconstruct.max()
# type cast to int32
contour_reconstruct = contour_reconstruct.astype(np.int32, copy=False)
print(contour_reconstruct)
black = np.zeros((800,800), np.uint8)
# draw and visualize
cv2.fillPoly(black, pts =[contour_reconstruct], color=255)
cv2.imshow("input", input)
cv2.imshow("black", black)
# plt.plot(get_distances_to_centroid(contour))
# plt.plot(get_distances_to_centroid(contour_rot))
# plt.show()
# plt.plot(truncated_fourier)
# plt.plot(contour_rot_fft)
# plt.show()
# cv2.imshow('input',input)
# cv2.imshow('input_rot',input_mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
# quit()


# Find contour of input image
# Loop for 25 images
#   find centroid for both input and test_shape
#   shift mask so centroid of input and test_shape is at center of image
#   Loop for rotations
#       rotate input mask by rotation val
#       find intersection between input and test_shape - size of bitwise and
#       find union between input and test_shape - size of bitwise or
#       if iou is greater than last max store save rotation and shape as best fit