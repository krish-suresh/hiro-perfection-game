import cv2
import numpy as np

frame = cv2.imread('game_pieces.jpg')
print()
num_rows = 5
num_cols = 5
row_pixel_prev = 0

for i in range(num_rows):
    row_pixel = int((i+1)*frame.shape[0]/num_rows)
    col_pixel_prev = 0
    for j in range(num_cols):
        col_pixel = int((j+1)*frame.shape[1]/num_cols)
        crop = frame[row_pixel_prev:row_pixel, col_pixel_prev:col_pixel]
        shape_number = i*5 + j
        # print(f"Shape number {shape_number}") 
        # print(f"Row: {row_pixel} Col: {col_pixel}")
        # cv2.imshow('Crop', crop)
        # cv2.waitKey(0)
        col_pixel_prev = col_pixel
        cv2.imwrite(f"shapes/piece_{shape_number}.png", crop)
    row_pixel_prev = row_pixel

# cv2.imshow('Original', frame)
# cv2.waitKey(0)
# cv2.destroyAllWindows()