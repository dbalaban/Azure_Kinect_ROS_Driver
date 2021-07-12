import sys
sys.path.remove('/home/david/prism-ws/devel/lib/python2.7/dist-packages')
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2
import numpy as np
from PIL import Image
from scipy import interpolate

def flip_image(img, col):
  rows = img.shape[0]
  cols = img.shape[1]

  if col < cols/2:
    max_col = 2*col + 1
    min_col = 0
  else:
    max_col = cols
    min_col = 2*col - cols
  
  print("columns: {}-{}".format(min_col, max_col))

  flipped = img.copy()
  for row in range(rows):
    flipped[row, min_col:max_col] = np.flip(img[row, min_col:max_col])
  return flipped

def eval_flip(img, flipped):
  diff = 2*np.abs(img-flipped)/(img + flipped)
  cv2.imshow('symmetry test', diff)
  significance = (np.abs(img - flipped) > 30).astype(np.float)
  cv2.imshow('significance', significance)
  print(np.nansum(diff))

def interpolate_region(region, data, holes):
  rows, cols = data.shape
  xs = np.repeat(np.arange(cols).reshape([1,cols]), rows, 0)
  ys = np.repeat(np.arange(rows).reshape([rows,1]), cols, 1)

  x_interp = xs[data]
  y_interp = ys[data]
  z_interp = region[data]

  x_holes = xs[holes]
  y_holes = ys[holes]

  print(x_interp.shape)
  print(y_interp.shape)
  print(z_interp.shape)

  z_cubic = interpolate.griddata((x_interp, y_interp), z_interp,
                                 (x_holes, y_holes) , method='cubic')

  # z_cubic = f_cubic(x_holes, y_holes)
  
  interped = np.zeros(data.shape)
  for i in range(len(z_cubic)):
    x = x_holes[i]
    y = y_holes[i]
    interped[y,x] = z_cubic[i]
  return interped

raw = np.loadtxt(open("wfov_unbinned.csv", "rb"), delimiter=',')
# img_raw = raw/np.max(raw)
# cv2.imshow('raw', img_raw)

trimmed = raw*((raw < 2210) * (raw > 250))
img_trimmed = (trimmed - 250)/(2210 - 250)
cv2.imshow('trimmed', img_trimmed)

flipped = flip_image(trimmed, 517) # best
avg = (flipped == 0)*trimmed + (trimmed == 0)*flipped + (flipped != 0)*(trimmed != 0) * (flipped + trimmed)/2
avg_img = (avg - 250)/(2210 - 250)
# cv2.imshow('symmetric average', avg_img)

avg_binary = (avg > 0).astype(np.float)
trim_binary = (trimmed > 0).astype(np.float)
# cv2.imshow('binary', avg_binary)

kernel = np.ones((5,5),np.uint8)
closing = cv2.morphologyEx(avg_binary,cv2.MORPH_CLOSE,kernel, iterations = 4)
# cv2.imshow('closing', closing)

opening = cv2.morphologyEx(closing,cv2.MORPH_OPEN,kernel, iterations = 4)
# cv2.imshow('opening', opening)

# data_limits = np.ones([1024, 1024])
# data_limits[750:870, 150:870] = 0
# data_limits[870:950, 250:780] = 0

mask = np.copy(opening).astype(np.bool)
holes = (avg == 0)*mask
data = (avg > 0)*mask

# filled = interpolate_region(region1, region1_data, region1_holes)
filled = interpolate_region(avg, data, holes)
result = np.clip((avg*mask + filled).astype(np.int), 0, 2210)
img_result = (result - 250) / (2210 - 250)

cv2.imshow('result', img_result)

cv2.waitKey()

np.savetxt('result.csv', result, delimiter=',')
