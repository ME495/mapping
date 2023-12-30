import matplotlib.pyplot as plt
import cv2
import numpy as np

depth = cv2.imread('/home/lg/rosbags/map-1223-room_converted/depth/1703315766.280304.png',
                   cv2.IMREAD_ANYDEPTH).astype(np.float32)
depth /= depth.max()

rgb = cv2.imread('/home/lg/rosbags/map-1223-room_converted/rgb/1703315766.292211.png').astype(np.float32)
rgb /= 256.

depth = np.repeat(depth[:, :, np.newaxis], 3, axis=2)

img_show = np.concatenate((rgb, depth), axis=0)

dpi = 96
plt.figure(figsize=(img_show.shape[1]/dpi,img_show.shape[0]/dpi),dpi=dpi)
plt.imshow(img_show)
plt.show()