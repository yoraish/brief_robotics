import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl


aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
fig = plt.figure()
nx = 4
ny = 3
print('Here')

friends = cv2.imread('./friends.jpg', -1)
# print(friends.shape)

orig_img = np.zeros([200*16,200*11, 3]).astype(np.uint8)
orig_img.fill(255)

aruco_size = int(0.05*200)
pix_per_m = 200
count = 0
for i in range(16):
    for j in range(11):
        fimg = cv2.resize(friends, (pix_per_m, pix_per_m))
        orig_img[i*pix_per_m:(i+1)*pix_per_m, j*pix_per_m:(j+1)*pix_per_m] = fimg
        for k,l in zip([0.1,0.8,0.1,0.8],[0.1,0.8,0.8,0.1]):
            img = aruco.drawMarker(aruco_dict,count, int(aruco_size))
            x = int((i*pix_per_m) + (pix_per_m*k))
            y = int((j*pix_per_m) + (pix_per_m*l))
            orig_img[x:x+aruco_size, y:y+aruco_size] = np.stack([img, img, img], axis=-1)
            count += 1

cv2.imwrite('./aruco.png', orig_img)
plt.imshow(orig_img, cmap=mpl.cm.gray)
plt.show()
# for i in range(1, nx*ny+1):
#     ax = fig.add_subplot(ny,nx, i)
#     img = aruco.drawMarker(aruco_dict,i, 700)
#     plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
#     ax.axis("off")

# plt.show()