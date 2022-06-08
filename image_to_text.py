import matplotlib.image as image
import numpy as np
import cv2

filename = 'thanks.png'
img = cv2.imread(filename,cv2.IMREAD_GRAYSCALE)
print('The Shape of the image is:', img.shape)
print
# apply threshold for rounding
threshold=0.1
for i in img:
    for j, value in enumerate(i):
        print(i[j])
        i[j]=round(value/255-threshold+0.5)


#convert to int
bw_image=img.astype(int)
print(bw_image[250])
# save to txt
np.savetxt('thanks.txt', bw_image, fmt='%1.0f', delimiter=',')
