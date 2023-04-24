import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth
import cv2
import open3d as o3d
from pathlib import Path

import OpenEXR
import Imath


if __name__ == '__main__':

    K = np.array([[410.0, 0.0, 320.5],
                 [0.0, 410.0, 240.0],
                 [0.0, 0.0, 1.0]])

    # read the EXR file
    exr_file = OpenEXR.InputFile('/home/colin/Desktop/depthsegmentation/input/01396.exr')
    exr_header = exr_file.header()
    data_window = exr_header['dataWindow']
    width = data_window.max.x - data_window.min.x + 1
    height = data_window.max.y - data_window.min.y + 1
    float_type = Imath.PixelType(Imath.PixelType.FLOAT)
    img_str = exr_file.channel('Y', float_type)
    depth_image = np.frombuffer(img_str, dtype=np.float32).reshape(height, width)
    depth_image_norm = ((depth_image - np.min(depth_image)) / np.max(depth_image) * 255).astype(np.uint8)

    # Load the depth image and convert to point cloud
    rows, cols = depth_image.shape
    z = depth_image.flatten()
    x, y = np.meshgrid(np.arange(cols), np.arange(rows))
    x = (x.flatten() - K[0,2]) / K[0,0]
    y = (y.flatten() - K[1,2]) / K[1,1]
    x = x * z
    y = y * z
    point_cloud = np.vstack((x, y, z)).T

    # Estimate the bandwidth parameter for mean shift clustering
    # bandwidth = estimate_bandwidth(point_cloud, quantile=0.1)

    # Apply mean shift clustering to the point cloud
    bandwidth = 0.15
    ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
    ms.fit(point_cloud)
    labels = ms.labels_

    # Display the segmentation result
    result = np.zeros_like(depth_image, dtype=np.uint8)
    for i in range(np.max(labels) + 1):
        mask = labels == i
        mask = mask.reshape((np.shape(result)))
        result[mask] = (i + 1) * 255 // (np.max(labels) + 1)
    
    resultFolder = 'result/'
    # Define directory to save images
    output_dir_tf = Path().absolute() / resultFolder
    output_dir_tf.mkdir(exist_ok=True, parents=True)

    fileName = 'meanshift_' + str(bandwidth)
    fileName = fileName.replace('.', '_')
    fileName += '.png'

    cv2.imwrite(resultFolder + fileName, result)
    cv2.imwrite(resultFolder + 'normed_depthImage.png', depth_image_norm)
    cv2.imshow('Result', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()





















# import cv2
# import numpy as np
# import matplotlib.pyplot as plt

# import OpenEXR
# import Imath

# if __name__ == '__main__':

#     # read the EXR file
#     exr_file = OpenEXR.InputFile('/home/colin/Desktop/depthsegmentation/01396.exr')
#     exr_header = exr_file.header()
#     data_window = exr_header['dataWindow']
#     width = data_window.max.x - data_window.min.x + 1
#     height = data_window.max.y - data_window.min.y + 1
#     float_type = Imath.PixelType(Imath.PixelType.FLOAT)
#     img_str = exr_file.channel('Y', float_type)
#     image = np.frombuffer(img_str, dtype=np.float32).reshape(height, width)

#     # image = (( image - np.min(image)) / np.max(image) * 255)

#     # create a list of pixel coordinates
#     pixel_coords = np.array([(x, y) for x in range(width) for y in range(height)], dtype=np.float32)

#     # reshape the image to a 2D array of pixels and 3 color values (RGB)
#     pixel_values = image.reshape((-1, 1))
#     # convert to float
#     pixel_values = np.float32(pixel_values)
#     # concatenate pixel coordinates to pixel values
#     pixel_values = np.concatenate((pixel_values, pixel_coords), axis=1)
    




#     # # reshape the image to a 2D array of pixels and 3 color values (RGB)
#     # pixel_values = image.reshape((-1, 1))
#     # # convert to float
#     # pixel_values = np.float32(pixel_values)

#     # define stopping criteria
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10000, 0.001)

#     # number of clusters (K)
#     k = 4
#     compactness, labels, (centers) = cv2.kmeans(pixel_values, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

#     # convert back to 8 bit values
#     centers = np.uint8(centers)

#     # flatten the labels array
#     labels = labels.flatten()

#     # convert all pixels to the color of the centroids
#     segmented_image = centers[labels][:,0]

#     # reshape back to the original image dimension
#     segmented_image = segmented_image.reshape(image.shape)

#     # show the image
#     cv2.imshow('Original Image', image)
#     cv2.imshow('Segmented Image', segmented_image)


#     # disable only the cluster number 2 (turn the pixel into black)
#     masked_image = np.copy(image)
#     # convert to the shape of a vector of pixel values
#     masked_image = masked_image.reshape((-1, 1))
#     # color (i.e cluster) to disable
#     cluster = 2
#     masked_image[labels == cluster] = [0]

#     # convert back to original shape
#     masked_image = masked_image.reshape(image.shape)

#     segmented_image = (segmented_image - np.min(segmented_image)) / np.max(segmented_image) * 255

#     # show the image
#     cv2.imshow('Masked Image',masked_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

















# import cv2
# import numpy as np
# import OpenEXR
# import Imath
# import numpy as np

# # read the EXR file
# exr_file = OpenEXR.InputFile('/home/colin/Desktop/depthsegmentation/01396.exr')
# exr_header = exr_file.header()
# data_window = exr_header['dataWindow']
# width = data_window.max.x - data_window.min.x + 1
# height = data_window.max.y - data_window.min.y + 1
# float_type = Imath.PixelType(Imath.PixelType.FLOAT)
# img_str = exr_file.channel('Y', float_type)
# img = np.frombuffer(img_str, dtype=np.float32).reshape(height, width)


# # compute depth
# depth = img.copy()#[:,:] #.astype(float)
# depth[depth == 255] = -100

# # compute depth gradients
# dx, dy = np.gradient(depth)

# # consider only significant gradient
# bmsk = np.sqrt(dx**2 + dy**2) > 5

# # use morphological operations to "complete" the contours around the cars
# kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
# bmsk = cv2.dilate(bmsk.astype(np.uint8), kernel)
# bmsk = cv2.ximgproc.thinning(bmsk.astype(np.uint8))

# # once the contours are complete, use connected components
# _, cars = cv2.threshold(bmsk.astype(np.uint8), 0, 255, cv2.THRESH_BINARY_INV)

# cars = cv2.connectedComponents(cars)[1]
# cars = cars.astype(np.uint8)

# # display the results
# # for st in cv2.connectedComponentsWithStats(~cars):
# #     bbox = st[2]
# #     if st[0] > 0 and st[2][4] > 200:
# #         cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 255, 0), 3)

# cv2.imshow('Result', img)
# cv2.imshow('Mask', bmsk)
# cv2.imshow('Cars', cars)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


# # import cv2
# # import numpy as np
# # import OpenEXR
# # import Imath
# # import numpy as np

# # # read the EXR file
# # exr_file = OpenEXR.InputFile('/home/colin/Desktop/depthsegmentation/01396.exr')
# # exr_header = exr_file.header()
# # data_window = exr_header['dataWindow']
# # width = data_window.max.x - data_window.min.x + 1
# # height = data_window.max.y - data_window.min.y + 1
# # float_type = Imath.PixelType(Imath.PixelType.FLOAT)
# # img_str = exr_file.channel('Y', float_type)
# # img = np.frombuffer(img_str, dtype=np.float32).reshape(height, width)

# # img = (( img - np.min(img)) / np.max(img) * 255).astype(np.uint8)


# # # read the input image
# # # img = cv2.imread('/home/colin/Desktop/depthsegmentation/01396.exr', cv2.IMREAD_GRAYSCALE)

# # # threshold the image to create a binary mask
# # thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
# # thresh = thresh[1]

# # # convert the image to uint8 format
# # mask = cv2.convertScaleAbs(thresh)

# # # perform connected components analysis
# # n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

# # # display the results
# # cv2.imshow('Input', img)
# # cv2.imshow('Mask', mask)
# # cv2.imshow('Labels', np.uint8(labels / n_labels * 255))
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()