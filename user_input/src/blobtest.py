# Standard imports
import cv2
import numpy as np;

# Read image
im = cv2.imread("blobtest.jpg", cv2.IMREAD_GRAYSCALE)

# Set up the detector with default parameters.
params = cv2.SimpleBlobDetector_Params()
print(dir(params))
detector = cv2.SimpleBlobDetector_create(params)


# Detect blobs.
keypoints = detector.detect(im)

keypoint = [keypoints[0]]
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)