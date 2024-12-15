import cv2

# Generate a 6x6 marker with ID 0
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
marker_image = cv2.aruco.generateImageMarker(
    aruco_dict, 4, 200
)  # Marker ID 0, size 200x200 pixels

cv2.imshow("ArUco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
