import cv2

# Load an image with ArUco markers
image = cv2.imread("test_image.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Load the predefined dictionary and detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
aruco_params = cv2.aruco.DetectorParameters()

# Detect markers
corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

# Draw and show results
if ids is not None:
    print(f"Detected marker IDs: {ids.flatten()}")
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
else:
    print("No markers detected")

cv2.imshow("ArUco Detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
