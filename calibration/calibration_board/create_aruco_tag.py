import cv2 


markerLength = 0.45
# aruco_dict = cv2.aruco.Dictionary_get( cv2.aruco.DICT_4X4_1000 )
# arucoParams = cv2.aruco.DetectorParameters_create()
dictionary = cv2.aruco.getPredefinedDictionary( cv2.aruco.DICT_4X4_1000)
id_ = 0 
img = cv2.aruco.generateImageMarker(dictionary, id_,sidePixels=3300)
