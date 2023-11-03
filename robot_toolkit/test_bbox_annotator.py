import matplotlib.pyplot as plt
import cv2
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.camera.camera import get_bbox_annotations, get_segmap_from_bbox

def main(): 
    rs_camera = RealSenseCamera() 
    image = rs_camera.get_current_rgb_frame() 
    bbox, cropped_image = get_bbox_annotations(image)
    image = cv2.rectangle(image, (bbox.xmin, bbox.ymax), (bbox.xmax, bbox.ymin), (0, 0, 0), 2) 
    plt.imshow(image)
    plt.show()        
    plt.imshow(cropped_image)
    plt.show() 
    segmap = get_segmap_from_bbox(image, bbox)
    plt.imshow(segmap)
    plt.show()    
    
if __name__ == "__main__":
    main()    
        