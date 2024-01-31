import matplotlib.pyplot as plt
import cv2
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.camera.camera import get_bbox_annotations, get_segmap_from_bbox, get_segmap_from_bbox_with_depth

def main(): 
    rs_camera = RealSenseCamera() 
    rgb_image, depth_image = rs_camera.get_current_rgbd_frames() 
    bbox, cropped_image = get_bbox_annotations(rgb_image)
    rgb_image = cv2.rectangle(rgb_image, (bbox.xmin, bbox.ymax), (bbox.xmax, bbox.ymin), (0, 0, 0), 2) 
    plt.imshow(rgb_image)
    plt.show()        
    plt.imshow(cropped_image)
    plt.show() 
    segmap = get_segmap_from_bbox(rgb_image, bbox)
    plt.imshow(segmap)
    plt.show()    
    segmap = get_segmap_from_bbox_with_depth(rgb_image,depth_image, bbox)
    plt.imshow(segmap)
    plt.show()    
    
if __name__ == "__main__":
    main()    
        