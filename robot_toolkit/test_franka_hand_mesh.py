import open3d as o3d 

def main():
    object_mesh = o3d.io.read_triangle_mesh("robot_arm_algos/src/inference/meshes/collision/hand.obj")
    finger_mesh = o3d.io.read_triangle_mesh("robot_arm_algos/src/inference/meshes/collision/finger.obj")
    o3d.visualization.draw_geometries([ object_mesh, finger_mesh])
    
if __name__ == "__main__":
    main()   