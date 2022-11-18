def str2bool(value):
    if isinstance(value, bool):
        return value
    if value.lower() in {'false', 'f', '0', 'no', 'n'}:
        return False
    elif value.lower() in {'true', 't', '1', 'yes', 'y'}:
        return True
    raise ValueError(f'{value} is not a valid boolean value')

    
def point_to_object(obj_loc, distance, pan, yaw, rotate):
    r = R.from_rotvec(np.array([0, -pan, 0]))
    r1 = R.from_rotvec(np.array([0,0, yaw]))
    r2 = R.from_rotvec(np.array([0, pan, 0]))
    r3 = R.from_rotvec(np.array([np.pi, 0 ,0]))
    r4 = R.from_rotvec(np.array([0, 0, rotate]))
    translation = r1.as_matrix() @ r.as_matrix() @ np.array([distance, 0, 0])
    translation += obj_loc
    return translation, r1.as_matrix() @ r2.as_matrix() @ r3.as_matrix() @ r4.as_matrix() 


def write_to_file(line_list, file_name):
    # open file in append mode
    line  = ",".join(line_list)
    with open(file_name, 'a') as f:
        f.write(line)
        f.write('\n') 


# print(ee_rotation)
# rvec = cv2.Rodrigues(ee_rotation)[0]
# print(rvec)
# rotation_matrix = np.zeros(shape=(3,3))
# rotation_matrix = cv2.Rodrigues(rvec)
# print(rotation_matrix[0])        