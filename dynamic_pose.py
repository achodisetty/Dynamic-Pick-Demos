import time 
import math

import numpy as np

import rtde_control
import humatics_milo as milo

np.set_printoptions(suppress=True)

# Defining the global variables
ur_base_quat_frame = [0, 0, 0, 0, 0, 0, 0]
transponder_quat_frame = [0, 0, 0, 0, 0, 0, 0]

ur_base_updated = False
transponder_updated = False


# The XPIDs of the transponders that make the pose
xpid_1 = milo.points.add(0x79897641)
xpid_2 = milo.points.add(0xc4cbfeeb)
xpid_3 = milo.points.add(0xe149f81b)

# The XPID of the transponder to be tracked
xpid_4 = milo.points.add(0x202d5e8e)

# Model of the pose
# model = [
#         (xpid_1, (0, 0.097, -0.008)),
#         (xpid_2, (0, -0.042, 0.095)),
#         (xpid_3, (0, -0.041, -0.09))
#     ]
model = [
        (xpid_1, (0, 0.476, 0.028)),
        (xpid_2, (0, 0.288, -0.086)),
        (xpid_3, (0, 0.282, 0.108))
    ]

# The IP address of the milo base station
base_ip = "192.168.1.5"

# The IP address of the arm
ur_ip = "192.168.1.199"

# Starting pose for the UR arm
initial_ur_joint_pos = [-0.348, 0.125, 0.12, 3.141, 0.0, 0.0]

# Initialize UR RTDE lib and moving the robot to a starting pose
ur = rtde_control.RTDEControlInterface(ur_ip)
ur.setTcp([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
ur.moveL(initial_ur_joint_pos, 0.25, 0.5, True)
time.sleep(1.0)
print('Initialization Complete!')


def frame_callback(data):
    if data.position_error_flags != 0 or \
        data.pose_error_flags != 0 or \
        ((data.position.x == 0) and \
        (data.position.y == 0) and \
        (data.position.z == 0)):
        return

    global ur_base_quat_frame, ur_base_updated
    ur_base_quat_frame[0] = data.position.x
    ur_base_quat_frame[1] = data.position.y
    ur_base_quat_frame[2] = data.position.z
    ur_base_quat_frame[3] = data.orientation.w
    ur_base_quat_frame[4] = data.orientation.x
    ur_base_quat_frame[5] = data.orientation.y
    ur_base_quat_frame[6] = data.orientation.z
    ur_base_updated = True


def point_callback(data):
    if data.error_flags != 0 or \
        ((data.position.x == 0) and \
        (data.position.y == 0) and \
        (data.position.z == 0)):
        return

    global transponder_updated, transponder_quat_frame
    transponder_quat_frame[0] = data.position.x
    transponder_quat_frame[1] = data.position.y
    transponder_quat_frame[2] = data.position.z
    transponder_updated = True


# A method to convert the quaternion to matrix
# The source of the math is here: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
# Calculates the translation matrix from the quaternion and return the 4x4 matrix
def quaternion_to_matrix (quaternion):

    sqw = quaternion[3] * quaternion[3]
    sqx = quaternion[4] * quaternion[4]
    sqy = quaternion[5] * quaternion[5]
    sqz = quaternion[6] * quaternion[6]
    
    invs = 1 / (sqx + sqy + sqz + sqw)
    m00 = ( sqx - sqy - sqz + sqw)*invs
    m11 = (-sqx + sqy - sqz + sqw)*invs
    m22 = (-sqx - sqy + sqz + sqw)*invs

    tmp1 = quaternion[4] * quaternion[5]
    tmp2 = quaternion[6] * quaternion[3]
    m10 = 2.0 * (tmp1 + tmp2) * invs
    m01 = 2.0 * (tmp1 - tmp2) * invs
    
    tmp1 = quaternion[4] * quaternion[6]
    tmp2 = quaternion[5] * quaternion[3]
    m20 = 2.0 * (tmp1 - tmp2) * invs
    m02 = 2.0 * (tmp1 + tmp2) * invs
    tmp1 = quaternion[5] * quaternion[6]
    tmp2 = quaternion[4] * quaternion[3]
    m21 = 2.0 * (tmp1 + tmp2) * invs
    m12 = 2.0 * (tmp1 - tmp2) * invs 
    
    translation_matrix = [
        [m00, m01, m02, quaternion[0]],
        [m10, m11, m12, quaternion[1]],
        [m20, m21, m22, quaternion[2]],
        [0, 0, 0, 1]
    ]

    return translation_matrix
    

# A method to convert the matrix to quaternion
# The source of the math is here: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
# Calculates the quaternion from the translation matrix and returns a 1x7 vecotor in the form: [x, y, z, W, Rz, Ry, Rz]
def matrix_to_quaternion (matrix):
    
    tr = matrix[0][0] + matrix[1][1] + matrix[2][2]

    if (tr > 0):
        S = np.math.sqrt(tr+1.0) * 2
        qw = 0.25 * S
        qx = (matrix[2][1] - matrix[1][2]) / S
        qy = (matrix[0][2] - matrix[2][0]) / S 
        qz = (matrix[1][0] - matrix[0][1]) / S 
    elif ((matrix[0][0] > matrix[1][1])&(matrix[0][0] > matrix[2][2])):
        S = np.math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2 
        qw = (matrix[2][1] - matrix[1][2]) / S
        qx = 0.25 * S
        qy = (matrix[0][1] + matrix[1][0]) / S 
        qz = (matrix[0][2] + matrix[2][0]) / S 
    elif (matrix[1][1] > matrix[2][2]):
        S = np.math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2
        qw = (matrix[0][2] - matrix[2][0]) / S
        qx = (matrix[0][1] + matrix[1][0]) / S 
        qy = 0.25 * S
        qz = (matrix[1][2] + matrix[2][1]) / S 
    else:
        S = np.math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2
        qw = (matrix[1][0] - matrix[0][1]) / S
        qx = (matrix[0][2] + matrix[2][0]) / S
        qy = (matrix[1][2] + matrix[2][1]) / S
        qz = 0.25 * S

    pose = [matrix[0][3], matrix[1][3], matrix[2][3], qw, qx, qy, qz]
    return pose


# Method to convert the transponder from the Milo frame to UR base frame
# Returns the converted frame in quaternion format
def frame_conversion():

    global ur_base_quat_frame, transponder_quat_frame

    current_ur_frame = ur_base_quat_frame
    current_transponder_frame = [transponder_quat_frame[0], transponder_quat_frame[1],
    transponder_quat_frame[2], current_ur_frame[3], current_ur_frame[4], current_ur_frame[5],
    current_ur_frame[6]]

    ur_base_matrix = quaternion_to_matrix(current_ur_frame)
    transponder_matrix = quaternion_to_matrix(current_transponder_frame)

    transformed_matrix = np.matmul(np.linalg.pinv(ur_base_matrix), transponder_matrix)
    transformed_frame = matrix_to_quaternion(transformed_matrix)

    return transformed_frame

# Method to convert the frame in quternion format to axis angle format
# Returns a vector that can be fed to the moveL function
def quat2axisAngle(frame):

    frame_modified = [0, 0, 0, 0, 0, 0, 0]
    if (frame[3] > 1):
        mag = np.sqrt(frame[3] * frame[3] + frame[4] * frame[4] + frame[5] * frame[5] + frame[6] * frame[6])
        frame_modified = [frame[0], frame[1], frame[2], frame[3] / mag, frame[4] / mag, frame[5] / mag, frame[6] / mag]
    else:
        frame_modified = frame
    
    angle = 2 * math.acos(frame_modified[3])
    den = math.sqrt(1 - frame_modified[3] * frame_modified[3])
    v = [frame[0], frame[1], frame[2], 0, 0, 0]

    if (den < 0.001):
        v[3] = angle * frame_modified[4]
        v[4] = angle * frame_modified[5]
        v[5] = angle * frame_modified[6]
    else:
        v[3] = angle * frame_modified[4] / den
        v[4] = angle * frame_modified[5] / den
        v[5] = angle * frame_modified[6] / den

    return v

if __name__ == '__main__':

    # Defining the base station frame
    bs_frame = milo.frames.get_root_frame()

    # connect to the base station: login, get base station ID, and set up Position API UDP subscription
    milo.base_stations.add(base_ip, "admin", "admin", bs_frame)

    # Adding the pose from the model
    pose = milo.poses.add(model, bs_frame)

    # Output for the pose/ UR base
    output_pose = milo.outputs.add_event_callback(frame_callback)
    output_pose.subscribe_to_pose(pose, bs_frame)

    while not ur_base_updated:
        print('Waiting on Pose...')
        time.sleep(1)

    # Output for the transponder
    output_xp = milo.outputs.add_event_callback(point_callback)
    output_xp.subscribe_to_point(xpid_4, bs_frame)

    while not transponder_updated:
        print('Waiting on transponder...')
        time.sleep(1)

    cur_pos = ur.getForwardKinematics()

    while True:

        if ur_base_updated or transponder_updated:
            transponder_in_urFrame = frame_conversion()
            transponder_in_urFrame_converted = quat2axisAngle(transponder_in_urFrame)
            print(f'The transponder in UR frame: {transponder_in_urFrame_converted}')
            transponder_in_urFrame_converted[3] = cur_pos[3]
            transponder_in_urFrame_converted[4] = cur_pos[4]
            transponder_in_urFrame_converted[5] = cur_pos[5]
            
            transponder_in_urFrame_convertedAxis = list(transponder_in_urFrame_converted)
            transponder_in_urFrame_convertedAxis[0] = transponder_in_urFrame_converted[1]
            transponder_in_urFrame_convertedAxis[1] = transponder_in_urFrame_converted[2]
            transponder_in_urFrame_convertedAxis[2] = transponder_in_urFrame_converted[0]

            ur.moveL(transponder_in_urFrame_convertedAxis, 0.25, 0.5, False)
            time.sleep(0.5)

            ur_base_updated = False
            transponder_updated = False
        time.sleep(0.1)