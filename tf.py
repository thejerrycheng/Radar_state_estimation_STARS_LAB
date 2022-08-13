
import math
import matplotlib.pyplot as plt
import numpy as np

def angle_to_matrix(angle):

    matrix = [[math.cos(angle), -math.sin(angle)],
              [math.sin(angle), math.cos(angle)]]
    print(matrix)
    return matrix

def make_tf(rotation, translation):
    rows = 3
    cols = 3

    tf = [[0 for _ in range(cols)]] * rows
    tf[0] = [rotation[0][0], rotation[0][1], translation[0]]
    tf[1] = [rotation[1][0], rotation[1][1], translation[1]]
    tf[2] = [0, 0, 1]
    print(tf)

    return tf

def to_tf(x, y, angle):
    translation = [x,y]
    rotation = angle_to_matrix(angle)
    tf = make_tf(rotation, translation)
    print(np.array(tf))
    return np.array(tf)

def print_rotation_matrix(matrix):
    rotation = [[matrix[0][0], matrix[0][1]],
          [matrix[1][0], matrix[1][1]]]
    # print(rotation)
    return np.array(rotation)


def print_translation_matrix(matrix):
    translation = [[matrix[0][2]], [matrix[1][2]]]
    # print([matrix[0][2], matrix[1][2]])
    return np.array(translation)

def rotaion_matrix_to_angle(rotation):

    angle = math.acos(rotation[0][0])
    if angle > math.pi/2:
        return -angle
    else:
        return angle

if __name__ == '__main__':

    # define the variables:
    angle_R5_fm = 1.61642
    angle_R3_fl = 1.3938
    angle_R4_fr = 1.48319
    angle_R1_fl = 1.40565
    angle_R2_fr = 1.51327

    transform_R5_fm = [-0.106221, 0.109482]
    transform_R3_fl = [-0.1035707, -0.707823]
    transform_R4_fr = [-0.184478, 0.706248]
    transform_R1_fl = [-0.957704, 4.51128]
    transform_R2_fr = [0.158231, -4.78012]

    # tf_R5_fm = np.array(make_tf(angle_to_matrix(angle_R5_fm), transform_R5_fm))
    # tf_R3_fl = np.array(make_tf(angle_to_matrix(angle_R3_fl), transform_R3_fl))
    # tf_R4_fr = np.array(make_tf(angle_to_matrix(angle_R4_fr), transform_R4_fr))
    # tf_R1_fl = np.array(make_tf(angle_to_matrix(angle_R1_fl), transform_R1_fl))
    # tf_R2_fr = np.array(make_tf(angle_to_matrix(angle_R2_fr), transform_R2_fr))

    tf_fm_R5 = np.array(make_tf(angle_to_matrix(angle_R5_fm), transform_R5_fm))
    tf_fl_R3 = np.array(make_tf(angle_to_matrix(angle_R3_fl), transform_R3_fl))
    tf_fr_R4 = np.array(make_tf(angle_to_matrix(angle_R4_fr), transform_R4_fr))
    tf_fl_R1 = np.array(make_tf(angle_to_matrix(angle_R1_fl), transform_R1_fl))
    tf_fr_R2 = np.array(make_tf(angle_to_matrix(angle_R2_fr), transform_R2_fr))

    # tf_fl_fm = to_tf(-0.882, -1.290, -1.360)
    # tf_fr_fm = to_tf(-0.917, 1.042, 1.693)

    tf_fm_fl = to_tf(-0.882, -1.290, -1.360)
    tf_fm_fr = to_tf(-0.917, 1.042, 1.693)

    tf_fl_fm = np.linalg.inv(tf_fm_fl)
    tf_fr_fm = np.linalg.inv(tf_fm_fr)

    tf_R5_fm = np.linalg.inv(tf_fm_R5)
    tf_R3_fl = np.linalg.inv(tf_fl_R3)
    tf_R4_fr = np.linalg.inv(tf_fr_R4)
    tf_R1_fl = np.linalg.inv(tf_fl_R1)
    tf_R2_fr = np.linalg.inv(tf_fr_R2)

    tf_R3_R5 = tf_R3_fl * tf_fl_fm * tf_fm_R5
    tf_R1_R5 = tf_R1_fl * tf_fl_fm * tf_fm_R5
    tf_R2_R5 = tf_R2_fr * tf_fr_fm * tf_fm_R5
    tf_R4_R5 = tf_R4_fr * tf_fr_fm * tf_fm_R5

    print(tf_R1_R5)
    print(tf_R2_R5)
    print(tf_R3_R5)
    print(tf_R4_R5)

    print(rotaion_matrix_to_angle(tf_R1_R5))
    print(rotaion_matrix_to_angle(tf_R2_R5))
    print(rotaion_matrix_to_angle(tf_R3_R5))
    print(rotaion_matrix_to_angle(tf_R4_R5))

    print(print_translation_matrix(tf_R1_R5))
    print(print_translation_matrix(tf_R2_R5))
    print(print_translation_matrix(tf_R3_R5))
    print(print_translation_matrix(tf_R4_R5))

    print(tf_R4_R5[0][2])
    print('tf_fm_fl', tf_fm_fl)
    print('tf_fl_fm', tf_fl_fm)

    print('tf_R5_fm', tf_R5_fm)
    print('tf_fm_R5', tf_fm_R5)
    print('tf_R3_fl', tf_R3_fl)
    print('tf_fl_R3', tf_fl_R3)

# visualize the locations of the five radars
    plt.plot(tf_R1_R5[0][1], tf_R1_R5[0][2], 'o')
    plt.plot(tf_R2_R5[0][1], tf_R2_R5[0][2], 'o')
    plt.plot(tf_R3_R5[0][1], tf_R3_R5[0][2], 'o')
    plt.plot(tf_R4_R5[0][1], tf_R4_R5[0][2], 'o')
    plt.plot(0, 0, 'x')
    plt.xlabel('x_translation')
    plt.ylabel('y_translation')
    plt.show()

