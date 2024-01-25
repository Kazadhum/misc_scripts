def main():
    import tf.transformations
    import numpy as np

    rotation_matrix = np.array([[-0.286486, -0.646401, -0.707171, 0],
                            [-0.958053, 0.18732, 0.2169, 0],
                            [-0.00462435, 0.751655, -0.65954, 0],
                            [0, 0, 0, 1]])

    fros_T_fmp3d = np.array([[1,0,0,0],
                        [0,-1,0,0],
                        [0,0,-1,0],
                        [0,0,0,1]])

    composed_rot = np.dot(rotation_matrix, fros_T_fmp3d)

    angles = tf.transformations.euler_from_matrix(composed_rot, axes = 'szyx')


    print(angles)

if __name__ == '__main__':
    main()