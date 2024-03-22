import matplotlib.pyplot as plt
import numpy as np
import math


def get_lidar_data(filename):
    lidar_data = []
    with open(filename, "r") as file:
        for line in file:
            parts = line.split(';')
            pose = np.array([float(i) for i in parts[0].split(',')])
            data = np.array([float(i) for i in parts[1].split(',')])
            lidar_data.append((pose, data))
    return np.array(lidar_data, dtype=[('pose', np.float64, (3,)), ('data', np.float64, (len(data),))])


def convert_matrix(pos_x, pos_y, pos_angle, shift_x, shift_y, shift_angle):
    transform_matrix = np.array([
        [np.cos(pos_angle), -np.sin(pos_angle), pos_x],
        [np.sin(pos_angle), np.cos(pos_angle), pos_y],
        [0, 0, 1]
    ])
    shift_vector = np.array([[shift_x], [shift_y], [shift_angle]])
    return transform_matrix @ shift_vector


def draw_point(lidar_data):
    for data in lidar_data:
        pos_x = data[0][0]
        pos_y = data[0][1]
        pos_angle = data[0][2]
        plt.plot(pos_x, pos_y, 'ro')

        for i in range(0, len(data[1]), 4):
            shot = data[1][i]
            if shot != 5.6 and shot > 0.5:
                angle_lidar = np.deg2rad(135 - 270 / 681 * i)
                x_lidar = shot * math.cos(angle_lidar)
                y_lidar = shot * math.sin(angle_lidar)
                point_coordinates = convert_matrix(pos_x, pos_y, pos_angle, x_lidar, y_lidar, 1)
                plt.plot(point_coordinates[0], point_coordinates[1], 'b*')


if __name__ == '__main__':

    window = plt.figure()
    window.set_figwidth(15)
    window.set_figheight(10)
    window.suptitle('LiDAR Map')

    lidar_data = get_lidar_data('data.txt')
    draw_point(lidar_data)

    plt.show()
