#!/usr/bin/env python

import numpy as np

class QuinticPolynomial:

    def __init__(self, init_pos, init_vel, init_accel, final_pos, final_vel, final_accel, dist):

        # Derived coefficients
        self.a_0 = init_pos
        self.a_1 = init_vel
        self.a_2 = init_accel / 2.0

        # Solve the linear equation (Ax = B)
        A = np.array([[dist ** 3,       dist ** 4,        dist ** 5], 
                     [3 * dist ** 2,   4 * dist ** 3,    5 * dist ** 4],
                     [6 * dist,        12 * dist ** 2,   20 * dist ** 3]])

        B = np.array([final_pos - self.a_0 - self.a_1 * dist - self.a_2 * dist ** 2, final_vel - self.a_1 - init_accel * dist, final_accel - init_accel])

        x = np.linalg.solve(A, B)

        self.a_3 = x[0]
        self.a_4 = x[1]
        self.a_5 = x[2]

    def calc_point(self, s):

        xs = self.a_0 + self.a_1 * s + self.a_2 * s ** 2 + self.a_3 * s ** 3 + self.a_4 * s ** 4 + self.a_5 * s ** 5

        return xs

    def calc_first_derivative(self, s):

        xs = self.a_1 + 2 * self.a_2 * s + 3 * self.a_3 * s ** 2 + 4 * self.a_4 * s ** 3 + 5 * self.a_5 * s ** 4

        return xs

    def calc_second_derivative(self, s):

        xs = 2 * self.a_2 + 6 * self.a_3 * s + 12 * self.a_4 * s ** 2 + 20 * self.a_5 * s ** 3

        return xs

    def calc_third_derivative(self, s):

        xs = 6 * self.a_3 + 24 * self.a_4 * s + 60 * self.a_5 * s ** 2

        return xs

class QuarticPolynomial:

    def __init__(self, init_pos, init_vel, init_accel, final_vel, final_accel, dist):

        self.a_0 = init_pos
        self.a_1 = init_vel
        self.a_2 = init_accel / 2.0

        A = np.array([[3 * dist ** 2,  4 * dist ** 3],
                      [6 * dist,       12 * dist ** 2]])

        B = np.array([final_vel - self.a_1 - init_accel * dist,
                      final_accel - init_accel])

        x = np.linalg.solve(A, B)

        self.a_3 = x[0]
        self.a_4 = x[1]

    def calc_point(self, s):

        xs = self.a_0 + self.a_1 * s + self.a_2 * s ** 2 + self.a_3 * s ** 3 + self.a_4 * s ** 4

        return xs

    def calc_first_derivative(self, s):

        xs = self.a_1 + 2 * self.a_2 * s + 3 * self.a_3 * s ** 2 + 4 * self.a_4 * s ** 3

        return xs

    def calc_second_derivative(self, s):

        xs = 2 * self.a_2 + 6 * self.a_3 * s + 12 * self.a_4 * s ** 2

        return xs

    def calc_third_derivative(self, s):

        xs = 6 * self.a_3 + 24 * self.a_4 * s

        return xs

class CubicPolynomial:

    def __init__(self, init_pos, init_vel, init_accel):
        
        # Derived coefficients
        self.d = init_pos
        self.c = init_vel
        self.b = init_accel / 2.0

def quintic_polynomial_planner(x_i, y_i, yaw_i, v_i, a_i, x_f, y_f, yaw_f, v_f, a_f, max_accel, max_jerk, ds):
    
    # Calculate the velocity boundary conditions based on vehicle's orientation
    v_xi = v_i * np.cos(yaw_i)
    v_xf = v_f * np.cos(yaw_f)
    v_yi = v_i * np.sin(yaw_i)
    v_yf = v_f * np.sin(yaw_f)

    # Calculate the acceleration boundary conditions based on vehicle's orientation
    a_xi = a_i * np.cos(yaw_i)
    a_xf = a_f * np.cos(yaw_f)
    a_yi = a_i * np.sin(yaw_i)
    a_yf = a_f * np.sin(yaw_f)

    for S in np.arange(1.0, 1000.0, 1.0):
        # Initialise the class
        xqp = QuinticPolynomial(x_i, v_xi, a_xi, x_f, v_xf, a_xf, S)
        yqp = QuinticPolynomial(y_i, v_yi, a_yi, y_f, v_yf, a_yf, S)

        # Instantiate/clear the arrays
        x = []
        y = []
        v = []
        a = []
        j = []
        yaw = []

        for s in np.arange(0.0, S + ds, ds):
            # Solve for position (m)
            x.append(xqp.calc_point(s))
            y.append(yqp.calc_point(s))

            # Solve for velocity (m/s)
            v_x = xqp.calc_first_derivative(s)
            v_y = yqp.calc_first_derivative(s)
            v.append(np.hypot(v_x, v_y))

            # Solve for orientation (rad)
            yaw.append(np.arctan2(v_y, v_x))

            # Solve for acceleration (m/s^2)
            a_x = xqp.calc_second_derivative(s)
            a_y = yqp.calc_second_derivative(s)

            if len(v) >= 2 and v[-1] - v[-2] < 0.0:
                a.append(-1.0 * np.hypot(a_x, a_y))

            else:
                a.append(np.hypot(a_x, a_y))
        
            # Solve for jerk (m/s^3)
            j_x = xqp.calc_third_derivative(s)
            j_y = yqp.calc_third_derivative(s)
            
            if len(a) >= 2 and a[-1] - a[-2] < 0.0:
                j.append(-1 * np.hypot(j_x, j_y))

            else:
                j.append(np.hypot(j_x, j_y))

        if max([abs(i) for i in a]) <= max_accel and max([abs(i) for i in j]) <= max_jerk:
            break

    return x, y, v, a, j, yaw

def quartic_polynomial_planner():
    pass

def cubic_polynomial_planner():
    pass

def plot():
    '''
    This function is for debugging and understanding purposes. Comment it out in main() if used as a library instead.
    '''
    # Initial values
    x_i = -10.0
    y_i = 10.0
    yaw_i = np.deg2rad(10.0)
    v_i = 1.0
    a_i = 0.1
    
    # Final values
    x_f = 30.0
    y_f = -10.0
    yaw_f = np.deg2rad(20.0)
    v_f = 0.0
    a_f = 0.1

    # Limits (Source: http://www.diva-portal.org/smash/get/diva2:839140/FULLTEXT01.pdf [Pg. 6])
    max_accel = 2.0
    max_jerk = 0.9
    ds = 0.1
    limits = max(abs(x_i), abs(x_f), abs(y_i), abs(y_f)) + 10.0

    x, y, v, a, j, yaw = quintic_polynomial_planner(x_i, y_i, yaw_i, v_i, a_i, x_f, y_f, yaw_f, v_f, a_f, max_accel, max_jerk, ds)

    print('no. of cells in x: {}'.format(len(x)))
    print('no. of cells in y: {}'.format(len(y)))

    import matplotlib.pyplot as plt

    plt.style.use('fivethirtyeight')
    plt.plot(x, y, label='Path', marker='.')
    plt.xlim(-limits, limits)
    plt.ylim(-limits, limits)
    plt.title('2D Path Trajectory')
    plt.xlabel('Horizontal Position (m)')
    plt.ylabel('Vertical Position (m)')
    plt.legend()
    plt.tight_layout()
    plt.show()

def main():

    plot()

if __name__ == '__main__':
    main()