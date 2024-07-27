#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():
    def __init__(self, namespace):
        self.namespace = namespace

        # Publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher(f"{self.namespace}/command/motor_speed", Actuators, queue_size=10)

        # Subscriber to Odometry topic
        self.odom_sub = rospy.Subscriber(f"{self.namespace}/ground_truth/odometry", Odometry, self.odom_callback)

        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False

        global omega
        omega = 0

    def traj_coeff(self, t0, tf, pi, pf):
        # Calculating the coefficients for trajectory segment
        A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                      [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                      [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                      [1, tf, tf**2, tf**3, tf**4, tf**5],
                      [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                      [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

        # Velocity and Acceleration at each Waypoint is 0 i.e pi_dot, pi_ddot, pf_dot, pf_ddot = 0
        b = np.array([pi, 0, 0, pf, 0, 0])

        coeff = np.linalg.solve(A, b)

        return coeff

    def traj_evaluate(self):
        # Evaluate coefficients for each segment of the trajectory
        if self.t <= 5:  # P0 (2,0,0) to P1 (2,0,1)
            coeff_x = self.traj_coeff(0, 5, 2, 2)
            coeff_y = self.traj_coeff(0, 5, 0, 0)
            coeff_z = self.traj_coeff(0, 5, 0, 1)

        elif self.t <= 20:  # P1 (2,0,1) to P2 (3,0,1)
            coeff_x = self.traj_coeff(5, 20, 2, 3)
            coeff_y = self.traj_coeff(5, 20, 0, 0)
            coeff_z = self.traj_coeff(5, 20, 1, 1)

        elif self.t <= 35:  # P2 (3,0,1) to P3 (3,1,1)
            coeff_x = self.traj_coeff(20, 35, 3, 3)
            coeff_y = self.traj_coeff(20, 35, 0, 1)
            coeff_z = self.traj_coeff(20, 35, 1, 1)

        elif self.t <= 50:  # P3 (3,1,1) to P4 (2,1,1)
            coeff_x = self.traj_coeff(35, 50, 3, 2)
            coeff_y = self.traj_coeff(35, 50, 1, 1)
            coeff_z = self.traj_coeff(35, 50, 1, 1)

        elif self.t <= 65:  # P4 (2,1,1) to P5 (2,0,1)
            coeff_x = self.traj_coeff(50, 65, 2, 2)
            coeff_y = self.traj_coeff(50, 65, 1, 0)
            coeff_z = self.traj_coeff(50, 65, 1, 1)

        else:  # Zero input after 65 secs
            coeff_x = np.zeros(6)
            coeff_y = np.zeros(6)
            coeff_z = np.zeros(6)

        return np.array([coeff_x, coeff_y, coeff_z])

    def traj_gen(self):
        # Generate trajectory from coefficients and time matrix 'T'
        coeff = self.traj_evaluate()
        T = np.transpose(np.array([[1, self.t, self.t**2, self.t**3, self.t**4, self.t**5],
                                   [0, 1, 2*self.t, 3*self.t**2, 4*self.t**3, 5*self.t**4],
                                   [0, 0, 2, 6*self.t, 12*self.t**2, 20*self.t**3]]))

        traj_x = coeff[0] @ T
        xd = traj_x[0]
        xd_dot = traj_x[1]
        xd_ddot = traj_x[2]

        traj_y = coeff[1] @ T
        yd = traj_y[0]
        yd_dot = traj_y[1]
        yd_ddot = traj_y[2]

        traj_z = coeff[2] @ T
        zd = traj_z[0]
        zd_dot = traj_z[1]
        zd_ddot = traj_z[2]

        return xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, zd, zd_dot, zd_ddot

    def sign_function(self, S):
        # Sign function to restrict the value to -1, 0, 1 depending upon the value of S
        if S > 0:
            sgn = +1
        elif S < 0:
            sgn = -1
        else:
            sgn = 0
        return sgn

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # Obtain desired values by evaluating the corresponding trajectories
        xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, zd, zd_dot, zd_ddot = self.traj_gen()

        m = 27*(10**-3)
        l = 46*(10**-3)
        Ix = 16.57171*(10**-6)
        Iy = 16.57171*(10**-6)
        Iz = 29.261652*(10**-6)
        Ip = 12.65625 *(10**-8)
        kf = 1.28192*(10**-8)
        km = 5.964552*(10**-3)
        ak = 1/(4*kf)
        bk = np.sqrt(2)*ak/l
        g = 9.8

        global omega

        # Tuned Parameters
        Kp = 40
        Kd = 5.4
        K = [2, 140, 140, 5]
        lam = [2 , 12.5, 12.5, 5]

        # Current Position
        x = xyz[0, 0]
        y = xyz[1, 0]
        z = xyz[2, 0]

        # Current Velocity
        x_dot = xyz_dot[0, 0]
        y_dot = xyz_dot[1, 0]
        z_dot = xyz_dot[2, 0]

        # Current RPY angle
        phi = rpy[0, 0]
        theta = rpy[1, 0]
        shi = rpy[2, 0]

        # Current RPY velocity
        phi_dot = rpy_dot[0, 0]
        theta_dot = rpy_dot[1, 0]
        shi_dot = rpy_dot[2, 0]

        print("------------------------- Time: ", round(self.t, 2), "-------------------------------")

        # Z Input
        u1 = m * (zd_ddot + g - lam[0] * (z_dot - zd_dot)  - K[0] * self.sign_function((z_dot - zd_dot) + lam[0] * (z - zd)))

        # Calculating the force in x and y direction
        Fx = m*(-Kp*(x-xd) - Kd*(x_dot-xd_dot) + xd_ddot)
        Fy = m*(-Kp*(y-yd) - Kd*(y_dot-yd_dot) + yd_ddot)

        # Restricting the forces between (-1, 1) to resolve math domain error
        force_x = max(min(Fx / u1, 1), -1)
        force_y = max(min(Fy / u1, 1), -1)

        theta_d = np.arcsin(force_x)
        phi_d = np.arcsin(-force_y)

        # RPY Error Dynamics
        s_phi = (phi_dot - 0) + lam[1] * (phi - phi_d)
        s_theta = (theta_dot - 0) + lam[2] * (theta - theta_d)
        s_shi = (shi_dot - omega) + lam[3] * (shi - 0)

        # Converting nan values to zero
        phi_d = np.nan_to_num(phi_d)
        theta_d = np.nan_to_num(theta_d)

        # Input for angular roll, pitch, yaw
        u2 = Ix*(-0 - lam[1]*(phi - phi_d) - K[1] * self.sign_function(s_phi)) + (theta_dot * shi_dot * (Iy - Iz))
        u3 = Iy*(-0 - lam[2]*(theta - theta_d) - K[2] * self.sign_function(s_theta)) + (phi_dot * shi_dot * (Iz - Ix))
        u4 = Iz*(-0 - lam[3]*(shi - 0) - K[3] * self.sign_function(s_shi))

        print('phi_d:', round(phi_d, 2))
        print('theta_d:', round(theta_d, 2))

        # Individual rotor angular velocity square
        omega1 = np.sqrt(ak*u1 - bk*u3 - km*omega)
        omega2 = np.sqrt(ak*u1 - bk*u2 + km*omega)
        omega3 = np.sqrt(ak*u1 + bk*u3 - km*omega)
        omega4 = np.sqrt(ak*u1 + bk*u2 + km*omega)

        omega1 = min(omega1, 2200)
        omega2 = min(omega2, 2200)
        omega3 = min(omega3, 2200)
        omega4 = min(omega4, 2200)

        omega = np.array([omega1, omega2, omega3, omega4])
        omega = np.nan_to_num(omega)

        print('omega: ', np.round(omega, 2))

        motor_speed = Actuators()
        motor_speed.angular_velocities = omega

        self.motor_speed_pub.publish(motor_speed)

    def odom_callback(self, msg):
        # Initial position of the drone
        pos = msg.pose.pose.position
        x0 = pos.x
        y0 = pos.y
        z0 = pos.z

        # Initial orientation of the drone
        ori = msg.pose.pose.orientation
        (phi0, theta0, shi0) = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        # Current time
        if self.t0 is None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0

        # Current linear velocity of the drone
        x_dot = msg.twist.twist.linear.x
        y_dot = msg.twist.twist.linear.y
        z_dot = msg.twist.twist.linear.z

        # Current angular velocity of the drone
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        xyz = np.transpose(np.array([[x0, y0, z0]]))
        xyz_dot = np.transpose(np.array([[x_dot, y_dot, z_dot]]))
        rpy = np.transpose(np.array([[phi0, theta0, shi0]]))
        rpy_dot = np.transpose(np.array([[p, q, r]]))

        self.x_series.append(x0)
        self.y_series.append(y0)
        self.z_series.append(z0)
        self.t_series.append(self.t)

        if self.t > 65:
            # Save data as a pickle file
            self.save_pkl()
        else:
            self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

    def save_pkl(self):
        # Save the data to a pickle file
        if not self.mutex_lock_on:
            self.mutex_lock_on = True
            data = {
                'x': self.x_series,
                'y': self.y_series,
                'z': self.z_series,
                't': self.t_series,
            }
            dir_path = os.path.dirname(os.path.realpath(__file__))
            with open(os.path.join(dir_path, 'trajectory_data.pkl'), 'wb') as f:
                pickle.dump(data, f)

if __name__ == '__main__':
    rospy.init_node('quadrotor_trajectory_tracking')

    namespaces = ["crazyflie2_1", "crazyflie2_2", "crazyflie2_3", "crazyflie2_4"]
    quadrotors = [Quadrotor(ns) for ns in namespaces]

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
