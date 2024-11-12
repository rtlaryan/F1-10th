from __future__ import division
import numpy as np

from control.controller import BaseController
from control.controller import compute_position_in_frame


class StanleyController(BaseController):
    def __init__(self, **kwargs):
        self.ke = kwargs.pop("ke")
        self.kv = kwargs.pop("kv")

        # Get the keyword args that we didn't consume with the above initialization
        super(StanleyController, self).__init__(**kwargs)


    def get_error(self, pose, reference_xytv):
        """Compute the Stanley error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: Stanley error
        """
        # TODO: calculate the across-track and cross-track error used for stanley.
        # The odometry provides the pose in terms of the lidar but for stanley the pose
        # should be based on the front axle of the vehicle

        # First, calculate the pose of the axle from the pose of the lidar.
        # The heading should be the same but the position will be different
        # In sim: The front axle is 0.3177 m in front of the lidar
        # On the real car: The front axle is 0.107 m in front of the lidar

        axle_pose0 = pose[0] + np.cos(pose[2]) * 0.107
        axle_pose1 = pose[1] + np.sin(pose[2]) * 0.107

        axle_pose = np.array([axle_pose0, axle_pose1, pose[2]])

        # Then calculate the error. For PID the error is calculated as the target position expressed in the vehicles frame
        error = compute_position_in_frame(
            p=reference_xytv[:3],
            frame=axle_pose
        )

        return error

    def get_control(self, pose, reference_xytv, error):
        """Compute the Stanley control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: python list of velocity and steering angle (not a np.array, just use [])
        """

        # TODO: calculate the output of the stanley controller
        # Stanley typically calculates the curvature (tangent angle) of the trajectory
        # by finding the angle between two points on the trajectory but the controller only provides
        # the reference point so instead use the theta provided by the reference. This is usually effectively the same
        # unless the particle filter on the real car drifts which can lead to the trajectory moving "sideways" relative
        # to the angle encoded in the theta value.

        heading_error = reference_xytv[2] - pose[2]

        steer_angle = heading_error + np.arctan(((self.ke)*error[1])/(reference_xytv[3]+self.kv))
        steer_angle = np.arctan2(np.sin(steer_angle), np.cos(steer_angle))

        # while abs(steer_angle) - np.pi > 0:
        #     if steer_angle > 0:
        #         steer_angle -= 2*np.pi

        #     elif steer_angle < 0:
        #         steer_angle += 2*np.pi

        print("corrected angle:", steer_angle)

        return [reference_xytv[3], steer_angle]
