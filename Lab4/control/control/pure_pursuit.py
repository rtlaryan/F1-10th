from __future__ import division
import numpy as np

from control.controller import BaseController
from control.controller import compute_position_in_frame


class PurePursuitController(BaseController):
    def __init__(self, **kwargs):
        #self.car_length = kwargs.pop("car_length")

        # Get the keyword args that we didn't consume with the above initialization
        super(PurePursuitController, self).__init__(**kwargs)


    def get_error(self, pose, reference_xytv):
        """Compute the Pure Pursuit error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: Pure Pursuit error
        """
        # TODO: calculate the across-track and cross-track error used for pure pursuit.
        # The odometry provides the pose in terms of the lidar but for pure pursuit the pose
        # should be based on the back axle of the vehicle

        # First, Calculate the pose of the axle from the pose of the lidar.
        # The heading should be the same but the position will be different
        # In sim: The back axle is 0.0125 m behind the lidar
        # On the real car: The back axle is 0.1286 m behind the lidar

        axle_pose0 = pose[0] + np.cos(pose[2]) * 0.1286
        axle_pose1 = pose[1] + np.sin(pose[2]) * 0.1286

        axle_pose = np.array([axle_pose0, axle_pose1, pose[2]])

        # Then calculate the error. For PID the error is calculated as the target position expressed in the vehicles frame
        error = compute_position_in_frame(
            p=reference_xytv[:3],
            frame=axle_pose
        )

        return error

    def get_control(self, pose, reference_xytv, error):
        """Compute the Pure Pursuit control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: python list of velocity and steering angle (not a np.array, just use [])
        """

        print('Cross track error:', error[1])
        steer_angle = ((2*error[1])/(self.distance_lookahead**2)) / 4 
        print('Original steering angle:', steer_angle)
        steer_angle = np.arctan2(np.sin(steer_angle), np.cos(steer_angle))
        
        # while abs(steer_angle) - np.pi > 0:
        #     if steer_angle > 0:
        #         steer_angle -= 2*np.pi

        #     elif steer_angle < 0:
        #         steer_angle += 2*np.pi

        print("Corrected steering angle:", steer_angle)

        return [reference_xytv[3], steer_angle]
