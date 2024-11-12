from __future__ import division
import numpy as np

from control.controller import BaseController
from control.controller import compute_position_in_frame


class PIDController(BaseController):
    def __init__(self, **kwargs):
        self.kp = kwargs.pop("kp")
        self.kd = kwargs.pop("kd")

        # Get the keyword args that we didn't consume with the above initialization
        super(PIDController, self).__init__(**kwargs)


    def get_error(self, pose, reference_xytv):
        """Compute the PD error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: across-track and cross-track error
        """

        # TODO: calculate the across-track and cross-track error used for PID control.
        # The odometry provides the pose in terms of the lidar but for PID the pose
        # should be based on the back axle of the vehicle

        # First, calculate the pose of the axle from the pose of the lidar.
        # The heading should be the same but the position will be different
        # In sim: The back axle is 0.0125 m behind the lidar
        # On the real car: The back axle is 0.1286 m behind the lidar
        axle_pose0 = pose[0] - np.cos(pose[2]) * 0.1286
        axle_pose1 = pose[1] - np.sin(pose[2]) * 0.1286

        axle_pose = np.array([axle_pose0, axle_pose1, pose[2]])

        # Then calculate the error. For PID the error is calculated as the target position expressed in the vehicles frame
        error = compute_position_in_frame(
            p=reference_xytv[:3],
            frame=axle_pose
        )

        return error

    def get_control(self, pose, reference_xytv, error):
        """Compute the PD control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: python list of velocity and steering angle (not a np.array, just use [])
                (velocity should be copied from reference velocity)
        """

        # TODO: Calculate the output of the PD controller
        # NOTE: I think this is incorrect
        new_error = self.get_error(pose, reference_xytv)
        steer_angle = self.kp * error[1] + self.kd * (new_error[1] - error[1])

        return [reference_xytv[3], steer_angle]
    
def PD(self, pose, reference_xytv, error):
    # Calculate the error at time "t", NOTE: error is from time "t-1"
    new_error = self.get_error(pose, reference_xytv)
    # Calculate the steering angle using PD control
    steer_angle = self.kp * error[1] + self.kd * (new_error[1] - error[1])

    return [reference_xytv[3], steer_angle]
        
        

