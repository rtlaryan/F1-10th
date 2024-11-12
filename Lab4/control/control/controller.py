from __future__ import division
import numpy as np
import threading
import time


def compute_position_in_frame(p, frame):
    """Compute the position in a new coordinate frame.

    Args:
        p: [x, y, heading]
        frame: [x, y, heading]

    Returns:
        error: the position components of p expressed in the provided frame in the format [e_x, e_y]
    """
    # TODO: calculate the position of the pose "p" in the coordinate frame "frame"

    # Extract the position components of p and frame
    p_x, p_y, p_heading = p
    frame_x, frame_y, frame_heading = frame
    rot_angle = frame_heading #NOTE: Is the subtraction order correct?

    # Create the rotation matrix
    rotation_matrix = np.array([[np.cos(rot_angle), np.sin(rot_angle)],
                                [-np.sin(rot_angle), np.cos(rot_angle)]])

    # Create the position vector
    position_vector = np.array([[p_x - frame_x], [p_y - frame_y]])

    # Calculate the relative position of p with respect to frame using matrix multiplication
    relative_position = rotation_matrix @ position_vector

    # Extract the relative x and y components
    relative_x, relative_y = relative_position.flatten()

    # Return the relative position as [e_x, e_y]
    return [relative_x, relative_y]


class BaseController(object):
    def __init__(self, **kwargs):
        self.__properties = {
            "finish_threshold",
            "exceed_threshold",
            "distance_lookahead",
            "min_speed",
        }
        if not self.__properties == set(kwargs):
            raise ValueError(
                "Invalid keyword argument provided",
                set(kwargs).difference(self.__properties),
            )

        self.__dict__.update(kwargs)
        # Error is typically going to be near this distance lookahead parameter,
        # so if the exceed threshold is lower we'll just immediately error out
        assert self.distance_lookahead < self.exceed_threshold

        self.path = []
        self.real_path = []
        self.current_pose = []
        self.selected_pose = None
        self.completed = False
        self.distance_error = None

        self.last_reference = 0


    def get_reference_index(self, pose, path_xytv, distance_lookahead):
        """Return the index to the next control target on the reference path.

        To compute a reference state that is some lookahead distance away, we
        recommend first finding the state on the path that is closest to the
        current vehicle state. Then, step along the path's waypoints and select
        the index of the first state that is greater than the lookahead distance
        from the current state. (You can also look back one index to see which
        state is closer to the desired lookahead distance.)

        Note that this method must be computationally efficient, since it runs
        directly in the control loop. Vectorize where you can.

        Args:
            pose: current state of the vehicle [x, y, heading]
            path_xytv: np.array of states and speeds with shape L x 4
            distance_lookahead (float): lookahead distance

        Returns:
            index to the reference state on the path

        """
        # TODO: Return the index of the lookahead point. Start by finding the closest point 
        # on the trajectory then iterating forward until you find the first point
        # that is farther than the lookahead distance.

        # if self.last_reference >= len(path_xytv) - 1:
        #     return len(path_xytv) - 1

        # Calculate the distance between the pose and each point on the path
        distances = np.linalg.norm(path_xytv[:, :2] - pose[:2], axis=1).flatten()

        # Find the index of the closest point on the path
        closest_index = np.argmin(distances)

        # Find the index of the first point that is farther than the lookahead distance
        lookahead_index = np.where(distances[closest_index:] > distance_lookahead)

        # If there are no points farther than the lookahead distance, return the closest point
        if len(lookahead_index) == 0 or lookahead_index[0].size == 0:
            # self.last_reference = len(path_xytv) - 1
            return len(path_xytv) - 1
        
        # Return the index of the first point that is farther than the lookahead distance
        reference_point = lookahead_index[0][0] + closest_index
        # self.last_reference = reference_point
        return reference_point

    def get_error(self, pose, reference_xytv):
        """Compute the error vector.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: error vector
        """
        # Subclasses will override this method
        raise NotImplementedError

    def get_control(self, pose, reference_xytv, error):
        """Compute the control action.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: [velocity, steering angle]
        """
        # Subclasses will override this method
        raise NotImplementedError

    def get_controller_status(self, pose, distance_error, distance_lookahead):
        """Return whether the reference path has been completed and if we have exceeded the error threshold

        The path completes successfully if the reference index refers to the
        final point on the path, and when the error is below finish_threshold.

        The path completes unsuccessfully when the error is above
        exceed_threshold.

        Args:
            pose: current state of the vehicle [x, y, heading]
            error: error vector [e_x, e_y]

        Returns:
            complete: whether the robot has reached the end of the path
            errored: whether the robot has exceeded its error threshold
        """
        ref_is_path_end = self.get_reference_index(
            pose, self.path, distance_lookahead
        ) == (len(self.path) - 1)

        err_l2 = np.linalg.norm(distance_error)
        print('L2 norm of error: ', err_l2)
        within_error = err_l2 < self.finish_threshold
        beyond_exceed = err_l2 > self.exceed_threshold
        return (ref_is_path_end and within_error), beyond_exceed

    ################
    # Control Loop #
    ################

    def control_step(self):
        """Implement the control loop."""


        # Wait until we have new a valid path and know where we are
        if self.completed or len(self.current_pose) == 0 or len(self.path) == 0:
            return None

        # Calculate what point on the path to use as our current target
        index = self.get_reference_index(
            self.current_pose, self.path, self.distance_lookahead
        )

        self.selected_pose = self.path[index]

        # Calculate the distance from the target point to publish from the control node
        self.distance_error = np.linalg.norm(
            self.selected_pose[:2] - self.current_pose[:2]
        )

        error = self.get_error(self.current_pose, self.selected_pose)

        # Check if we have completed the path or exceeded our error threshold
        complete, errored = self.get_controller_status(
            self.current_pose, error, self.distance_lookahead
        )

        if complete:
            if not self.completed:
                print("Path Completed")
                self.completed = True

            return None

        self.completed = False

        self.real_path.append(self.current_pose)

        if errored:
            print("Exceeded error threshold")

        # Return our controller output
        return self.get_control(self.current_pose, self.selected_pose, error)

    def reset_state(self):
        """Reset the controller's internal state."""
        # Override in the child class and ensure that super is called
        self.selected_pose = None
        self.current_pose = []
        self.distance_error = None
        self.path = []
        self.real_path = []
        self.completed = False

    def set_path_with_speed(self, path, speed):
        """Set the reference path to track.

        This implicitly resets the internal state of the controller.
        """
        self.reset_state()
        self.path = time_parameterize_ramp_up_ramp_down(path, speed, self.min_speed)


def time_parameterize_ramp_up_ramp_down(path_xyt, speed, min_speed):
    """Parameterize a geometric path of states with a desired speed.

    Vehicles can't instantly reach the desired speed, so we need to ramp up to
    full speed and ramp down to 0 at the end of the path.

    Args:
        path_xyt: np.array of states with shape L x 3
        speed (double): desired speed

    Returns:
        path_xytv: np.array of states and speed with shape L x 4
    """
    # For paths with fewer waypoints than necessary to ramp up and ramp down,
    # just set the desired speed directly (with a non-zero final speed, to
    # guarantee forward movement).
    if path_xyt.shape[0] < 4:
        speeds = speed * np.ones(path_xyt.shape[0])
        speeds = np.maximum(speeds, min_speed)
        return np.hstack([path_xyt, speeds[:, np.newaxis]])

    ramp_distance = 0.5
    displacement_vectors = np.diff(path_xyt[:, [0, 1]], axis=0)
    displacements = np.linalg.norm(displacement_vectors, axis=1)
    cumulative_path_length = np.cumsum(displacements)
    ramp_mask = (cumulative_path_length < ramp_distance) | (
        displacements.sum() - ramp_distance < cumulative_path_length
    )
    # At least one step of slowdown on the front and back
    ramp_mask[[0, -1]] = True
    change_points = np.where(np.diff(ramp_mask))[0]
    speeds = np.interp(
        np.arange(len(path_xyt)) / len(path_xyt),
        [0.0, change_points[0] / len(path_xyt), change_points[1] / len(path_xyt), 1.0],
        [0, speed, speed, 0],
    )
    speeds = np.maximum(speeds, min_speed)
    return np.hstack([path_xyt, speeds[:, np.newaxis]])
