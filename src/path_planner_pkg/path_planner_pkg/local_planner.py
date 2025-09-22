import math

class LocalPlanner:
    """
    LocalPlanner class for calculating the best velocity command for a UAV.
    """

    def __init__(self):
        """
        Initialize the LocalPlanner.
        """
        # Define the search space for velocities
        self.linear_velocities = [0.5, 1.0, 1.5]
        self.angular_velocities = [-0.5, 0, 0.5]

        # Define weights for the scoring criteria
        self.path_following_weight = 1.0
        self.obstacle_avoidance_weight = 2.0

    def get_best_velocity(self, current_pose, current_velocity, global_path_segment, obstacles):
        """
        Get the best velocity command based on the current state and environment.

        Args:
            current_pose (PoseWithCovariance): The current pose of the UAV.
            current_velocity (tuple): (linear_velocity, angular_velocity) of the UAV.
            global_path_segment (list): A list of (x, y) tuples representing the next segment of the global path.
            obstacles (list): A list of (x, y) tuples representing obstacle positions.

        Returns:
            tuple: The best (linear_velocity, angular_velocity) pair.
        """
        best_score = -float('inf')
        best_velocity = (0, 0)

        for linear_velocity in self.linear_velocities:
            for angular_velocity in self.angular_velocities:
                # Predict the future position
                future_pose = self._predict_future_pose(current_pose, linear_velocity, angular_velocity)

                # Score the predicted trajectory
                score = self._score_trajectory(future_pose, global_path_segment, obstacles)

                # Update the best velocity if the current one is better
                if score > best_score:
                    best_score = score
                    best_velocity = (linear_velocity, angular_velocity)

        return best_velocity

    def _quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw).
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def _predict_future_pose(self, current_pose, linear_velocity, angular_velocity, dt=0.5):
        """
        Predict the future pose of the UAV given a velocity command.

        Args:
            current_pose (PoseStamped): The current pose of the UAV.
            linear_velocity (float): The linear velocity command.
            angular_velocity (float): The angular velocity command.
            dt (float): The time step for prediction.

        Returns:
            tuple: The predicted (x, y, theta) of the UAV.
        """
        x = current_pose.position.x
        y = current_pose.position.y
        orientation = current_pose.orientation
        _, _, theta = self._quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        x_new = x + linear_velocity * math.cos(theta) * dt
        y_new = y + linear_velocity * math.sin(theta) * dt
        theta_new = theta + angular_velocity * dt
        return (x_new, y_new, theta_new)

    def _score_trajectory(self, future_pose, global_path_segment, obstacles):
        """
        Score a predicted trajectory based on path following and obstacle avoidance.

        Args:
            future_pose (tuple): The predicted (x, y, theta) of the UAV.
            global_path_segment (list): The next segment of the global path.
            obstacles (list): A list of obstacle positions.

        Returns:
            float: The score of the trajectory.
        """
        path_score = self._calculate_path_following_score(future_pose, global_path_segment)
        obstacle_score = self._calculate_obstacle_avoidance_score(future_pose, obstacles)

        # Combine the scores using weights
        total_score = (self.path_following_weight * path_score +
                       self.obstacle_avoidance_weight * obstacle_score)

        return total_score

    def _calculate_path_following_score(self, future_pose, global_path_segment):
        """
        Calculate the path following score.

        Args:
            future_pose (tuple): The predicted (x, y, theta) of the UAV.
            global_path_segment (list): The next segment of the global path.

        Returns:
            float: The path following score.
        """
        if not global_path_segment:
            return 0.0

        next_waypoint = global_path_segment[0]
        distance_to_path = math.sqrt((future_pose[0] - next_waypoint[0])**2 +
                                     (future_pose[1] - next_waypoint[1])**2)

        # The score is inversely proportional to the distance
        return 1.0 / (1.0 + distance_to_path)

    def _calculate_obstacle_avoidance_score(self, future_pose, obstacles):
        """
        Calculate the obstacle avoidance score.

        Args:
            future_pose (tuple): The predicted (x, y, theta) of the UAV.
            obstacles (list): A list of obstacle positions.

        Returns:
            float: The obstacle avoidance score.
        """
        min_distance_to_obstacle = float('inf')

        if not obstacles:
            return 1.0  # No obstacles, so the score is high

        for obstacle in obstacles:
            distance = math.sqrt((future_pose[0] - obstacle[0])**2 +
                                 (future_pose[1] - obstacle[1])**2)
            if distance < min_distance_to_obstacle:
                min_distance_to_obstacle = distance

        # The score is proportional to the distance to the nearest obstacle
        return min_distance_to_obstacle
