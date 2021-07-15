from matplotlib.pyplot import step
from trajectory_generator import TrajectoryGenerator
from trajectory import Trajectory, TrajectoryPath, TrajectoryParameters
import numpy as np
from collections import defaultdict
from motion_model import MotionModel

class LatticeGenerator:

    def __init__(self, config):
        self.grid_separation = config["gridSeparation"]
        self.trajectory_generator = TrajectoryGenerator(config)
        self.turning_radius = config["turningRadius"]
        self.max_level = round(config["maxLength"] / self.grid_separation)
        self.number_of_headings = config["numberOfHeadings"]

        self.motion_model = MotionModel[config["motionModel"].upper()]
        
    def angle_difference(self, angle_1, angle_2):

        difference = abs(angle_1 - angle_2)
        
        if difference > np.pi:
            # If difference > 180 return the shorter distance between the angles
            difference = 2*np.pi - difference
        
        return difference

    def get_coords_at_level(self, level):
        positions = []

        max_point_coord = self.grid_separation * level

        for i in range(level):
            varying_point_coord = self.grid_separation * i

            # Varying y-coord
            positions.append((max_point_coord, varying_point_coord))
            
            # Varying x-coord
            positions.append((varying_point_coord, max_point_coord))


        # Append the corner
        positions.append((max_point_coord, max_point_coord))

        return np.array(positions)

    def get_heading_discretization(self):
        max_val = int((((self.number_of_headings + 4)/4) -1) / 2)

        outer_edge_x = []
        outer_edge_y = []

        for i in range(-max_val, max_val+1):
            outer_edge_x += [i, i]
            outer_edge_y += [-max_val, max_val]

            if i != max_val and i != -max_val:
                outer_edge_y += [i, i]
                outer_edge_x += [-max_val, max_val]

        return [np.rad2deg(np.arctan2(j, i)) for i, j in zip(outer_edge_x, outer_edge_y)]

    def point_to_line_distance(self, p1, p2, q):
        '''
        Return minimum distance from q to line segment defined by p1, p2.
            Projects q onto line segment p1, p2 and returns the distance
        '''

        # Get back the l2-norm without the square root
        l2 = np.inner(p1-p2, p1-p2)

        if l2 == 0:
            return np.linalg.norm(p1 - q)

        # Ensure t lies in [0, 1]
        t = max(0, min(1, np.dot(q - p1, p2 - p1) / l2))
        projected_point = p1 + t * (p2 - p1)

        return np.linalg.norm(q - projected_point)

    def is_minimal_path(self, trajectory_path: TrajectoryPath, minimal_spanning_trajectories):
    
        distance_threshold = 0.5 * self.grid_separation
        rotation_threshold = 0.5 * np.deg2rad(360 / self.number_of_headings)

        for x1, y1, x2, y2, yaw in zip(trajectory_path.xs[:-1], trajectory_path.ys[:-1], trajectory_path.xs[1:], trajectory_path.ys[1:], trajectory_path.yaws[:-1]):

            p1 = np.array([x1, y1])
            p2 = np.array([x2, y2])

            for prior_end_point in minimal_spanning_trajectories:

                # TODO: point_to_line_distance gives direct distance which means the distance_threshold represents a circle 
                # around each point. Change so that we calculate manhattan distance? <- d_t will represent a box instead
                if self.point_to_line_distance(p1, p2, prior_end_point[:-1]) < distance_threshold \
                    and self.angle_difference(yaw, prior_end_point[-1]) < rotation_threshold:
                    return False
        
        return True

    def compute_min_trajectory_length(self):
        # Compute arc length of circle that moves through an angle of 360/number of headings 
        return 2 * np.pi * self.turning_radius * (1/self.number_of_headings)

    def generate_minimal_spanning_set(self):
        single_quadrant_spanning_set = defaultdict(list)

        # Firstly generate the minimal set for a single quadrant
        single_quadrant_headings = self.get_heading_discretization()

        initial_headings = sorted(list(filter(lambda x: 0 <= x and x < 90, single_quadrant_headings)))

        min_trajectory_length = self.compute_min_trajectory_length()
        start_level = int(np.floor(min_trajectory_length / self.grid_separation))

        for start_heading in initial_headings:
            
            minimal_trajectory_set = []
            current_level = start_level

            # To get target headings: sort headings radially and remove those that are more than 90 degrees away
            target_headings = sorted(single_quadrant_headings, key=lambda x: (abs(x - start_heading),-x))
            target_headings = list(filter(lambda x : abs(start_heading - x) <= 90, target_headings))
                
            while current_level <= self.max_level:

                # Generate x,y coordinates for current level
                positions = self.get_coords_at_level(current_level)

                for target_point in positions:
                    for target_heading in target_headings:
                        # Use 10% of grid separation for finer granularity when checking if trajectory overlaps another already seen trajectory
                        trajectory = self.trajectory_generator.generate_trajectory(target_point, start_heading, target_heading, 0.1 * self.grid_separation)
                        

                        if trajectory:
                            # Check if path overlaps something in minimal spanning set
                            if(self.is_minimal_path(trajectory.path, minimal_trajectory_set)):
                                new_end_pose = np.array([target_point[0], target_point[1], np.deg2rad(target_heading)])
                                minimal_trajectory_set.append(new_end_pose)

                                single_quadrant_spanning_set[start_heading].append((target_point, target_heading))

                current_level += 1

        return self.create_complete_minimal_spanning_set(single_quadrant_spanning_set)

    def create_complete_minimal_spanning_set(self, single_quadrant_minimal_set):
        # Copy the 0 degree trajectories to 90 degerees
        end_points_for_90 = []

        for end_point, end_angle in single_quadrant_minimal_set[0.0]:
            x, y = end_point

            end_points_for_90.append((np.array([y, x]), 90 - end_angle))

        single_quadrant_minimal_set[90.0] = end_points_for_90

        # Generate the paths for all trajectories
        all_trajectories = defaultdict(list)
       
        for start_angle in single_quadrant_minimal_set.keys():

            for end_point, end_angle in single_quadrant_minimal_set[start_angle]:
                trajectory = self.trajectory_generator.generate_trajectory(end_point, start_angle, end_angle, self.grid_separation)

                xs = trajectory.path.xs
                ys = trajectory.path.ys

                flipped_xs = np.array([-x for x in xs])
                flipped_ys = np.array([-y for y in ys])

                yaws_quad1 = np.array([np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(xs[:-1], ys[:-1], xs[1:], ys[1:])] + [np.deg2rad(end_angle)])
                yaws_quad2 = np.array([np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(flipped_xs[:-1], ys[:-1], flipped_xs[1:], ys[1:])] + [np.pi - np.deg2rad(end_angle)])
                yaws_quad3 = np.array([np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(flipped_xs[:-1], flipped_ys[:-1], flipped_xs[1:], flipped_ys[1:])]  + [-np.pi + np.deg2rad(end_angle)])
                yaws_quad4 = np.array([np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(xs[:-1], flipped_ys[:-1], xs[1:], flipped_ys[1:])] + [-np.deg2rad(end_angle)])

                # Round position values
                xs = xs.round(5)
                ys = ys.round(5)
                flipped_xs = flipped_xs.round(5)
                flipped_ys = flipped_ys.round(5)

                # Hack to remove negative zeros
                xs += 0.
                ys += 0.
                flipped_xs += 0.
                flipped_ys += 0.
                yaws_quad1 += 0.
                yaws_quad2 += 0.
                yaws_quad3 += 0.
                yaws_quad4 += 0. 

                arc_length = 2 * np.pi * trajectory.parameters.radius * abs(start_angle - end_angle) / 360.0
                straight_length = trajectory.parameters.start_to_arc_distance + trajectory.parameters.arc_to_end_distance
                trajectory_length = arc_length + trajectory.parameters.start_to_arc_distance + trajectory.parameters.arc_to_end_distance

                trajectory_info = (trajectory.parameters.radius, trajectory_length, arc_length, straight_length)

                '''
                Quadrant 1: +x, +y
                Quadrant 2: -x, +y
                Quadrant 3: -x, -y
                Quadrant 4: +x, -y
                '''
                
                # Special cases for trajectories that run straight across the axis
                if start_angle == 0 and end_angle == 0:
                    quadrant_1 = (0.0, 0.0, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                    quadrant_2 = (-180, -180, *trajectory_info, list(zip(flipped_xs, ys, yaws_quad2)))
                    
                    all_trajectories[quadrant_1[0]].append(quadrant_1)
                    all_trajectories[quadrant_2[0]].append(quadrant_2)
                
                elif (start_angle == 90 and end_angle == 90):
                    quadrant_1 = (start_angle, end_angle, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                    quadrant_4 = (-start_angle, -end_angle, *trajectory_info, list(zip(xs, flipped_ys, yaws_quad4)))

                    all_trajectories[quadrant_1[0]].append(quadrant_1)
                    all_trajectories[quadrant_4[0]].append(quadrant_4)

                else:
                    # Need to prevent 180 or -0 being added as a start or end angle
                    if start_angle == 0:
                        quadrant_1 = (start_angle, end_angle, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                        quadrant_2 = (-180, 180 - end_angle, *trajectory_info, list(zip(flipped_xs, ys, yaws_quad2)))
                        quadrant_3 = (-180, end_angle - 180, *trajectory_info, list(zip(flipped_xs, flipped_ys, yaws_quad3)))
                        quadrant_4 = (start_angle, -end_angle, *trajectory_info, list(zip(xs, flipped_ys, yaws_quad4)))

                    elif end_angle == 0:
                        quadrant_1 = (start_angle, end_angle, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                        quadrant_2 = (180 - start_angle, -180, *trajectory_info, list(zip(flipped_xs, ys, yaws_quad2)))
                        quadrant_3 = (start_angle - 180, -180, *trajectory_info, list(zip(flipped_xs, flipped_ys, yaws_quad3)))
                        quadrant_4 = (-start_angle, end_angle, *trajectory_info, list(zip(xs, flipped_ys, yaws_quad4)))

                    else:
                        quadrant_1 = (start_angle, end_angle, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                        quadrant_2 = (180 - start_angle, 180 - end_angle, *trajectory_info, list(zip(flipped_xs, ys, yaws_quad2)))
                        quadrant_3 = (start_angle - 180, end_angle - 180, *trajectory_info, list(zip(flipped_xs, flipped_ys, yaws_quad3)))
                        quadrant_4 = (-start_angle, -end_angle, *trajectory_info, list(zip(xs, flipped_ys, yaws_quad4)))

                    all_trajectories[quadrant_1[0]].append(quadrant_1)
                    all_trajectories[quadrant_2[0]].append(quadrant_2)
                    all_trajectories[quadrant_3[0]].append(quadrant_3)
                    all_trajectories[quadrant_4[0]].append(quadrant_4)

        return all_trajectories

    def handle_motion_model(self, spanning_set):

        if self.motion_model == MotionModel.ACKERMANN:
            return spanning_set

        elif self.motion_model == MotionModel.DIFF:
            diff_spanning_set = self.add_in_place_turns(spanning_set)
            return diff_spanning_set

        elif self.motion_model == MotionModel.OMNI:
            omni_spanning_set = self.add_in_place_turns(spanning_set)
            omni_spanning_set = self.add_horizontal_motions(omni_spanning_set)
            return omni_spanning_set

        else:
            print(f"No handling implemented for Motion Model: {self.motion_model}")
            raise NotImplementedError
    
    def add_in_place_turns(self, spanning_set):
        all_angles = sorted(spanning_set.keys(), key=spanning_set.get)
        
        for idx, start_angle in enumerate(all_angles):
            prev_angle_idx = idx - 1 if idx - 1 >= 0 else len(all_angles) - 1
            next_angle_idx = idx + 1 if idx + 1 < len(all_angles) else 0

            prev_angle = all_angles[prev_angle_idx]
            next_angle = all_angles[next_angle_idx]

            turn_left = (start_angle, prev_angle, 0, 0, 0, 0, [[0, 0, start_angle], [0, 0, prev_angle]])
            turn_right = (start_angle, next_angle, 0, 0, 0, 0, [[0, 0, start_angle], [0, 0, next_angle]])

            spanning_set[start_angle].append(turn_left)
            spanning_set[start_angle].append(turn_right)

        return spanning_set

    def add_horizontal_motions(self, spanning_set):
        min_trajectory_length = self.compute_min_trajectory_length()
        steps = int(np.round(min_trajectory_length/self.grid_separation))

        for start_angle in spanning_set.keys():
            xs = np.linspace(0, min_trajectory_length * np.cos(np.deg2rad(start_angle + 90)), steps)
            ys = np.linspace(0, min_trajectory_length * np.sin(np.deg2rad(start_angle + 90)), steps)
            yaws = np.full(steps, np.deg2rad(start_angle), dtype=np.float64)

            flipped_xs = -xs
            flipped_ys = -ys

            xs = xs.round(5)
            ys = ys.round(5)
            flipped_xs = flipped_xs.round(5)
            flipped_ys = flipped_ys.round(5)

            xs += 0.
            ys += 0.
            yaws += 0.
            flipped_xs += 0.
            flipped_ys += 0.

            left_motion = (start_angle, start_angle, 0, min_trajectory_length, 0, min_trajectory_length, list(zip(xs, ys, yaws)))
            right_motion = (start_angle, start_angle, 0, min_trajectory_length, 0, min_trajectory_length, list(zip(flipped_xs, flipped_ys, yaws)))
        
            spanning_set[start_angle].append(left_motion)
            spanning_set[start_angle].append(right_motion)

        return spanning_set

    def run(self):
        complete_spanning_set = self.generate_minimal_spanning_set()

        return self.handle_motion_model(complete_spanning_set)