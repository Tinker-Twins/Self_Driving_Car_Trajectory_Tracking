#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division

# System level imports
import sys
import os
import argparse
import logging
import time
import math
import numpy as np
import csv
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import configparser
# Required to ignore python warnings in terminal window
import warnings
warnings.filterwarnings("ignore")

# Local level imports
import Controller

# Script level imports
sys.path.append(os.path.abspath(sys.path[0] + '/..'))
import Live_Plotter as lv   # Custom live plotting library
from carla            import sensor
from carla.client     import make_carla_client, VehicleControl
from carla.settings   import CarlaSettings
from carla.tcp        import TCPConnectionError
from carla.controller import utils

"""
Configurable Parameters
"""
ITER_FOR_SIM_TIMESTEP  = 10     # no. iterations to compute approx sim timestep
WAIT_TIME_BEFORE_START = 5.00   # simulator seconds (time before controller start)
TOTAL_RUN_TIME         = 200.00 # simulator seconds (total runtime before sim end)
TOTAL_FRAME_BUFFER     = 300    # number of frames to buffer after total runtime
NUM_PEDESTRIANS        = 0      # total number of pedestrians to spawn
NUM_VEHICLES           = 0      # total number of vehicles to spawn
SEED_PEDESTRIANS       = 0      # seed for pedestrian spawn randomizer
SEED_VEHICLES          = 0      # seed for vehicle spawn randomizer

WEATHERID = {
    "DEFAULT": 0,
    "CLEARNOON": 1,
    "CLOUDYNOON": 2,
    "WETNOON": 3,
    "WETCLOUDYNOON": 4,
    "MIDRAINYNOON": 5,
    "HARDRAINNOON": 6,
    "SOFTRAINNOON": 7,
    "CLEARSUNSET": 8,
    "CLOUDYSUNSET": 9,
    "WETSUNSET": 10,
    "WETCLOUDYSUNSET": 11,
    "MIDRAINSUNSET": 12,
    "HARDRAINSUNSET": 13,
    "SOFTRAINSUNSET": 14,
}
SIMWEATHER = WEATHERID["CLEARNOON"]     # set simulation weather

PLAYER_START_INDEX = 1      # spawn index for player (keep to 1)
FIGSIZE_X_INCHES   = 6      # x figure size of feedback in inches
FIGSIZE_Y_INCHES   = 8      # y figure size of feedback in inches
PLOT_LEFT          = 0.1    # in fractions of figure width and height
PLOT_BOT           = 0.1
PLOT_WIDTH         = 0.8
PLOT_HEIGHT        = 0.8

WAYPOINTS_FILENAME = 'Waypoints.txt'  # waypoint file to load
DIST_THRESHOLD_TO_LAST_WAYPOINT = 1.0 # some distance from last position before simulation ends (6 for Bang-Bang, 1 for others)

# Path interpolation parameters
INTERP_MAX_POINTS_PLOT    = 10   # number of points used for displaying
                                 # lookahead path
INTERP_LOOKAHEAD_DISTANCE = 20   # lookahead in meters
INTERP_DISTANCE_RES       = 0.01 # distance between interpolated points

# Controller output directory
CONTROLLER_OUTPUT_FOLDER = os.path.dirname(os.path.realpath(__file__)) +\
                          '/Results/'

def make_carla_settings(args):
    """Make a CarlaSettings Object with the Required Settings
    """
    settings = CarlaSettings()

    # There is no need for non-agent info requests if there are no pedestrians or vehicles
    get_non_player_agents_info = False
    if (NUM_PEDESTRIANS > 0 or NUM_VEHICLES > 0):
        get_non_player_agents_info = True

    # Base level settings
    settings.set(
        SynchronousMode=True,
        SendNonPlayerAgentsInfo=get_non_player_agents_info,
        NumberOfVehicles=NUM_VEHICLES,
        NumberOfPedestrians=NUM_PEDESTRIANS,
        SeedVehicles=SEED_VEHICLES,
        SeedPedestrians=SEED_PEDESTRIANS,
        WeatherId=SIMWEATHER,
        QualityLevel=args.quality_level)
    return settings

class Timer(object):
    """ Timer Class

    The steps are used to calculate FPS, while the lap or seconds since lap is
    used to compute elapsed time.
    """
    def __init__(self, period):
        self.step = 0
        self._lap_step = 0
        self._lap_time = time.time()
        self._period_for_lap = period

    def tick(self):
        self.step += 1

    def has_exceeded_lap_period(self):
        if self.elapsed_seconds_since_lap() >= self._period_for_lap:
            return True
        else:
            return False

    def lap(self):
        self._lap_step = self.step
        self._lap_time = time.time()

    def ticks_per_second(self):
        return float(self.step - self._lap_step) /\
                     self.elapsed_seconds_since_lap()

    def elapsed_seconds_since_lap(self):
        return time.time() - self._lap_time

def get_current_pose(measurement):
    """
    Obtains current x,y,yaw pose from the client measurements

    Args:
        measurement: The CARLA client measurements (from read_data())

    Returns: (x, y, yaw)
        x: X position in meters
        y: Y position in meters
        yaw: Yaw position in radians
    """
    x   = measurement.player_measurements.transform.location.x
    y   = measurement.player_measurements.transform.location.y
    yaw = math.radians(measurement.player_measurements.transform.rotation.yaw)

    return (x, y, yaw)

def get_start_pos(scene):
    """
    Obtains player start x,y, yaw pose from the scene

    Args:
        scene: The CARLA scene object

    Returns: (x, y, yaw)
        x: X position in meters
        y: Y position in meters
        yaw: Yaw position in radians
    """
    x = scene.player_start_spots[0].location.x
    y = scene.player_start_spots[0].location.y
    yaw = math.radians(scene.player_start_spots[0].rotation.yaw)

    return (x, y, yaw)

def send_control_command(client, throttle, steer, brake,
                         hand_brake=False, reverse=False):
    """
    Send control command to CARLA client

    Args:
        client: The CARLA client object
        throttle: Throttle command for the sim car [0, 1]
        steer: Steer command for the sim car [-1, 1]
        brake: Brake command for the sim car [0, 1]
        hand_brake: Whether the hand brake is engaged
        reverse: Whether the sim car is in the reverse gear
    """
    control = VehicleControl()
    # Clamp all values within their limits
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control.steer = steer
    control.throttle = throttle
    control.brake = brake
    control.hand_brake = hand_brake
    control.reverse = reverse
    client.send_control(control)

def create_controller_output_dir(output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

def store_trajectory_plot(graph, fname):
    """ Store the Resulting Plot
    """
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, fname)
    graph.savefig(file_name)

def write_trajectory_file(x_list, y_list, v_list, t_list):
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, 'Trajectory.csv') # t (s), x (m), y (m), v (m/s)
    with open(file_name, 'w') as trajectory_file:
        for i in range(len(x_list)):
            trajectory_file.write('%0.3f, %0.3f, %0.3f, %0.3f\n' %\
                                  (t_list[i], x_list[i], y_list[i], v_list[i]))

def write_error_log(cte_list, he_list):
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, 'Tracking Error Log.csv') # cte (m), he (rad)
    with open(file_name, 'w') as error_log:
        for i in range(len(cte_list)):
            error_log.write('%0.10f,%0.10f\n' % (cte_list[i], he_list[i]))

def write_latency_log(latency_list):
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, 'Latency Log.csv') # latency (ms)
    with open(file_name, 'w') as latency_log:
        for i in range(len(latency_list)):
            latency_log.write('%0.10f\n' % (latency_list[i]))

def exec_waypoint_nav_demo(args):
    """ Executes Waypoint Navigation
    """
    print('---------------------------------------------------------------------------')
    if args.longitudinal_controller == 'PID':
        print("\nLongitudinal Control: PID Controller")
    elif args.longitudinal_controller == 'ALC':
        print("\nLongitudinal Control: Adaptive Throttle Controller")
    else:
        print("\nUndefined Longitudinal Control Method Selected")

    if args.lateral_controller == 'BangBang':
        print("Lateral Control: Bang-Bang Controller\n")
    elif args.lateral_controller == 'PID':
        print("Lateral Control: PID Controller\n")
    elif args.lateral_controller == 'PurePursuit':
        print("Lateral Control: Pure Pursuit Controller\n")
    elif args.lateral_controller == 'Stanley':
        print("Lateral Control: Stanley Controller\n")
    elif args.lateral_controller == 'POP':
        print("Lateral Control: Proximally Optimal Pursuit Controller\n")
    else:
        print("Undefined Lateral Control Method Selected\n")

    with make_carla_client(args.host, args.port) as client:
        print('Connection Established with the Simulator\n')

        settings = make_carla_settings(args)

        # Now we load these settings into the server. The server replies
        # with a scene description containing the available start spots for
        # the player. Here we can provide a CarlaSettings object or a
        # CarlaSettings.ini file as string.
        scene = client.load_settings(settings)

        # Refer to the player start folder in the WorldOutliner to see the
        # player start information
        player_start = PLAYER_START_INDEX

        # Notify the server that we want to start the episode at the
        # player_start index. This function blocks until the server is ready
        # to start the episode.

        # print('Starting New Episode at %r\n' % scene.map_name)
        client.start_episode(player_start)

        #############################################
        # Load Configurations
        #############################################

        # Load configuration file (Configurations.cfg) and then parses for the various
        # options. Here we have two main options:
        # live_plotting and live_plotting_period, which controls whether
        # live plotting is enabled or how often the live plotter updates
        # during the simulation run.
        config = configparser.ConfigParser()
        config.read(os.path.join(
                os.path.dirname(os.path.realpath(__file__)), 'Configurations.cfg'))
        demo_opt = config['Demo Parameters']

        # Get options
        enable_live_plot = demo_opt.get('live_plotting', 'true').capitalize()
        enable_live_plot = enable_live_plot == 'True'
        live_plot_period = float(demo_opt.get('live_plotting_period', 0))

        # Set options
        live_plot_timer = Timer(live_plot_period)

        #############################################
        # Load Waypoints
        #############################################
        # Opens the waypoint file and stores it to "waypoints"
        waypoints_file = WAYPOINTS_FILENAME
        waypoints_np   = None
        with open(waypoints_file) as waypoints_file_handle:
            waypoints = list(csv.reader(waypoints_file_handle,
                                        delimiter=',',
                                        quoting=csv.QUOTE_NONNUMERIC))
            waypoints_np = np.array(waypoints)

        # Because the waypoints are discrete and our controller performs better
        # with a continuous path, here we will send a subset of the waypoints
        # within some lookahead distance from the closest point to the vehicle.
        # Interpolating between each waypoint will provide a finer resolution
        # path and make it more "continuous". A simple linear interpolation
        # is used as a preliminary method to address this issue, though it is
        # better addressed with better interpolation methods (spline
        # interpolation, for example).
        # More appropriate interpolation methods will not be used here for the
        # sake of demonstration on what effects discrete paths can have on
        # the controller. It is made much more obvious with linear
        # interpolation, because in a way part of the path will be continuous
        # while the discontinuous parts (which happens at the waypoints) will
        # show just what sort of effects these points have on the controller.
        # Can you spot these during the simulation? If so, how can you further
        # reduce these effects?

        # Linear interpolation computations
        # Compute a list of distances between waypoints
        wp_distance = []   # distance array
        for i in range(1, waypoints_np.shape[0]):
            wp_distance.append(
                    np.sqrt((waypoints_np[i, 0] - waypoints_np[i-1, 0])**2 +
                            (waypoints_np[i, 1] - waypoints_np[i-1, 1])**2))
        wp_distance.append(0)  # last distance is 0 because it is the distance
                               # from the last waypoint to the last waypoint

        # Linearly interpolate between waypoints and store in a list
        wp_interp      = []    # interpolated values
                               # (rows = waypoints, columns = [x, y, v])
        wp_interp_hash = []    # hash table which indexes waypoints_np
                               # to the index of the waypoint in wp_interp
        interp_counter = 0     # counter for current interpolated point index
        for i in range(waypoints_np.shape[0] - 1):
            # Add original waypoint to interpolated waypoints list (and append
            # it to the hash table)
            wp_interp.append(list(waypoints_np[i]))
            wp_interp_hash.append(interp_counter)
            interp_counter+=1

            # Interpolate to the next waypoint. First compute the number of
            # points to interpolate based on the desired resolution and
            # incrementally add interpolated points until the next waypoint
            # is about to be reached.
            num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                         float(INTERP_DISTANCE_RES)) - 1)
            wp_vector = waypoints_np[i+1] - waypoints_np[i]
            wp_uvector = wp_vector / np.linalg.norm(wp_vector)
            for j in range(num_pts_to_interp):
                next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
                wp_interp.append(list(waypoints_np[i] + next_wp_vector))
                interp_counter+=1
        # Add last waypoint at the end
        wp_interp.append(list(waypoints_np[-1]))
        wp_interp_hash.append(interp_counter)
        interp_counter+=1

        #############################################
        # Controller  Class Declaration
        #############################################
        # This is where we take the Controller.py class
        # and apply it to the simulator
        controller = Controller.Controller(waypoints, args.lateral_controller, args.longitudinal_controller)

        ###########################################################
        # Determine simulation average timestep (and total frames)
        ###########################################################
        # Ensure at least one frame is used to compute average timestep
        num_iterations = ITER_FOR_SIM_TIMESTEP
        if (ITER_FOR_SIM_TIMESTEP < 1):
            num_iterations = 1

        # Gather current data from the CARLA server. This is used to get the
        # simulator starting simulator time. Note that we also need to
        # send a command back to the CARLA server because synchronous mode
        # is enabled.
        measurement_data, sensor_data = client.read_data()
        sim_start_stamp = measurement_data.game_timestamp / 1000.0
        # Send a control command to proceed to next iteration.
        # This mainly applies for simulations that are in synchronous mode.
        send_control_command(client, throttle=0.0, steer=0, brake=1.0)
        # Computes the average timestep based on several initial iterations
        sim_duration = 0
        for i in range(num_iterations):
            # Gather current data
            measurement_data, sensor_data = client.read_data()
            # Send a control command to proceed to next iteration
            send_control_command(client, throttle=0.0, steer=0, brake=1.0)
            # Last stamp
            if i == num_iterations - 1:
                sim_duration = measurement_data.game_timestamp / 1000.0 - sim_start_stamp

        # Outputs average simulation timestep and computes how many frames
        # will elapse before the simulation should end based on various
        # parameters that we set in the beginning.
        SIMULATION_TIME_STEP = sim_duration / float(num_iterations)
        print("SERVER SIMULATION STEP APPROXIMATION: " + str(SIMULATION_TIME_STEP))
        TOTAL_EPISODE_FRAMES = int((TOTAL_RUN_TIME + WAIT_TIME_BEFORE_START) / SIMULATION_TIME_STEP) + TOTAL_FRAME_BUFFER

        #############################################
        # Frame-by-Frame Iteration and Initialization
        #############################################
        # Store pose history starting from the start position
        measurement_data, sensor_data = client.read_data()
        start_x, start_y, start_yaw = get_current_pose(measurement_data)
        send_control_command(client, throttle=0.0, steer=0, brake=1.0)
        x_history     = [start_x]
        y_history     = [start_y]
        yaw_history   = [start_yaw]
        time_history  = [0]
        speed_history = [0]

        # Initialize lsits to store error values
        cte_history = []
        he_history = []

        # Initialize lsit to store latency values
        latency_history = []

        #############################################
        # Vehicle Trajectory Live Plotting Setup
        #############################################
        # Uses the live plotter to generate live feedback during the simulation
        # The two feedback includes the trajectory feedback and
        # the controller feedback (which includes the speed tracking).
        lp_traj = lv.LivePlotter(tk_title="Trajectory Trace", win_pos=[1250,125])
        lp_params = lv.LivePlotter(tk_title="Ego-Vehicle Parameters", win_pos=[50,125])

        ###
        # Add 2D position (trajectory) plot
        ###
        trajectory_fig = lp_traj.plot_new_dynamic_2d_figure(
                title='Ego-Vehicle Trajectory',
                figsize=(FIGSIZE_X_INCHES, FIGSIZE_Y_INCHES),
                edgecolor="black",
                rect=[PLOT_LEFT, PLOT_BOT, PLOT_WIDTH, PLOT_HEIGHT])

        trajectory_fig.set_invert_x_axis() # Because UE4 uses left-handed
                                           # coordinate system the X
                                           # axis in the graph is flipped
        trajectory_fig.set_axis_equal()    # X-Y spacing should be equal in size

        # Add waypoint markers
        trajectory_fig.add_graph("waypoints", window_size=waypoints_np.shape[0],
                                 x0=-waypoints_np[:,0], y0=-waypoints_np[:,1],
                                 linestyle="-", marker="", color='gray')
        # Add trajectory markers
        trajectory_fig.add_graph("trajectory", window_size=TOTAL_EPISODE_FRAMES,
                                 x0=-np.asarray([start_x]*TOTAL_EPISODE_FRAMES),
                                 y0=-np.asarray([start_y]*TOTAL_EPISODE_FRAMES),
                                 color='deepskyblue')

        # Add lookahead path
        trajectory_fig.add_graph("lookahead_path",
                                 window_size=INTERP_MAX_POINTS_PLOT,
                                 x0=-np.asarray([start_x]*INTERP_MAX_POINTS_PLOT),
                                 y0=-np.asarray([start_y]*INTERP_MAX_POINTS_PLOT),
                                 color='green',
                                 linewidth=4)

        # Add starting position marker
        trajectory_fig.add_graph("start_pos", window_size=1,
                                 x0=-np.asarray([start_x]), y0=-np.asarray([start_y]),
                                 marker="D", color='red',
                                 markertext="Start",
                                 marker_text_offset_x=26, marker_text_offset_y=-26)
        # Add end position marker
        trajectory_fig.add_graph("end_pos", window_size=1,
                                 x0=-np.asarray([waypoints_np[-1, 0]]),
                                 y0=-np.asarray([waypoints_np[-1, 1]]),
                                 marker="D", color='red',
                                 markertext="Finish",
                                 marker_text_offset_x=30, marker_text_offset_y=20)
        # Add car marker
        trajectory_fig.add_graph("car", window_size=1,
                                 marker="s", color='blue', markertext="Car",
                                 marker_text_offset_x=15, marker_text_offset_y=-25)

        ###
        # Add speed profile updater
        ###
        forward_speed_fig = lp_params.plot_new_dynamic_figure(title="Forward Velocity (km/h)")
        forward_speed_fig.add_graph("reference_signal",
                                    label="reference_Signal",
                                    window_size=TOTAL_EPISODE_FRAMES,
                                    color='gray')
        forward_speed_fig.add_graph("forward_speed",
                                    label="forward_speed",
                                    window_size=TOTAL_EPISODE_FRAMES,
                                    color='deepskyblue')

        # Add throttle signals graph
        throttle_fig = lp_params.plot_new_dynamic_figure(title="Throttle (%)")
        throttle_fig.add_graph("throttle",
                              label="throttle",
                              window_size=TOTAL_EPISODE_FRAMES,
                              color='deepskyblue')

        # Add brake signals graph
        brake_fig = lp_params.plot_new_dynamic_figure(title="Brake (%)")
        brake_fig.add_graph("brake",
                              label="brake",
                              window_size=TOTAL_EPISODE_FRAMES,
                              color='deepskyblue')

        # Add steering signals graph
        steer_fig = lp_params.plot_new_dynamic_figure(title="Steering Angle (deg)")
        steer_fig.add_graph("steer",
                              label="steer",
                              window_size=TOTAL_EPISODE_FRAMES,
                              color='deepskyblue')

        # If live plotter is disabled, hide windows
        if not enable_live_plot:
            lp_traj._root.withdraw()
            lp_params._root.withdraw()

        #############################################
        # Scenario Execution Loop
        #############################################

        # Iterate the frames until the end of the waypoints is reached or
        # the TOTAL_EPISODE_FRAMES is reached. The controller simulation then
        # ouptuts the results to the controller output directory.
        reached_the_end = False
        skip_first_frame = True
        closest_index    = 0  # Index of waypoint that is currently closest to
                              # the car (assumed to be the first index)
        closest_distance = 0  # Closest distance of closest waypoint to car
        for frame in range(TOTAL_EPISODE_FRAMES):
            # Gather current data from the CARLA server
            measurement_data, sensor_data = client.read_data()

            # Update pose, timestamp
            current_x, current_y, current_yaw = get_current_pose(measurement_data)
            current_speed = measurement_data.player_measurements.forward_speed
            current_timestamp = float(measurement_data.game_timestamp) / 1000.0

            # Shift (x,y) coordinates of vehicle frame to correspond front axle
            # or rear axle of the vehicle
            if args.lateral_controller == 'PurePursuit':
                length = -1.5 # Rear axle
            elif args.lateral_controller == 'BangBang' or args.lateral_controller == 'PID' or args.lateral_controller == 'Stanley' or args.lateral_controller == 'POP':
                length = 1.5 # Front axle
            else:
                length = 0.0

            current_x, current_y = controller.get_shifted_coordinate(current_x, current_y, current_yaw, length)

            # Wait for some initial time before starting the demo
            if current_timestamp <= WAIT_TIME_BEFORE_START:
                send_control_command(client, throttle=0.0, steer=0, brake=1.0)
                continue
            else:
                current_timestamp = current_timestamp - WAIT_TIME_BEFORE_START

            # Store history
            x_history.append(current_x)
            y_history.append(current_y)
            yaw_history.append(current_yaw)
            speed_history.append(current_speed)
            time_history.append(current_timestamp)

            ###
            # Controller update (this uses the Controller.py implementation)
            ###

            # To reduce the amount of waypoints sent to the controller,
            # provide a subset of waypoints that are within some
            # lookahead distance from the closest point to the car. Provide
            # a set of waypoints behind the car as well.

            # Find closest waypoint index to car. First increment the index
            # from the previous index until the new distance calculations
            # are increasing. Apply the same rule decrementing the index.
            # The final index should be the closest point (it is assumed that
            # the car will always break out of instability points where there
            # are two indices with the same minimum distance, as in the
            # center of a circle)
            closest_distance = np.linalg.norm(np.array([
                    waypoints_np[closest_index, 0] - current_x,
                    waypoints_np[closest_index, 1] - current_y]))

            new_distance = closest_distance
            new_index = closest_index
            while new_distance <= closest_distance:
                closest_distance = new_distance
                closest_index = new_index
                new_index += 1
                if new_index >= waypoints_np.shape[0]:  # End of path
                    break
                new_distance = np.linalg.norm(np.array([
                        waypoints_np[new_index, 0] - current_x,
                        waypoints_np[new_index, 1] - current_y]))
            new_distance = closest_distance
            new_index = closest_index
            while new_distance <= closest_distance:
                closest_distance = new_distance
                closest_index = new_index
                new_index -= 1
                if new_index < 0:  # Beginning of path
                    break
                new_distance = np.linalg.norm(np.array([
                        waypoints_np[new_index, 0] - current_x,
                        waypoints_np[new_index, 1] - current_y]))



            # Once the closest index is found, return the path that has 1
            # waypoint behind and X waypoints ahead, where X is the index
            # that has a lookahead distance specified by
            # INTERP_LOOKAHEAD_DISTANCE
            waypoint_subset_first_index = closest_index - 1
            if waypoint_subset_first_index < 0:
                waypoint_subset_first_index = 0

            waypoint_subset_last_index = closest_index
            total_distance_ahead = 0
            while total_distance_ahead < INTERP_LOOKAHEAD_DISTANCE:
                total_distance_ahead += wp_distance[waypoint_subset_last_index]
                waypoint_subset_last_index += 1
                if waypoint_subset_last_index >= waypoints_np.shape[0]:
                    waypoint_subset_last_index = waypoints_np.shape[0] - 1
                    break

            #print("length of wp_interp",len(wp_interp))
            #print("new_index :",new_index)
            #print("waypoint_subset_first_index :",waypoint_subset_first_index)
            #print("waypoint_subset_last_index :",waypoint_subset_last_index)
            #print("wp_interp_hash[waypoint_subset_first_index] :",wp_interp_hash[waypoint_subset_first_index])
            #print("wp_interp_hash[waypoint_subset_last_index] :",wp_interp_hash[waypoint_subset_last_index])


            # Use the first and last waypoint subset indices into the hash
            # table to obtain the first and last indicies for the interpolated
            # list. Update the interpolated waypoints to the controller
            # for the next controller update.
            new_waypoints = \
                    wp_interp[wp_interp_hash[waypoint_subset_first_index]:\
                              wp_interp_hash[waypoint_subset_last_index] + 1]
            controller.update_waypoints(new_waypoints)

            # Update the other controller values and controls
            controller.update_values(current_x, current_y, current_yaw,
                                     current_speed,
                                     current_timestamp, frame, new_distance)
            controller.update_controls()
            cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()

            # Store error values
            cte_history.append(controller.get_crosstrack_error(current_x, current_y, new_waypoints))
            he_history.append(controller.get_heading_error(new_waypoints, current_yaw))

            # Store latency values
            latency_history.append(controller._latency)

            # Skip the first frame (so the controller has proper outputs)
            if skip_first_frame and frame == 0:
                pass
            else:
                # Update live plotter with new feedback

                ##############################################################
                # Trajectory Plot
                ##############################################################
                trajectory_fig.roll("trajectory", -current_x, -current_y)
                trajectory_fig.roll("car", -current_x, -current_y)
                # When plotting lookahead path, only plot a number of points
                # (INTERP_MAX_POINTS_PLOT amount of points). This is meant
                # to decrease load when live plotting
                new_waypoints_np = np.array(new_waypoints)
                path_indices = np.floor(np.linspace(0,
                                                    new_waypoints_np.shape[0]-1,
                                                    INTERP_MAX_POINTS_PLOT))

                trajectory_fig.update("lookahead_path",
                        -new_waypoints_np[path_indices.astype(int), 0],
                        -new_waypoints_np[path_indices.astype(int), 1],
                        new_colour='green')

                ##############################################################
                # Velocity Plot
                ##############################################################
                # Actual Velocity
                forward_speed_fig.roll("forward_speed",
                                       current_timestamp,
                                       current_speed*3.6) # m/s to km/h
                # Reference Velocity
                if  args.longitudinal_controller == 'PID': # Plot reference velocity only if tracking it
                    forward_speed_fig.roll("reference_signal",
                                           current_timestamp,
                                           controller._desired_speed*3.6) # m/s to km/h

               ##############################################################
               # Control Input Sub-Plots
               ##############################################################
                throttle_fig.roll("throttle", current_timestamp, cmd_throttle*100)
                brake_fig.roll("brake", current_timestamp, cmd_brake*100)
                steer_fig.roll("steer", current_timestamp, cmd_steer*180/np.pi)

                # Refresh the live plot based on the refresh rate set by the options
                if enable_live_plot and \
                   live_plot_timer.has_exceeded_lap_period():
                   lp_traj.refresh()
                   lp_params.refresh()
                   live_plot_timer.lap()

            # Output controller command to CARLA server
            send_control_command(client,
                                 throttle=cmd_throttle,
                                 steer=cmd_steer,
                                 brake=cmd_brake)

            # Find if car has reached the end of trajectory:
            # If the car is within DIST_THRESHOLD_TO_LAST_WAYPOINT to the last waypoint, the simulation will end.
            dist_to_last_waypoint = np.linalg.norm(np.array([
                waypoints[-1][0] - current_x,
                waypoints[-1][1] - current_y]))
            if  dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
                reached_the_end = True
            if reached_the_end:
                break

        # End of simulation - Stop vehicle and Store outputs to the controller output directory.
        if reached_the_end:
            print("\nReached End of Trajectory. Logging Results...")
        else:
            print("\nExceeded Simulation Time. Logging Results...")
        # Stop the car
        send_control_command(client, throttle=0.0, steer=0.0, brake=1.0)
        # Store the various outputs
        store_trajectory_plot(trajectory_fig.fig, 'Trajectory Plot.png')
        store_trajectory_plot(forward_speed_fig.fig, 'Forward Velocity Plot.png')
        store_trajectory_plot(throttle_fig.fig, 'Throttle Command Plot.png')
        store_trajectory_plot(brake_fig.fig, 'Brake Command Plot.png')
        store_trajectory_plot(steer_fig.fig, 'Steering Angle Plot.png')
        write_trajectory_file(x_history, y_history, speed_history, time_history)
        write_error_log(cte_history, he_history)
        write_latency_log(latency_history)

def main():
    """
    Main function

    Args:
        -v, --verbose: print debug information
        --host: IP of the host server (default: localhost)
        -p, --port: TCP port to listen to (default: 2000)
        -a, --autopilot: enable autopilot
        -q, --quality-level: graphics quality level [Low or Epic]
        -i, --images-to-disk: save images to disk
        -c, --carla-settings: Path to CarlaSettings.ini file
    """
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Epic',
        help='graphics quality level.')
    argparser.add_argument(
        '-c', '--carla-settings',
        metavar='PATH',
        dest='settings_filepath',
        default=None,
        help='Path to a "CarlaSettings.ini" file')
    argparser.add_argument(
        '-lat_ctrl', '--Lateral-Controller:',
        metavar='LATERAL CONTROLLER',
        dest='lateral_controller',
        choices = {'BangBang','PID','PurePursuit','Stanley','POP'},
        default='POP',
        help='Select Lateral Controller')
    argparser.add_argument(
        '-lon_ctrl', '--Longitudinal-Controller:',
        metavar='LONGITUDINAL CONTROLLER',
        dest='longitudinal_controller',
        choices = {'PID','ALC'},
        default='ALC',
        help='Select Longitudinal Controller')
    args = argparser.parse_args()

    # Logging startup info
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('listening to server %s:%s', args.host, args.port)

    args.out_filename_format = '_out/episode_{:0>4d}/{:s}/{:0>6d}'

    # Execute when server connection is established
    while True:
        try:
            exec_waypoint_nav_demo(args)
            print('\nSimulation Complete')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nKeyboard Interrupt Detected...\nTerminating Simulation')
