import numpy as np


# List of states
# - forward
# - stop
# - recovery
# - steering_to_rock
# - slow_forward_to_rock
# - pick_up_rock

# Implemention of left wall following method

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    print (Rover.mode) # Print robot action state

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:

                # It only stores the angle between 50 to -30 degree
                # So as to restrict the robot field of view, to create a left bias direction
                fov_angles = list() 
                for angle in Rover.nav_angles:
                    if angle <= 0.698132 and angle >= -0.523599: # 50 to -30 degrees
                        fov_angles.append(angle * 180/np.pi)

                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0

                # Release brake
                Rover.brake = 0

                mean_fov_angle = np.mean(fov_angles)
                if np.isnan(mean_fov_angle):
                    mean_fov_angle = 0.0

                # steering adjustment
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(mean_fov_angle, -15, 15)

                print(Rover.struck_counter)

                # struck_counter to check whether the robot is struck
                if Rover.vel < 0.2:
                    Rover.struck_counter = Rover.struck_counter + 1
                else:
                    Rover.struck_counter = 0

                # if the roock is detected, go to next state 'stop'
                if Rover.rock_found == True:
                    Rover.mode = 'stop'

                # if the robot is struck, go to next state 'stop'
                if Rover.struck_counter >= 54: # Struck 3 seconds , 18fps, 3 * 16 = 54
                    Rover.struck_counter = 0
                    Rover.mode = 'stop'
            # If there's a lack of navigable terrain pixels then go to next state 'stop'
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0

            # start next state 'recovery'
            elif Rover.vel <= 0.2 and Rover.rock_found == False:
                Rover.steer = 0
                Rover.mode = 'recovery'

            # if the rock is found and robot is stationary, go to next state 'steering_to_rock'
            if Rover.vel == 0.0 and Rover.rock_found == True:
                Rover.steer = 0
                Rover.mode = 'steering_to_rock'
        # Perform recovery when the robot is struck
        elif Rover.mode == 'recovery':
            left_dists = list()
            center_dists = list()
            right_dists = list()
            # Robot vision is separated into 3 views, left, right and center
            for angle, dist in zip(Rover.nav_angles, Rover.nav_dists):
                if angle <= 0.0349066 and angle >= -0.0349066: # -2 to 2 deg
                    center_dists.append(dist)
                if angle <= -0.436332 and angle >= -0.523599: # -25 to -30 deg
                    right_dists.append(dist)
                if angle <= 0.52359 and angle >= 0.436332: # 25 to 30 deg
                    left_dists.append(dist)
            
            # compute the mean value
            mean_center_dist = np.mean(center_dists)
            mean_left_dist = np.mean(left_dists)
            mean_right_dist = np.mean(right_dists)

            if np.isnan(mean_center_dist):
                mean_center_dist = 0.0
            if np.isnan(mean_left_dist):
                mean_left_dist = 0.0
            if np.isnan(mean_right_dist):
                mean_right_dist = 0.0

            print(mean_center_dist)
            print(mean_left_dist)
            print(mean_right_dist)

            if Rover.vel <= 0.1:
                Rover.vel_counter = Rover.vel_counter + 1
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = -15
                print('vel = 0, turning')
                if Rover.vel_counter >= 36:
                    Rover.vel_counter = 0
                    Rover.mode = 'forward'
            elif (len(Rover.nav_angles) < Rover.go_forward) or (mean_center_dist <= 15.0) or (mean_left_dist <= 15.0) or (mean_right_dist <= 15.0):
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                if mean_right_dist <= 15.0:
                    Rover.steer = 15 # Turn left
                else:
                    Rover.steer = -15 #Turn right
            # If we're stopped but see sufficient navigable terrain in front then go!
            # Return back to 'forward'
            elif len(Rover.nav_angles) >= Rover.go_forward:
                Rover.steer = 0
                # Set throttle back to stored value
                Rover.throttle = 0
                # Release the brake
                Rover.brake = 0
                Rover.mode = 'forward'
        # Steer the robot direction to face the rock
        elif Rover.mode == 'steering_to_rock':

            Rover.throttle = 0
            Rover.brake = 0

            mean_rock_dist = np.mean(Rover.rock_dists)
            mean_rock_angle = np.mean(Rover.rock_angles) * 180/np.pi

            if np.isnan(mean_rock_dist):
                mean_rock_dist = 0.0
            if np.isnan(mean_rock_angle):
                mean_rock_angle = 0.0

            print(mean_rock_dist)
            print(mean_rock_angle)

            Rover.slow_steering_counter = Rover.slow_steering_counter + 1
            # the robot must face the rock between -15 to 15 deg,
            # if true , go to next state 'slow_forward_to_rock'
            if mean_rock_angle <= 15.0 and mean_rock_angle >= -15.0 or Rover.slow_steering_counter >= 100:
                Rover.slow_steering_counter = 0
                Rover.mode = 'slow_forward_to_rock'
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            elif mean_rock_angle >= 0.0:
                Rover.steer = 4.0
                print("left")
            else:
                Rover.steer = -4.0
                print("right")
        
        elif Rover.mode == 'slow_forward_to_rock':
            Rover.throttle = 0.2
            Rover.brake = 0
            Rover.steer = 0
            Rover.slow_forward_counter = Rover.slow_forward_counter + 1

            # if the robot still unable to reach the rock, go to next state 'stop'
            if Rover.slow_forward_counter >= 100:
                Rover.slow_forward_counter = 0
                Rover.mode = 'stop'
            # the robot is near the rock, set send_pickup and go to next state 'pick_up_rock'
            if Rover.near_sample == 1:
                Rover.throttle = 0.0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.send_pickup = True
                Rover.mode = 'pick_up_rock'
        # Waiting for the picking of rock process to be completed
        elif Rover.mode == 'pick_up_rock':
            Rover.pickup_counter = Rover.pickup_counter + 1
            # When the pick_counter timeout, resume back to 'forward'
            if Rover.pickup_counter >= 500:
                print("Done")
                Rover.pickup_counter = 0
                Rover.rock_found = False
                Rover.mode = 'forward'



    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    
    return Rover

