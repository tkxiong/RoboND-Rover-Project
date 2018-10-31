import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def find_path_thresh(img, rgb_thresh=(160, 140, 140)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] >= rgb_thresh[0]) \
                & (img[:,:,1] >= rgb_thresh[1]) \
                & (img[:,:,2] >= rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify pixels above the threshold
# Find rock pixel
def find_rock_thresh(img, rgb_thresh=(100, 100, 40)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] >= rgb_thresh[0]) \
                & (img[:,:,1] >= rgb_thresh[1]) \
                & (img[:,:,2] <= rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Identify pixels above the threshold
# Find obstacle pixel
def find_obstacle_thresh(img, rgb_thresh=(160, 140, 140)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] <= rgb_thresh[0]) \
                & (img[:,:,1] <= rgb_thresh[1]) \
                & (img[:,:,2] <= rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    rover_img = Rover.img
    rover_roll, rover_pitch, rover_yaw = Rover.roll, Rover.pitch, Rover.yaw
    rover_xpos, rover_ypos = Rover.pos[0], Rover.pos[1]
    world_size =  Rover.worldmap.shape[0]
    # 1) Define source and destination points for perspective transform
    dst_size = 8
    bottom_offset = 6
    scale = 2 * dst_size
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[rover_img.shape[1]/2 - dst_size, rover_img.shape[0] - bottom_offset],
                  [rover_img.shape[1]/2 + dst_size, rover_img.shape[0] - bottom_offset],
                  [rover_img.shape[1]/2 + dst_size, rover_img.shape[0] - 2*dst_size - bottom_offset], 
                  [rover_img.shape[1]/2 - dst_size, rover_img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(rover_img, source, destination)  
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    path_threshed_img = find_path_thresh(warped)            # terrain
    rock_threshed_img = find_rock_thresh(warped)            # rock
    obstacle_threshed_img = find_obstacle_thresh(warped)    # obstacles
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle_threshed_img * 255 # obstacle color-thresholded binary image
    Rover.vision_image[:,:,2] = path_threshed_img * 255     # navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    obstacle_x_rover, obstacle_y_rover = rover_coords(obstacle_threshed_img)
    rock_x_rover, rock_y_rover = rover_coords(rock_threshed_img)
    path_x_rover, path_y_rover = rover_coords(path_threshed_img)

    # 6) Convert rover-centric pixel values to world coordinates
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x_rover, obstacle_y_rover, rover_xpos, rover_ypos, rover_yaw, world_size, scale)
    path_x_world, path_y_world = pix_to_world(path_x_rover, path_y_rover, rover_xpos, rover_ypos, rover_yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if rover_roll <= 0.7 or rover_roll >= 359.5:
        if rover_pitch <= 0.7 or rover_pitch >= 359.5:
            Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 50
            Rover.worldmap[path_y_world, path_x_world, 2] += 50
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    # See if we can find some rocks
    if rock_threshed_img.any():
        rock_x_world, rock_y_world = pix_to_world(rock_x_rover, rock_y_rover, rover_xpos, rover_ypos, rover_yaw, world_size, scale)
        rock_dists, rock_angles = to_polar_coords(rock_x_rover, rock_y_rover)
        rock_idx = np.argmin(rock_dists)
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]
        Rover.rock_dists = rock_dists
        Rover.rock_angles = rock_angles
        # Get the rock mean angle and distance
        mean_rock_dist = np.mean(Rover.rock_dists)
        mean_rock_angle = np.mean(Rover.rock_angles) * 180/np.pi
        print(mean_rock_dist)
        print(mean_rock_angle)
        # Identify rock when the rock distance is equal or less than 60.0 
        # and the rock is on the left side of the robot
        if mean_rock_dist <= 100.0 and mean_rock_angle >= 0:
            Rover.rock_found = True

        Rover.vision_image[:,:,1] = rock_threshed_img * 255     # rock_sample color-thresholded binary image
        Rover.worldmap[rock_y_world, rock_x_world, 1] = 255
    else:
        Rover.vision_image[:,:,1] = 0

    Rover.nav_dists, Rover.nav_angles = to_polar_coords(path_x_rover, path_y_rover)

    return Rover