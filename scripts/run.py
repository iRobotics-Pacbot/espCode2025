# import UDPPeer
# import filter
# import numpy as np
# from maze import maze
# from sensor_projection import _project_sensors_c


# PARTICLE_COUNT = 20000
# CELL_SIZE = 14
# MAZE_WIDTH = len(maze[0])
# MAZE_HEIGHT = len(maze)
# WIDTH = MAZE_WIDTH * CELL_SIZE
# HEIGHT = MAZE_HEIGHT * CELL_SIZE
# MAX_DISTANCE = 200


# #get odom, tof
# #predict(dt)
# #update
# #resample
# #get state estimate
# #send pose





# def main():
#     num_particles = 200
#     starting_pose = np.array([200, 250, 0, 0], dtype = np.float64)
#     initial_sigma = np.array([CELL_SIZE / 2, CELL_SIZE / 2, 0, 0], dtype = np.float64)
#     initial_pose = np.random.normal(starting_pose, initial_sigma)
#     process_sigma = np.array([2, 2, 10, 10])
#     myFilter = filter.ParticleFilter(PARTICLE_COUNT, starting_pose, initial_sigma, process_sigma)
#     myUDP = UDPPeer(myFilter, lambda poses: project_sensors(poses))

#     print("TS")
#     while True:
#         myUDP.processDin()








# if __name__ == "__main__":
#     main()

# def project_sensors(poses):
#     """
#     Project sensors for given poses using the original API.
    
#     Args:
#         poses: numpy array of shape (N, 3) containing (x, y, heading) for each pose
        
#     Returns:
#         numpy array of shape (N, 4) containing sensor distances
#     """
#     # Ensure poses is a numpy array with correct shape and type
#     poses = np.asarray(poses, dtype=np.float64)
#     if len(poses.shape) != 2 or poses.shape[1] != 3:
#         raise ValueError("poses must be a Nx3 array")

#     # Ensure maze is correct type
#     global maze
#     maze = np.asarray(maze, dtype=np.int8)
    
#     # Call C implementation
#     return _project_sensors_c(poses, maze, CELL_SIZE, WIDTH, HEIGHT, MAX_DISTANCE)
