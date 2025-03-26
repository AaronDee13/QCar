#This program uses pulls pick-up and drop-off locations from queue, creates a path,
#loads path into another queue to be used in PID Controller. 
import queue
import multiprocessing
import numpy as np
from hal.products.mats import SDCSRoadMap
import matplotlib.image as mpimg
import os

import numpy as np
from hal.products.mats import SDCSRoadMap
import matplotlib.image as mpimg
import os

def main(path_queue: multiprocessing.Queue,
         PU_DO_queue: multiprocessing.Queue,):
    # Configuration parameters
    print("Starting route planning...")
    useSmallMap = False
    leftHandTraffic = False
    home_waypoint = 10
    initial_x = PU_DO_queue.get()
    initial_y = PU_DO_queue.get()
    dest_x = PU_DO_queue.get()
    dest_y = PU_DO_queue.get()
    print(f"Initial Waypoint Coordinates: {initial_x},{initial_y}")
    print(f"Destination Waypoint Coordinates: {dest_x},{dest_y}")

    #initial_waypoint = PU_DO_queue.get()
    #dest_waypoint = PU_DO_queue.get()

    # Create a SDCSRoadMap instance with desired configuration.
    roadmap = SDCSRoadMap(
        leftHandTraffic=leftHandTraffic,
        useSmallMap=useSmallMap
    )

    #we need to find the nearest waypoints for these
    initial_waypoint = 0
    initial_waypoint_distance = 100000000
    dest_waypoint = 0
    dest_waypoint_distance = 100000000
    for i in range(len(roadmap.nodes)):
        node = roadmap.nodes[i]
        distance = np.sqrt((node.pose[0,0] - initial_x)**2 + (node.pose[1,0] - initial_y)**2)
        print(f"x = {node.pose[0,0]}, y = {node.pose[1,0]}")
        if distance < initial_waypoint_distance:
            initial_waypoint = i
            initial_waypoint_distance = distance
        distance = np.sqrt((node.pose[0,0] - dest_x)**2 + (node.pose[1,0] - dest_y)**2)
        if distance < dest_waypoint_distance:
            dest_waypoint = i
            dest_waypoint_distance = distance
    
    print(f"Initial Waypoint: {initial_waypoint}")
    print(f"Destination Waypoint: {dest_waypoint}")

    nodeSequence = [home_waypoint,initial_waypoint, dest_waypoint,home_waypoint]

    # Generate the shortest path passing through the given sequence of nodes.
    # - nodeSequence can be a list or tuple of node indicies.
    # - The generated path takes the form of a 2xn numpy array
    path = roadmap.generate_path(nodeSequence=nodeSequence)
    # for i in nodeSequence:
    #     node = roadmap.nodes[i]
    #     print(
    #         'Node ' + str(i) + ': Pose = ['
    #         + str(node.pose[0, 0]) + ', '
    #         + str(node.pose[1 ,0]) + ', '
    #         + str(node.pose[2, 0]) + ']'
    #     )
    # Display the roadmap with nodes, edges, and labels using matplotlib.
    plt, ax = roadmap.display()
    #plt.ion()  # Enable interactive mode
    # Plot the generated path, if one exists
    if path is None:
        print('failed to find path')
    else:
        print("Mapped...")
        ax.plot(path[0, :], path[1, :], 'red', linestyle='-', linewidth=2)

    path_queue.put((path))
    PU_DO_queue.put(initial_waypoint)
    PU_DO_queue.put(dest_waypoint)
    #plt.show()  
    print("Exited Path Planning")
    return 0


#dunder method used for testing/debugging
if __name__ == '__main__':
    PU_DO_queue = multiprocessing.Queue()
    path_queue = multiprocessing.Queue()
    initial_waypoint = 10
    dest_waypoint = 22
    PU_DO_queue.put(initial_waypoint)
    PU_DO_queue.put(dest_waypoint)  
    main(path_queue,PU_DO_queue)

    # path = path_queue.get()
    # print(path)
    