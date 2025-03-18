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
    initial_waypoint = PU_DO_queue.get()
    dest_waypoint = PU_DO_queue.get()

    nodeSequence = [initial_waypoint, dest_waypoint]

    # Create a SDCSRoadMap instance with desired configuration.
    roadmap = SDCSRoadMap(
        leftHandTraffic=leftHandTraffic,
        useSmallMap=useSmallMap
    )

    # Generate the shortest path passing through the given sequence of nodes.
    # - nodeSequence can be a list or tuple of node indicies.
    # - The generated path takes the form of a 2xn numpy array
    path = roadmap.generate_path(nodeSequence=nodeSequence)

    # Display the roadmap with nodes, edges, and labels using matplotlib.
    plt, ax = roadmap.display()
    plt.ion()  # Enable interactive mode
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
    