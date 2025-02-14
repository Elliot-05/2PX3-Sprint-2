from new_intersection import *

#defining the intersection
intersection = nIntersection()
#assigning the simulation
sim = intersection.get_sim()
#defining the window that displays the simulation
win = Window(sim)

#starts running the simulation then displays it
win.run()
win.show()