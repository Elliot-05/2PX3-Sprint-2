
from trafficSimulator import *
import numpy as np


class nIntersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 24
        island_width = 2
        length = 100
        outer_lane = 6.25
        inner_lane = 2.75
        boundry = 20
        intersection_stop = 12
        left_turn_peak = boundry + intersection_stop



#---------------------------------------------------------------Variables----------------------------------------------------------------------------#
        self.vehicle_rate = 50
        self.v = 50
        self.speed_variance = 0
        self.self_driving_vehicle_proportion = 0 #number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 0.2:
            self.v = self.v * 1.5
#----------------------------------------------------------------------------------------------------------------------------------------------------#
    #this section defines all the paths that a vehicle can take
    # SOUTH, EAST, NORTH, WEST
        #INNER, OUTER
        # Intersection in
            #paths 0-7
        #South right road
        self.sim.create_segment((inner_lane, boundry), (inner_lane,intersection_stop)) 
        self.sim.create_segment((outer_lane, boundry), (outer_lane, intersection_stop)) 
        #East upper road
        self.sim.create_segment((boundry, -inner_lane), (intersection_stop, -inner_lane)) 
        self.sim.create_segment((boundry, -outer_lane), (intersection_stop,-outer_lane))
        #North left road
        self.sim.create_segment((-inner_lane, -boundry), (-inner_lane, -intersection_stop)) 
        self.sim.create_segment((-outer_lane, -boundry), (-outer_lane, -intersection_stop)) 
        #West lower road
        self.sim.create_segment((-boundry, inner_lane), (-intersection_stop, inner_lane)) 
        self.sim.create_segment((-boundry, outer_lane), (-intersection_stop, outer_lane))

        # Intersection out
            #paths 8-15
        #South left road
        self.sim.create_segment((-inner_lane, intersection_stop), (-inner_lane, boundry+20))
        self.sim.create_segment((-outer_lane, intersection_stop), (-outer_lane, boundry+20))
        
        #East lower road
        self.sim.create_segment((intersection_stop, inner_lane), (boundry+20, inner_lane))
        self.sim.create_segment((intersection_stop, outer_lane), (boundry+20, outer_lane))
        
        #North Right road
        self.sim.create_segment((inner_lane, -intersection_stop), (inner_lane, -boundry-20))
        self.sim.create_segment((outer_lane, -intersection_stop), (outer_lane, -boundry-20))
        
        #West upper road
        self.sim.create_segment((-intersection_stop, -inner_lane), (-boundry-20, -inner_lane))
        self.sim.create_segment((-intersection_stop, -outer_lane), (-boundry-20, -outer_lane))
        
        # Straight
            #paths 16-19
        #South
        self.sim.create_segment((inner_lane, intersection_stop), (inner_lane, -intersection_stop))
        #East
        self.sim.create_segment((intersection_stop, -inner_lane), (-intersection_stop, -inner_lane))
        #North
        self.sim.create_segment((-inner_lane, -intersection_stop), (-inner_lane, intersection_stop))
        #West
        self.sim.create_segment((-intersection_stop, inner_lane), (intersection_stop, inner_lane))
        
        #Right turn
            #paths 20-23
        #South to East
        self.sim.create_quadratic_bezier_curve((outer_lane, intersection_stop), (outer_lane, outer_lane), (intersection_stop, outer_lane))
        #East to North
        self.sim.create_quadratic_bezier_curve((intersection_stop, -outer_lane), (outer_lane, -outer_lane), (outer_lane, -intersection_stop))
        #North to West
        self.sim.create_quadratic_bezier_curve((-outer_lane, -intersection_stop), (-outer_lane, -outer_lane), (-intersection_stop, -outer_lane))
        #West to South
        self.sim.create_quadratic_bezier_curve((-intersection_stop, outer_lane), (-outer_lane, outer_lane), (-outer_lane, intersection_stop))
        
    
        # Left turn
            #paths 24-35
        #From South to West
        #Make a u-turn | Transition phase midpoint, one of the axis values stay the same, other adds left turn peak. Ending has same value as left turn peak, changed distance
        self.sim.create_quadratic_bezier_curve((inner_lane, -boundry), (inner_lane, - boundry - intersection_stop), (inner_lane - intersection_stop, - boundry - intersection_stop))
        self.sim.create_quadratic_bezier_curve((inner_lane - intersection_stop, - boundry - intersection_stop), (inner_lane - intersection_stop*2, - boundry - intersection_stop), (inner_lane - intersection_stop*2, - boundry))
        self.sim.create_quadratic_bezier_curve((inner_lane - intersection_stop*2, - boundry), (inner_lane - intersection_stop*2, -outer_lane), (inner_lane - intersection_stop*3, -outer_lane))
        
        #From East to South
        self.sim.create_quadratic_bezier_curve((-boundry, -inner_lane), (-boundry - intersection_stop, -inner_lane), (-boundry - intersection_stop, -inner_lane + intersection_stop))
        self.sim.create_quadratic_bezier_curve((-boundry - intersection_stop, -inner_lane + intersection_stop), (-boundry - intersection_stop, -inner_lane + intersection_stop*2), (-boundry, -inner_lane + intersection_stop*2))
        self.sim.create_quadratic_bezier_curve((-boundry, -inner_lane + intersection_stop*2), (-outer_lane, -inner_lane + intersection_stop*2), (-outer_lane, -inner_lane + intersection_stop*3))
        
        #From North to East
        self.sim.create_quadratic_bezier_curve((-inner_lane, boundry), (-inner_lane, boundry + intersection_stop), (-inner_lane + intersection_stop, boundry + intersection_stop))
        self.sim.create_quadratic_bezier_curve((-inner_lane + intersection_stop, boundry + intersection_stop), (-inner_lane + intersection_stop*2, boundry + intersection_stop), (-inner_lane + intersection_stop*2, boundry))
        self.sim.create_quadratic_bezier_curve((-inner_lane + intersection_stop*2, boundry), (-inner_lane + intersection_stop*2, outer_lane), (-inner_lane + intersection_stop*3, outer_lane))

        #From West to North
        self.sim.create_quadratic_bezier_curve((boundry, inner_lane), (boundry + intersection_stop, inner_lane), (boundry + intersection_stop, inner_lane - intersection_stop))
        self.sim.create_quadratic_bezier_curve((boundry + intersection_stop, inner_lane - intersection_stop), (boundry + intersection_stop, inner_lane - intersection_stop*2), (boundry, inner_lane - intersection_stop*2))
        self.sim.create_quadratic_bezier_curve((boundry, inner_lane - intersection_stop*2), (outer_lane, inner_lane - intersection_stop*2), (outer_lane, inner_lane - intersection_stop*3))
        
        

        #Connecting straights and left turns
            #paths 36-39
        #South
        self.sim.create_segment((inner_lane, -intersection_stop), (inner_lane, -boundry))
        #East
        self.sim.create_segment((-intersection_stop, -inner_lane), (-boundry, -inner_lane))
        #North
        self.sim.create_segment((-inner_lane, intersection_stop), (-inner_lane, boundry))
        #West
        self.sim.create_segment((intersection_stop, inner_lane), (boundry, inner_lane))

        #Connecting end of left turn to exit
            #paths 40-43
        #South
        self.sim.create_segment((-outer_lane, -inner_lane + intersection_stop*3), (-outer_lane, boundry+20))
        #East
        self.sim.create_segment((-inner_lane + intersection_stop*3, outer_lane), (boundry+20, outer_lane))
        #North
        self.sim.create_segment((outer_lane, inner_lane - intersection_stop*3), (outer_lane, -boundry-20))
        #West
        self.sim.create_segment((inner_lane - intersection_stop*3, -outer_lane), (-boundry-20, -outer_lane))

        #Regular vehicle driving
        self.vg = VehicleGenerator({
            #The first variable: 1 defines the weight if the vehicle; the higher the weight the more likely that type of vehicle will generate
            # 'path' defines the order of segments the vehicle will drive over
            #'v_max' defines the fastest speed a vehicle can drive at

            'vehicles': [
                #South [Inner Straight, Left Turn, Right Turn]
                (1, {'path': [0, 16, 12], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (225, 0, 0)}),
                (1, {'path': [0, 16, 36, 24 , 25 , 26, 43], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (225, 0, 0)}),
                (1, {'path': [1, 20, 11], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (225, 0, 0)}),
                #East [Inner Straight, Left Turn, Right Turn]
                (1, {'path': [2, 17, 14], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (0, 225, 0)}),
                (1, {'path': [2, 17, 37, 27 , 28 , 29, 40], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (0, 225, 0)}),
                (1, {'path': [3, 21, 13], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (0, 225, 0)}),
                #North [Inner Straight, Left Turn, Right Turn]
                (1, {'path': [4, 18, 8], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (225, 0, 0)}),
                (1, {'path': [4, 18, 38, 30 , 31 , 32, 41], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (225, 0, 0)}),
                (1, {'path': [5, 22, 15], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (225, 0, 0)}),
                #West [Inner Straight, Left Turn, Right Turn]
                (1, {'path': [6, 19, 10], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance , "colour": (0, 225, 0)}),
                (1, {'path': [6, 19, 39, 33 , 34 , 35, 42], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (0, 225, 0)}),
                (1, {'path': [7, 23, 9], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance, "colour": (0, 225, 0)}),
                ], 'vehicle_rate' : self.vehicle_rate*(1-self.self_driving_vehicle_proportion)

            
            })
        
            

        #adding both vehicle generators
        self.sim.add_vehicle_generator(self.vg)
        #self.sim.add_vehicle_generator(self.sdvg)








    #this function returns an instance of the simulation defined above
    def get_sim(self):
        return self.sim