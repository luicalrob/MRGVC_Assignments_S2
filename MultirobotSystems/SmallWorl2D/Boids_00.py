## A solution to the Boids exercise
## Version: 20240215
## Author: Luis Calderon, Andres Martinez
## License: CC-BY-SA

from time import time, localtime, strftime
from random import uniform
import random
import numpy as np
from point2d import Point2D 
# YOU MIGHT FIND USEFUL FUNCTIONS IN shapely, Point2D, AND gadgETs
from SmallWorl2D import Space, KPIdataset, Obstacle, MoBody, Mobot, Soul, Food, AniBody
from math import pi

class Boid(Soul):

    # YOUR AUXILIARY FUNCTIONS
    def __init__(self,body,T,dd):  # ADD YOUR ARGUMENTS
        self.control="MDMC"
        self.potential = "Lennard" # set "Lennard" for Ferrante potential, "cubic" for own version
        self.dd=dd
        self.type = T
        self.noise=0.1
        self.K=[0.25, 0.01, 0.25]
        self.u_min = 0.1
        self.saturation = 10 # force saturation

        # Ferrante
        self.alpha=2
        self.epsilon=1.5
        # Custom
        self.slope=3.5

        # YOUR BOID INIT CODE
        super().__init__(body,T)

    def magnitude_p(self, d):
        p = -4*self.alpha*self.epsilon / d
        p = p * (2*pow(self.noise/d, 2*self.alpha) - pow(self.noise/d, self.alpha))
        return p
    
    def magnitude_cubic_p(self, d):
        p = self.slope / (self.dd * (self.noise * 10.0))
        p = p * pow(d-self.dd, 3) 
        return p

    def proximal_control(self, b):
        p = 0
        #nearby = b.space.RnB(b.index(),(type(b),Mobot), b.space.R)
        measurements = b.space.RnB(b.index(),(type(b),Mobot), b.space.R)
        neighbours_number = len(measurements)

        for measurement in measurements: 
            
            #Do not contribute with any force is the neighbour is already moreless at desired distance
            if(abs(measurement[0]-self.dd) < 0.1): continue

            if(self.potential == "Lennard"):
                magnitude = self.magnitude_p(measurement[0])
            elif(self.potential == "cubic"):
                magnitude = self.magnitude_cubic_p(measurement[0])
            
            #Saturate the magnitude of repulsive and attractive forces
            if(magnitude > self.saturation):
                magnitude = self.saturation
            elif(magnitude < -self.saturation):
                magnitude = -self.saturation
            
            p += magnitude*np.exp(1j * measurement[1])
            
        return p
        # if neighbours_number == 0:
        #     return 0
        # else:
        #     return p/neighbours_number
    
    def allignment_control(self, b):
        a = np.exp(1j * b.th)
        for measurement in b.space.RnB(b.index(),(type(b),Mobot), 2*b.space.R):
            a += np.exp(1j * measurement[1])
        if(a!=0): a = a / abs(a)
        return a
    
    def mdmc(self, f):
        u=self.K[0]*f.real + self.u_min
        w=self.K[1]*f.imag
        return u,w
    
    def mimc(self, b, f):
        ## Method in paper
        # o = np.cos(b.th) + 1j * np.sin(b.th)
        # orientation_error = np.dot(o, f / abs(f))
        # if orientation_error >= 0:
        #     u = orientation_error * b.v_max
        # else:
        #     u = 0
        # w=self.K[2]*(o.imag - f.imag)
        
        ## Method in slides
        s = f / abs(f)
        u = max(0.0, s.real)*b.v_max
        w = self.K[2]*np.arctan2(s.imag, s.real)
        return u,w
        
    def update(self):
        if super().update():
            b=self.body
            i=b.index()
            s=b.space

            p=self.proximal_control(b)
            a=self.allignment_control(b)
            if self.type: #this robot is informed
                goal = [3.0, 7.0]
                xg_robot = (goal[0] - b.pos.x)*np.cos(b.th) + (goal[1] - b.pos.y)*np.sin(b.th)
                yg_robot = -(goal[0] - b.pos.x)*np.sin(b.th) + (goal[1] - b.pos.y)*np.cos(b.th)
                magnitude = np.sqrt(pow(xg_robot,2) + pow(yg_robot,2))
                theta = np.arctan2(yg_robot, xg_robot)
                g = magnitude*np.exp(1j*theta)
                #g = g / abs(g)
            else: 
                g=0.0

            f=p+a+g
            
            if self.control == "MDMC":
                v, w = self.mdmc(f)
            else:
                v, w = self.mimc(b,f)
            
            # # YOUR BOID UPDATE CODE

            b.cmd_vel(v = v, w = w, vth = 0)
            
    # actualizar todo lo que haga falta, distancias y todo para ver el comportamiento que tiene que tener cada uno

def set_mobot_formation(i, s, center, large, th=0, fc=(0.2, 0.2, 0), v=0, v_max=None, w_max=None):
    if v_max is None:
        v_max = s.vN / 2
    if w_max is None:
        w_max = s.wN

    pos = (uniform(-large/2,large/2) + center[0], uniform(-large/2,large/2) + center[1])
    return Mobot(s, 'm' + str(i), pos=pos, th=th, fc=fc, v=v, v_max=v_max, w_max=w_max)

def init():

    ## Create Data Structures
    name='Boids_'+strftime("%Y%m%d%H%M", localtime())
    global s, N, R
    dd=2 # or whatever
    R=1.8*dd
    s=Space(name,R=R,limits='',visual=True, showconn=False, ConnCtrl=0)
    KPIdataset(name,s,[1,1,0],[(0,'.y'),(1,'.k'),(2,'.g')])
        # 0 simulation time scale -- recommended "default" KPI
        # 1 Fraction remaining
        # 2 Fraction in largest group
        # ...

    ## Populate the world

    # N Mobots
    #N=25
    N=40
    i=0

    # limits in X = +-16, limits in Y = +-9
    posX = -6
    posY = 0
    large = 5
    iterations = 0
    while i<N and iterations<250:
        new=set_mobot_formation(i, s, center=(posX, posY), large=large, v_max=0.8, w_max=np.pi/2)
        # en vez de posicionar aleatoriamente, poner todos juntitos y con velocidad nula, ya pondremos command vel en el boid
        if s.fits(new,s.room,safe=0.35):
            s.bodies.append(new)
            # YOUR BOID PARAMETRIZATION, DIFFERENT KINDS?
            mobot_type = random.random() < 0.35
            Boid(new, mobot_type, dd)
            # creamos un comportamiento "alma" para cada robot
            i += 1
        iterations += 1

    # and several Obstacles, or Killers, or whatever
            
    # Five Obstacles
    # i=0
    # while i<5:
    #     new=Obstacle(s,'O'+str(i),pos=(uniform(-s.W,s.W),uniform(-s.H,s.H)),th=uniform(-pi,pi))
    #     if s.fits(new,s.room,safe=1):
    #         s.bodies.append(new)
    #         i += 1

    # init distances matrix and connections graph
    s.dist=np.zeros((len(s.bodies),len(s.bodies))) # distances between centroids in a np.matrix
    s.update_dist()
    s.update_conn()

    # WHATEVER ELSE YOU WANT TO DO INITIALLY, E.G., REPORT ON LOG

init()
end=False
while not end: # THE loop 
    s.step()

    # COLLISION MANAGEMENT
    ko=[]
    # YOUR COLLISION DETECTION CODE 
    for b in s.bodies:
        i = b.index()
        if(i>=0):
            if isinstance(b,Obstacle):
                    ko+=s.incontact(i,AniBody)
            elif isinstance(b,Mobot):
                    ko+=s.incontact(i,(Food,Mobot))
    s.remobodies(ko,'collision')
    # si se chocan quitarlos

    # KPIs
    KPI=[s.time/(time()-s.t0),0,0]
    KPI[2] = 0
    graph = s.conn_subgraph(Mobot)
    for b in s.bodies:
        if b.on:
            if isinstance(b,Mobot):
                #remaining robots
                KPI[1] += 1
                #members in largest group
                group = 1 + len(graph[b.index()])
                if group > KPI[2]:
                    KPI[2] = group
                
    KPI[1]/=N
    KPI[2]/=N
    # la idea es actualizar el mapa y toda la informacion, los comportamientos y todo a parte de la visualizacion
    # YOUR KPI COMPUTATIONS
    s.KPIds.update(KPI)

    end=s.has_been_closed() or s.KPIds.KPI[1]==0 or s.time>60 # or whatever
s.close()