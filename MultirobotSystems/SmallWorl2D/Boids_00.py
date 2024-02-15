## A solution to the Boids exercise
## Version: 20240215
## Author: Luis Calderon, Andres Martinez
## License: CC-BY-SA

from time import time, localtime, strftime
from random import uniform
import numpy as np
from point2d import Point2D 
# YOU MIGHT FIND USEFUL FUNCTIONS IN shapely, Point2D, AND gadgETs
from SmallWorl2D import Space, KPIdataset, Obstacle, MoBody, Mobot, Soul

class Boid(Soul):

    # YOUR AUXILIARY FUNCTIONS

    def __init__(self,body,T,dd):  # ADD YOUR ARGUMENTS
        self.dd=dd
        # YOUR BOID INIT CODE
        super().__init__(body,T)

    def update(self):
        if super().update():
            b=self.body
            i=b.index()
            s=b.space
            # YOUR BOID UPDATE CODE

    # actualizar todo lo que haga falta, distancias y todo para ver el comportamiento que tiene que tener cada uno

def init():

    ## Create Data Structures
    name='Boids_'+strftime("%Y%m%d%H%M", localtime())
    global s, N, R
    dd=1 # or whatever
    R=2
    s=Space(name,R=R,limits='hv',visual=True,showconn=True)
    KPIdataset(name,s,[1,1,0],[(0,'.y'),(1,'.k'),(2,'.g')])
        # 0 simulation time scale -- recommended "default" KPI
        # 1 Fraction remaining
        # 2 Fraction in largest group
        # ...

    ## Populate the world

    # N Mobots
    N=25
    i=0
    while i<N:
        new=Mobot(s,'m'+str(i),pos=(uniform(-s.W,s.W),uniform(-s.H,s.H)),th=uniform(-np.pi,np.pi),fc=(0.8,0.8,1),v=1,v_max=s.vN/2,w_max=s.wN) 
        # en vez de posicionar aleatoriamente, poner todos juntitos y con velocidad nula, ya pondremos command vel en el boid
        if s.fits(new,s.room,safe=s.R):
            s.bodies.append(new)
            # YOUR BOID PARAMETRIZATION, DIFFERENT KINDS?
            Boid(new, 0, dd)
            # creamos un comportamiento "alma" para cada robot
            i += 1

    # and several Obstacles, or Killers, or whatever

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
    s.remobodies(ko,'collision')
    # si se chocan quitarlos

    # KPIs
    KPI=[s.time/(time()-s.t0),0,0]
    for b in s.bodies:
        if b.on:
            if isinstance(b,Mobot):
                KPI[1] += 1
    KPI[1]/=N
    KPI[2]/=N
    # la idea es actualizar el mapa y toda la informacion, los comportamientos y todo a parte de la visualizacion
    # YOUR KPI COMPUTATIONS
    s.KPIds.update(KPI)

    end=s.has_been_closed() or s.KPIds.KPI[1]==0 or s.time>60 # or whatever
s.close()