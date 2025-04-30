import math
import numpy as np
import random

#Robots physical constants
S = 10**2 * np.array(([0.0477,-0.0416],[0.0477,0.0416])) # seonsors position in the torso reference frame
dZeta=math.atan2(S[0,1],S[0,0]); # angular position of the sensor respect to the torso reference frame
Zeta = np.array([-dZeta , dZeta ]);
Nsectors = len(Zeta) # number of sensors
TOO_FAR = 250 #  70 cm is the limit range distance of the Master sonar sensors                       
dc = np.zeros((Nsectors,1),'double') # initialize perceived distance vector
tandZeta = math.tan(dZeta/2)
cm = 1 # 0.01 # if expressing in meters
degree = math.pi / 180.0  # radians per degree

#Parameters for dynamical systems
Q = 0.0001 # stocastic component

def cmp(a, b): # python 3 implementation of python2 function
    return (a > b) - (a < b)


def FTarget(target_distance, target_angle):
    Ktar=0.5 #2.4
    beta=100

    #Ftar =Ktar*sin(HeadYaw[0])
    #if targetDirection !=-1000:
    #   Ftar =exp(-(abs(dist-60)**2)/1000)*Ktar*sin(HeadYaw[0])+Ftar   #if targetDirection not available use headorientation instead 
    #else:
    #   Ftar =exp(-(abs(dist-60)**2)/1000)*Ktar*sin(targetDirection)*2 #uses targetDirection if available
       
    #Ftar =(1+math.exp(-(abs(target_distance-6)**2)/beta))*Ktar*math.sin(target_angle)
    Ftar =Ktar*math.sin(target_angle)

    return Ftar

def FObstacle(obs_distance,obs_angle):
        #######################################################################
        # Define Force Obstacle here
        #######################################################################


    TOO_FAR = 10*cm # max sonar distance
    Kobs = 10
    beta2 = 10*cm #beta2 = 80
    
    sonar_beam_width= 60.0*degree

    # the angle of a 40cm (Rrobot=20*cm) object at distance obs_distance + Rrobot
    Rrobot =3*cm # robot footprint in sim world
    object_width = 10*cm
    # if obs_distance <= 0:
    #     sigma = math.pi
    # else:
    #     sigma = math.atan( object_width/(obs_distance*cm))
    sigma = math.pi/3
    if obs_distance <= TOO_FAR:
        w_obs = Kobs*math.exp(-(obs_distance-Rrobot)/beta2)
    else:
        w_obs = 0.0
        
    Fobsi = -w_obs*(obs_angle)*(math.exp(-obs_angle**2/(2*sigma**2)))
 
    return Fobsi

def FStochastic():
    ############################################################################
    #  Stochastic force
    ############################################################################
    Qphi=0.001
    
    Fstoch =math.sqrt(Qphi)*random.randint(1,100)* 10**(-2)
    return Fstoch

def check_collision(SonarLeft, SonarRight, dtheta, Fstoch, SonarCusum):
    # evaluate whether the robot is facing a parallel object
    #dtheta=( Ftar+fobsi_L+fobsi_R)*dt
    #print " dtheta" , dtheta
    if (math.fabs(SonarLeft-SonarRight)<10*cm and SonarLeft<40*cm and SonarRight<40*cm):
        SonarCusum=SonarCusum+10
        Fstoch= cmp(dtheta,0)*SonarCusum*Fstoch #probably easier to use np.sign()?
    else:
                # was used to switch between aligning and approaching # SonarFlag=0
        SonarCusum=60
        #? countFstoch=0
    return Fstoch, SonarCusum


def FOrienting():
    Forient = 0.0
    return Forient

def compute_velocity(SonarLeft,SonarRight):
    #dx=0.5 # fixed forward velocity
    #variable forward velocity
    #dx=0.5*exp(-(math.fabs(dtheta/5)))
    vx=5.0* math.exp(-(10/(SonarLeft+SonarRight+10**(-3))))
    vy=0.0
#    frequency=0.2 # fixed robot step frequency

#     if(SonarLeft<30*cm and SonarRight<30*cm):
# #        print " stop because obstacle is too close"
#         dx =0
#         dy =0
#         dtheta=0
#         frequency=0
    
    return vx

def compute_turnrate(target_dist, target_angle, sonar_distance_left, sonar_distance_right):
    max_turnrate = 0.349  # rad/s # may need adjustment!

    delta_t = 1  # may need adjustment!
    sonar_angle_left = 30 * degree
    sonar_angle_right = -30 * degree

    Fobs_left = FObstacle(sonar_distance_left, sonar_angle_left)
    Fobs_right = FObstacle(sonar_distance_right, sonar_angle_right)

    FTotal = FTarget(target_dist, target_angle) + \
             Fobs_left + \
             Fobs_right + \
             FOrienting() + \
             FStochastic()

    # turnrate: d phi(t) / dt = sum( forces )
    turnrate = FTotal * delta_t

    # normalise turnrate value
    turnrate = turnrate / max_turnrate
    if turnrate > max_turnrate:
        turnrate = 1.0
    elif turnrate < -max_turnrate:
        turnrate = -1.0
        
    return turnrate

    # maxdtheta=0.8 #0.68
        # if dt<0.01:
        #     dt=0.01
        # if dt>5:
        #     dt=5
        #
        # #dtheta=( Ftar+Fobs_left + Fobs_right +Fstoch-0.1)*dt # added a constant term to correct the bias to go left
        # dtheta=( Ftar + Fobs_left + Fobs_right + Fstoch)*dt
        #
        # # translating dtheta in values for the robot API
        # if dtheta>maxdtheta:
        #     dtheta=maxdtheta
        # elif dtheta<(-maxdtheta):
        #     dtheta= -maxdtheta
        #
        # # dtheta=dtheta/maxdtheta
        # return dtheta

if __name__=="__main__":
    pass
