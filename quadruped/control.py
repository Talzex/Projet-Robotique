import math
import re
import numpy as np
import scipy
from scipy import optimize
import matplotlib.pyplot as pyplot
import interpolation as intp
def sandbox(t):
    """
    python simulator.py -m sandbox

    Un premier bac à sable pour faire des expériences

    La fonction reçoit le temps écoulé depuis le début (t) et retourne une position cible
    pour les angles des 12 moteurs

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    # Par exemple, on envoie un mouvement sinusoidal
    targets = [0]*12

    targets[1] = np.sin(-90)
    targets[2] = np.sin(-10)

    targets[4] = np.sin(-90)
    targets[5] = np.sin(-10)

    targets[7] = np.sin(-90)
    targets[8] = np.sin(-10)

    targets[10] = np.sin(-90)
    targets[11] = np.sin(-10)

    targets[0] = np.sin(t)
    targets[3] = np.sin(t)

    targets[6] = np.cos(t)
    targets[9] = np.cos(t)
    


    return targets

def direct(alpha, beta, gamma):
    """
    python simulator.py -m direct

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument la cible (alpha, beta, gamma) des degrés de liberté de la patte, et produit
    la position (x, y, z) atteinte par le bout de la patte

    - Sliders: les angles des trois moteurs (alpha, beta, gamma)
    - Entrées: alpha, beta, gamma, la cible (radians) des moteurs
    - Sortie: un tableau contenant la position atteinte par le bout de la patte (en mètres)
    """
    l1, l2, l3 = 45/1000, 65/1000, 87/1000
    xp = l1 + math.cos(beta)*l2 + math.cos(beta - gamma)*l3
    yp = math.sin(beta)*l2 + math.sin(beta - gamma)*l3
    
    x = math.cos(alpha) * xp 
    y = math.sin(alpha) * xp 
    z = yp
    return [x,y,z]

def inverse(x, y, z):
    """
    python simulator.py -m inverse

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Sliders: la position cible x, y, z du bout de la patte
    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    l1, l2, l3 = 45/1000, 65/1000, 87/1000
    #Alpha
    alpha = np.arctan2(y,x)

    #Gamma
    moteur_a = np.array([l1*np.cos(alpha),l1*np.sin(alpha),0])
    point = np.array([x,y,z])
    dist_ac = np.linalg.norm(point-moteur_a) #Distance du moteur au point C

    division = ((l2**(2)+l3**(2)-dist_ac**(2)))/(2*l2*l3)
    if(division > 1):
        division = 1
    if(division < -1):
        division = -1

    gamma = -np.arccos(division)+np.pi

    #print(gamma)

    #Beta
    epsilon = np.arcsin(z/dist_ac)
    divisionsigma = (l2**(2)+dist_ac**(2)-l3**(2))/(2*l2*dist_ac)
    if(divisionsigma > 1):
        divisionsigma = 1
    if(divisionsigma < -1):
        divisionsigma = -1

    sigma = np.arccos(divisionsigma)
    

    beta = sigma + epsilon
    return [alpha,beta,gamma]


def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    temps = 3
    spline = intp.LinearSpline3D()
    spline.add_entry(0,            0.15,  0.123,     -0.123)
    spline.add_entry(temps/3,      0.15, -0.123,     -0.123)
    spline.add_entry(temps*2/3,    0.15,      0,          0)
    spline.add_entry(temps,        0.15,  0.123,     -0.123)

    v = spline.interpolate(math.fmod(t,temps))
    return inverse(v[0],v[1],v[2])


def legs(leg1, leg2, leg3, leg4):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    targets = [0]*12

    pi = np.pi

    teta = -3*(pi/4)
    newLeg1 = ((leg1[0]*np.cos(teta)-leg1[1]*np.sin(teta))-0.04, leg1[0]*np.sin(teta) + leg1[1]*np.cos(teta), leg1[2])
    
    teta = 3*(pi/4)
    newLeg2 = ((leg2[0]*np.cos(teta)-leg2[1]*np.sin(teta))-0.04, leg2[0]*np.sin(teta) + leg2[1]*np.cos(teta), leg2[2])

    teta = pi/4
    newLeg3 = ((leg3[0]*np.cos(teta)-leg3[1]*np.sin(teta))-0.04, leg3[0]*np.sin(teta) + leg3[1]*np.cos(teta), leg3[2])
    
    teta = -pi/4
    newLeg4 = ((leg4[0]*np.cos(teta)-leg4[1]*np.sin(teta))-0.04, leg4[0]*np.sin(teta) + leg4[1]*np.cos(teta), leg4[2])

    leg1 = inverse(newLeg1[0], newLeg1[1], newLeg1[2])
    leg2 = inverse(newLeg2[0], newLeg2[1], newLeg2[2])
    leg3 = inverse(newLeg3[0], newLeg3[1], newLeg3[2])
    leg4 = inverse(newLeg4[0], newLeg4[1], newLeg4[2])

    targets = [0]*12

    targets[0] = leg1[0]
    targets[1] = leg1[1]
    targets[2] = leg1[2]
    targets[3] = leg2[0]
    targets[4] = leg2[1]
    targets[5] = leg2[2]
    targets[6] = leg3[0]
    targets[7] = leg3[1]
    targets[8] = leg3[2]
    targets[9] = leg4[0]
    targets[10] = leg4[1]
    targets[11] = leg4[2]


    return targets

def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    targets = [0]*12

    temps = 3
    spline = intp.LinearSpline3D()
    spline.add_entry(0,            0, 0.167, -0.104)
    spline.add_entry(temps/3,      0, 0.167, -0.52)
    spline.add_entry(temps*2/3,    -0.104, 0.139, -0.104)
    spline.add_entry(temps,        0, 0.167, -0.104)

    spline2 = intp.LinearSpline3D()
    spline2.add_entry(0,            0 ,-0.126 ,-0.104)
    spline2.add_entry(temps/3,      0 ,-0.126 ,-0.52)
    spline2.add_entry(temps*2/3,    -0.104,  -0.145, -0.098)
    spline2.add_entry(temps,        0 ,-0.126 ,-0.104)

    spline3 = intp.LinearSpline3D()
    spline3.add_entry(0,            0.145, -0.104, -0.104)
    spline3.add_entry(temps/3,      0.145, -0.104, -0.52)
    spline3.add_entry(temps*2/3,    0, -0.158, -0.104)
    spline3.add_entry(temps,        0.145, -0.104, -0.104)

    spline4 = intp.LinearSpline3D()
    spline4.add_entry(0,            0.145, 0, -0.104)
    spline4.add_entry(temps/3,      0.145, 0, -0.52)
    spline4.add_entry(temps*2/3,    0, 0.164, -0.104)
    spline4.add_entry(temps,        0.145, 0, -0.104)

    v1 = spline.interpolate(math.fmod(t,temps))
    v2 = spline2.interpolate(math.fmod(t,temps))
    v3 = spline3.interpolate(math.fmod(t,temps))
    v4 = spline4.interpolate(math.fmod(t,temps))

    
    return legs(v1,v2,v3,v4)

if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")