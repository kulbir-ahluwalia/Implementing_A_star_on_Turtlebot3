import matplotlib.pyplot as plt
import numpy as np
import math

fig, ax = plt.subplots()

def plot_curve(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.033
    L = 0.160
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180

# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes

    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += r * (UL + UR) * math.cos(Thetan) * dt
        Yn += r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        plt.plot([Xs, Xn], [Ys, Yn], color="blue")

    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan
    

actions=[[5,5],[5,0],[0,5],[5,10],[10,5]]
        
for action in actions:
   X1= plot_curve(0,0,45, action[0],action[1]) # (0,0,45) hypothetical start configuration
   for action in actions:
        X2=plot_curve(X1[0],X1[1],X1[2], action[0],action[1])

plt.grid()

ax.set_aspect('equal')

plt.xlim(0,10)
plt.ylim(0,10)

plt.title('How to plot a vector in matplotlib ?',fontsize=10)

plt.show()
plt.close()
    
