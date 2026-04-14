import numpy as np
import scipy.optimize as opt


data = np.loadtxt(".\\Tire_Data\\Slip_Angle_Data.csv",delimiter=",")

angle = data[:,0]
normalized_fy = data[:,1]

# Calculates 1-r^2 for the fit with B, C, and D
def f(x):
    B = x[0]
    C = x[1]
    D = x[2]
    fit = D*np.sin(C*np.atan(B*angle*np.pi/180))
    rsum = np.sum((fit-normalized_fy)**2)
    msum = np.sum((normalized_fy-np.mean(normalized_fy))**2)
    return rsum/msum 

    


xmin = opt.minimize(f,[10,1.5,2.3])
print(xmin)