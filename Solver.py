from scipy.integrate import odeint
import DOF
from numpy import *
import pylab 
from pylab import *
from matplotlib.font_manager import FontProperties
import CRead

CRead=CRead.CReader("CL.csv","CD.csv")

rho = 1.225
m = 0.248
S_wing = 0.16709644
S_cross = 0.03
g = 9.81
AR = 4.84750004249043
Wz = 5
alpha=rad2deg(arctan2(Wz,5))
Cl = CRead.Coeff(alpha)[0]
Cd = CRead.Coeff(alpha)[1]
mu=10

P=[rho,m,S_wing,S_cross,g,AR,Cl,Cd,Wz,mu]

# Required position
x = 12
y= 5
hf = 30

# Input params
P=[rho,m,S_wing,S_cross,g,AR,Cl,Cd,Wz,mu]

# Time params
stoptime = 5.0
numpoints = 200
t = [stoptime * float(i) / (numpoints - 1) for i in range(numpoints)]



abserr = 1.0e-8
relerr = 1.0e-6

# Solve ODE 
# Runs between -25 and 25 degrees for every 5 degrees increment
w0= [5,0,40,5,0,5]
wsol = odeint(DOF.eom, w0, t, args=(P,),atol=abserr, rtol=relerr)
x=[]
y=[]
h=[]
dist=[]
gamma=[]
phi=[]
V=[]
time=[]
check=0
for t1, w1 in zip(t, wsol):
	x.append(w1[0])
	y.append(w1[1])
	h.append(w1[2])
 	dist.append(sqrt((w1[0])**2+(w1[1])**2+(w1[2])**2))
	gamma.append(w1[3])
	phi.append(w1[4])
	V.append(w1[5])
	time.append(t1)
# tolerance of +-2m
# Reject if stall
# 	for k in range(0,len(x)):
# 		if x+2>x[k] > x-2 and y+2>y[k] > y-2 and V[k]>5 and hf+2>h[k]> hf-2:
# 			check=1
# 			break
# 	if check==1:
# 		break		
E=[]
for i in range(0,len(h)):
	E.append(h[i]+V[i]**2/(2*g))



figure(1)
xlabel('t')
grid(True)
hold(True)
lw = 1
plot(t, V, 'b', linewidth=lw)
plot(t, gamma, 'g', linewidth=lw)
plot(t, phi, 'g', linewidth=lw)
plot(t, x, 'r', linewidth=lw)
plot(t, y, 'r', linewidth=lw)
plot(t, h, 'y', linewidth=lw)
plot(t, E, 'c', linewidth=lw)
legend((r'$V$', r'$gamma$',r'$phi$',r'$x$',r'$y$',r'$h$',r'$E$'), prop=FontProperties(size=16))
pylab.show()