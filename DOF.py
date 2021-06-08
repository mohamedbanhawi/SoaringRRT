from scipy.integrate import odeint
from numpy import *
from math import *
import pylab 
from pylab import *
from matplotlib.font_manager import FontProperties
import csv
import scipy.interpolate as intp

class RRT:
	def __init__(self):
		self.CRead("CL.csv","CD.csv")
		self.Wz=-2.5
		

	def CRead(self,file1,file2):
		print 'Reading CL/CD'
		with open(file1) as filein:
			reader = csv.reader(filein, quoting=csv.QUOTE_NONNUMERIC, skipinitialspace = True)
			self.alphaData1, self.CLData = zip(*reader)
		with open(file2) as filein:
			reader = csv.reader(filein, quoting=csv.QUOTE_NONNUMERIC, skipinitialspace = True)
			self.alphaData2, self.CDData = zip(*reader)
		self.CLf = intp.interp1d(self.alphaData1, self.CLData, kind='cubic')
		self.CDf = intp.interp1d(self.alphaData2, self.CDData, kind='cubic')
		print 'Done'

	def Coeff(self,alpha):
		if alpha>=20:
			result=(0.738675,0.324526)
		elif alpha<=-12:
			result=(-0.630536,0)
		else:
			CL=self.CLf(alpha)
			CD=self.CDf(alpha)
			result=(CL,CD)
		return result

	def eom(self,w,t,P):
		x,y,h,gamma,phi,V=w
		rho,m,S,g,AR,mu = P

		
		Cl,Cd=self.Coeff(rad2deg(atan2((V*sin(deg2rad(gamma))),(V*cos(deg2rad(gamma))))))
		f = [
		V*cos(deg2rad(gamma))*cos(deg2rad(phi)),
		V*cos(deg2rad(gamma))*sin(deg2rad(phi)),
		V*sin(deg2rad(gamma)),
		1/(m*V) * ((0.5*rho*V**2*S*(Cl))*cos(deg2rad(mu))-m*g*cos(deg2rad(gamma))),
		1/(m*V*cos(deg2rad(gamma)))*((0.5*rho*V**2*S*(Cl))*sin(deg2rad(mu))),
		1/m*(-((0.5)*rho*V**2*S*(Cd))-(m*g*sin(deg2rad(gamma))))
		]
		
		return f

	def dof(self):
		rho = 1.225
		m = 0.248
		S_wing = 0.16709644
		g = 9.81
		AR = 4.84750004249043
		mu=0

		# Required position
		x = 12
		y= 5
		hf = 30

		# Input params
		P=[rho,m,S_wing,g,AR,mu]

		# Time params
		stoptime = 10.0
		numpoints = 10
		t = [stoptime * float(i) / (numpoints - 1) for i in range(numpoints)]



		abserr = 1.0e-8
		relerr = 1.0e-6

		# Solve ODE 
		# Runs between -25 and 25 degrees for every 5 degrees increment
		w0= [0,0,20,0,0,1]
		print w0
		wsol = odeint(self.eom, w0, t, args=(P,),atol=abserr, rtol=relerr)
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
# 		plot(t, E, 'c', linewidth=lw)
		legend((r'$V$', r'$gamma$',r'$phi$',r'$x$',r'$y$',r'$h$',r'$E$'), prop=FontProperties(size=16))
		pylab.show()
	
A=RRT()
A.dof()