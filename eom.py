from numpy import *
from scipy.integrate import ode
import csv
import scipy.interpolate as intp
from math import *


class solver:
	def __init__(self,file1,file2):
		with open(file1) as filein:
			reader = csv.reader(filein, quoting=csv.QUOTE_NONNUMERIC, skipinitialspace = True)
			self.alphaData1, self.CLData = zip(*reader)
		with open(file2) as filein:
			reader = csv.reader(filein, quoting=csv.QUOTE_NONNUMERIC, skipinitialspace = True)
			self.alphaData2, self.CDData = zip(*reader)

	def Coeff(self,alpha):
		if alpha>=20:
			result=(0.738675,0.324526)
		elif alpha<=-12:
			result=(-0.630536,0)
		else:
			CLf = intp.interp1d(self.alphaData1, self.CLData, kind='cubic')
			CDf = intp.interp1d(self.alphaData2, self.CDData, kind='cubic')
			CL=CLf(alpha)
			CD=CDf(alpha)
			result=(CL,CD)
		return result

	def eom(self,t,w):
		x,y,h,gamma,phi,V=w
		
	#	Fixed Params
		rho = 1.225
		g = 9.81
	
	#	Aircraft Params
		m = 0.248
		S= 0.16709644
		AR = 4.84750004249043
		mu=0

		Wy,Wz=0,0
		Cl,Cd=self.Coeff(rad2deg(atan2((V*sin(deg2rad(gamma))),(V*cos(deg2rad(gamma))))))

	##	Simplified model
		f = [
		V*cos(deg2rad(gamma))*cos(deg2rad(phi)),
		V*cos(deg2rad(gamma))*sin(deg2rad(phi))-Wy,
		V*sin(deg2rad(gamma))+Wz,
		1/(m*V) * ((0.5*rho*V**2*S*(Cl))*cos(deg2rad(mu))-m*g*cos(deg2rad(gamma))),
		1/(m*V*cos(deg2rad(gamma)))*((0.5*rho*V**2*S*(Cl))*sin(deg2rad(mu))),
		1/m*(-((0.5)*rho*V**2*S*(Cd))-(m*g*sin(deg2rad(gamma))))
		]
	
		return f
		
	def solve(self):
		t_final = 10.0
		dt = 0.25

		y0 = [0,0,20,0,0,1]
		t0 = 0.0

		y_result = []
		t_output = []

		# Initialisation:

		backend = "dopri5"

		solver = ode(self.eom)
		solver.set_integrator(backend)  # nsteps=1
		solver.set_initial_value(y0, t0)

		y_result.append(y0)
		t_output.append(t0)

		while solver.successful() and solver.t < t_final:
			solver.integrate(solver.t + dt, step=1)
	
			y_result.append(solver.y)
			t_output.append(solver.t)
	
		y_result = array(y_result)
		t_output = array(t_output)

		print y_result

s=solver("CL.csv","CD.csv")
s.solve()
