#!/usr/bin/env python
#
#
# RRT Implementation in Python
# by Mohamed Elbanhawi (Ver 0.1 September 2013)
# Version 4.0  - May 2014
#
# This is a three dimensional implementation of RRT algorithm
#
# Building model and modified obstacle collision implemented by Chung (April 2014)
# 
# To do list:
#--------------
# -Plot energy -- done
# -Distance metric --done
#- Thresholds --done
#- Energy ==0 --done
#- NN search 
#- Truncation
#- BB search
#- Dynamic feasibility
#
# 
#--------------------------------------Libraries------------------------------------------
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy import sin,cos,arcsin,radians,degrees,arctan2,sqrt,arctan,square
import matplotlib.pyplot as plt
from math import *
import math
from pylab import *
import random
from random import *
from openpyxl import load_workbook
import os
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
from scipy.integrate import odeint
from numpy import *
import pylab 
from matplotlib.font_manager import FontProperties
import time
import csv
from scipy.spatial import cKDTree
import scipy.interpolate as intp

#--------------------------------------Classes--------------------------------------------
#Environment and Obstacles
class env3d:

#environment class is defined by obstacle vertices and boundaries
	def __init__(self,xmin,xmax,ymin,ymax,zmin,zmax):
		self.xmin=xmin
		self.xmax=xmax
		self.ymin=ymin
		self.ymax=ymax
		self.zmin=zmin
		self.zmax=zmax
		
#Collision checking for a path
	def inobstacle(self,x1,y1,z1,x2,y2,z2,poly):
	# Return the length of poly (number of sides)
		x = x2
		y = y2
		z = z2
		poly = poly
		
		for item in poly:
 			sides = len(item[0])
 			c = 1
		#p1x and p1y are the initial point of the polygon\
			i = 0
			while i < sides-1:
				p1x,p1y,pzl= item[0][i],item[1][i],item[2][0]
				if len(item[3])==1:
					p1zh=item[3][0]
					p2zh=item[3][0]
				else:
					p1zh=item[3][i]
					p2zh=item[3][i+1]
				p2x,p2y=item[0][i+1],item[1][i+1]
				for j in xrange(0,101):
					u=j/100.0
					x=x1*u+x2*(1-u)
					y=y1*u+y2*(1-u)
					if y > min(p1y,p2y):
						if y<= max(p1y,p2y):
							if x <= max(p1x,p2x):
								if z > pzl:
									if z <= max(p1zh,p2zh):
										if p1y != p2y:
											xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
										if p1x == p2x or x <= xinters:
											c = 0
											break
				i = i+1
			if c ==0: break
		return c

#check if newly added sample is in the free configuration space
	def isfree(self,poly):
		n= G.number_of_nodes()-1
		(x,y,z)= (G.x[n], G.y[n], G.z[n]) 
		for item in poly:
 			sides = len(item[0])
 			c = 1
# 		#p1x and p1y are the initial point of the polygon\
			i = 0
			while i < sides-1:
				p1x,p1y,pzl = item[0][i],item[1][i],item[2][0]
				if len(item[3])==1:
					p1zh=item[3][0]
					p2zh=item[3][0]
				else:
					p1zh=item[3][i]
					p2zh=item[3][i+1]
				p2x,p2y = item[0][i+1],item[1][i+1]
				if y > min(p1y,p2y):
					if y<= max(p1y,p2y):
						if x <= max(p1x,p2x):
							if z > pzl:
								if z <= max(p1zh,p2zh):
									if p1y != p2y:
										xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
									if p1x == p2x or x <= xinters:
										G.remove_node(n)
										c = 0
										break
				i = i+1
			if c ==0: break
		return c
				
#check if current node is in goal region
	def ingoal(self):
		n= G.number_of_nodes()-1
		(x,y,z)= (G.x[n], G.y[n],G.z[n]) 
		if (x>=xgmin) and (x<=xgmax) and (y>=ygmin) and (y<=ygmax) and (z>=zgmin) and (z<=zgmax) :
			return 1
		else:
			return 0
	
#draw the edges of a the Polygon
	def cubedraw(self,obsx,obsy,obszl,obszh,k):
		x = obsx
		y = obsy
		zl = obszl*(len(obsx))
		if len(obszh)==1:
			zh = obszh*(len(obsx))
		else:
			zh = obszh
  		ax.plot(x, y, zl,k)
  		
  		ax.plot(x, y, zh,k)
  		
 		for i in xrange (0,len(x)-1):
			obx = [x[i],x[i]]
			oby = [y[i],y[i]]
			obz = [zl[0],zh[i]]
			ax.plot(obx,oby,obz,k)
			
#Self define updraft regions for simple testing
	def updraft(self,x,y,z):
		#assume a cube shaped region
		xumin=0
		xumax=100
		yumin=self.ymin
		yumax=self.ymax
		zumin=5
		zumax=80
		if xumin<x<xumax and yumin<y<yumax and zumin<z<zumax:
			yv = 1.0
		else:
			yv = 0.0
		
		return yv				
			
#read x,y,zl,zh for each polygon from excel spreadsheet
	def read(self):	
		Polygons=[]
		wb = load_workbook(filename = 'P.xlsx' ,data_only=True)
		for Polygon in wb.worksheets:
			xList=[]
			yList=[]
			zlList=[]
			zhList=[]
			for row in Polygon.range('A1:A100'):
				for cell in row:
					if cell.value==None:
						break
					xList.append(cell.value)
			for row in Polygon.range('B1:B100'):
				for cell in row:
					if cell.value==None:
						break
					yList.append(cell.value)
			for row in Polygon.range('C1:C100'):
				for cell in row:
					if cell.value==None:
						break
					zlList.append(cell.value)
			for row in Polygon.range('D1:D100'):
				for cell in row:
					if cell.value==None:
						break
					zhList.append(cell.value)
			
			Polygons.append([xList,yList,zlList,zhList])
		return Polygons
						
#-----------------------------------------------------------------------------------------
class RRT3d:
	def __init__(self,nstart):
		(x,y,z)=nstart
		self.x=[]
		self.y=[]
		self.z=[]
		self.gamma=[]
		self.phi=[]
		self.Vi=[]
		self.yv=[]
		self.fail=[]
		self.parent=[]
		self.children=[]
		self.con=[]
		self.time=[]
		self.x.append(x)
		self.y.append(y)
		self.z.append(z)
		self.fail.append(0)
		self.con.append(0)
		self.children.append(0)
		#first node is the only node whose parent is itself
		self.parent.append(0)
		#self.yv.append(self.cfdmetric(x,y,z))
		self.CRead("CL.csv","CD.csv")
		self.yv.append(0)
		self.time.append(0)
		self.gamma.append(0)
		self.phi.append(0)
		
# 		csvFile = open("domain2.csv")
# 		self.data= numpy.loadtxt(csvFile, delimiter=',')
# 		self.coordinate=numpy.delete(self.data, numpy.s_[3:], 1)
		
		self.initialV=10
		self.Vi.append(self.initialV)
		
		self.Gd=1.0
		self.Gc =55.0
		self.Gv=1.2
		self.Gf=2.0
		self.Gz=1.5
		
	def cfdmetric(self,x,y,z):
		point=numpy.array([x,y,z])
		tree = cKDTree(self.coordinate, leafsize=self.coordinate.shape[0]+1)
		distances, ndx = tree.query([point], k=10)
		xdata=self.data[ndx[:10],[0]][0]
		ydata=self.data[ndx[:10],[1]][0]
		zdata=self.data[ndx[:10],[2]][0]
		vdata=self.data[ndx[:10],[3]][0]
			
		if (xdata[1:] == xdata[:-1]).all and (ydata[1:] == ydata[:-1]).all:
			vq=intp.griddata((zdata),vdata,(z),method='linear')
			
		elif (xdata[1:] == xdata[:-1]).all and (zdata[1:] == zdata[:-1]).all:
			vq=intp.griddata((ydata),vdata,(y),method='linear')
			
		elif (ydata[1:] == ydata[:-1]).all and (zdata[1:] == zdata[:-1]).all:
			vq=intp.griddata((xdata),vdata,(x),method='linear')
			
		elif (xdata[1:] == xdata[:-1]).all:
			vq=intp.griddata((ydata,zdata),vdata,(y,z),method='linear')
			
		elif (ydata[1:] == ydata[:-1]).all:
			vq=intp.griddata((xdata,zdata),vdata,(x,z),method='linear')
			
		elif (zdata[1:] == zdata[:-1]).all:
			vq=intp.griddata((xdata,ydata),vdata,(x,y),method='linear')
			
		else:
			vq=intp.griddata((xdata,ydata,zdata),vdata,(x,y,z),method='linear')
		return vq
		
	def CRead(self,file1,file2):
		print 'reading CL/CD'
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

		
	def eom(self,w,t,P):
		x,y,h,gamma,phi,V=w
# 		rho,m,S_wing,S_cross,g,AR,Cl,Cd,Wv,mu = P
		rho,m,S_wing,S_cross,g,AR,Wv,mu = P

		f = [
		V*cos(deg2rad(gamma))*cos(deg2rad(phi)),
		V*cos(deg2rad(gamma))*sin(deg2rad(phi)),
		V*sin(deg2rad(gamma))+Wv,
	# 	1/(m*V)*((0.5*rho*V**2*S_wing*)*cos(deg2rad(mu))-m*g*cos(deg2rad(gamma))),
# 		1/(m*V*cos(deg2rad(gamma)))*((0.5*rho*V**2*S_wing*Cl)*sin(deg2rad(mu))),
# 		1/m*(-((0.5)*rho*V**2*S_cross*Cd)-(m*g*sin(deg2rad(gamma))))
		1/(m*V)*((0.5*rho*V**2*S_wing*(self.Coeff(rad2deg(atan2((V*sin(deg2rad(gamma))+Wv),V*sin(deg2rad(gamma)))))[0]))*cos(deg2rad(mu))-m*g*cos(deg2rad(gamma))),
		1/(m*V*cos(deg2rad(gamma)))*((0.5*rho*V**2*S_wing*(self.Coeff(rad2deg(atan2((V*sin(deg2rad(gamma))+Wv),V*sin(deg2rad(gamma)))))[0]))*sin(deg2rad(mu))),
		1/m*(-((0.5)*rho*V**2*S_cross*(self.Coeff(rad2deg(atan2((V*sin(deg2rad(gamma))+Wv),V*sin(deg2rad(gamma)))))[1]))-(m*g*sin(deg2rad(gamma))))
		]
		return f
	
	def dof(self,initial,final):
	
# 		nodes param
		(x1,y1,z1)= (initial[0],initial[1],initial[2])
		(x2,y2,z2)= (final[0],final[1],final[2])
		
# 		Fixed params
		rho = 1.225
		g = 9.81
		Wz = initial[6]
		Vx = initial[5]*cos(deg2rad(initial[3]))
		
# 		aircraft params
		m = 0.248
		S_wing = 0.16709644
		S_cross = 0.03
		AR = 4.84750004249043
		stallV=5
# 		Cl = self.CRead.Coeff(rad2deg(atan2((initial[5]*sin(deg2rad(gamma))+Wz),Vx)))[0]
# 		Cd = self.CRead.Coeff(rad2deg(atan2(Wz,Vx)))[1]

# 		Control param
		(pitchMin,pitchMax)=(-25.0,25.0)
		pitchStep=5.0
		mu=0
		
# 		P=[rho,m,S_wing,S_cross,g,AR,Cl,Cd,Wz,mu]
		P=[rho,m,S_wing,S_cross,g,AR,Wz,mu]

# 		time params
		stoptime = 1
		numpoints = 100
		t = [stoptime * float(i) / (numpoints - 1) for i in xrange(numpoints)]
		
		
# 		ODE param
		abserr = 1.0e-8
		relerr = 1.0e-6
		
# 		Determining heading
		dx=x2-x1
		dy=y2-y1
# 		Estimate pitch angle
		a=rad2deg(atan2((z2-z1),sqrt((y2-y1)**2+(x2-x1)**2)))
		if dx>0:
			b=rad2deg(atan2(dy,dx))
		elif dx<0:
			if dy>0:
				b=90-rad2deg(atan2(dx,dy))
			elif dy<0:
				b=-90+rad2deg(atan2(dx,dy))
			elif dy==0:
				b=-180
		elif dx==0:
			if dy>0:
				b=90
			elif dy<0:
				b=-90
			elif dy==0:
				b=0
		
	# 	if  a>0:
# 			iRange=np.arange(25.0,0.0,-2.5)
# 		elif a<0:
# 			iRange=np.arange(-25.0,0.0,2.5)
# 		elif a==0:
# 			iRange=np.arange(5.0,-5.0,-1.0)
			
		if (b-initial[4])<pitchMin:
			jRange=initial[4]+pitchMin
		elif (b-initial[4])>pitchMax:
			jRange=initial[4]+pitchMax
		else:
			jRange=b
		
		iRange=np.arange(pitchMin,pitchMax+pitchStep,pitchStep)
		# jRange=np.arange((initial[4]-25),(initial[4]+25),2.5)
		
		dm=10000
		for i in iRange:
			w0= [initial[0],initial[1],initial[2],i,jRange,initial[5]]
			wsol = odeint(self.eom, w0, t, args=(P,),atol=abserr, rtol=relerr, mxstep=5000)
			x=[]
			y=[]
			h=[]
			gamma=[]
			phi=[]
			V=[]
			time=[]
			d2n=[]
			n2i=[]
			check=0
			for t1, w1 in zip(t, wsol):
				if w1[5]>stallV and w1[2]>1 and (sqrt((w1[0]-x1)**2+(w1[1]-y1)**2+(w1[2]-z1)**2))>0.5:
					x.append(w1[0])
					y.append(w1[1])
					h.append(w1[2])
					gamma.append(w1[3])
					phi.append(w1[4])
					V.append(w1[5])
					time.append(t1)
					d2n.append(sqrt((x2-w1[0])**2+(y2-w1[1])**2+(z2-w1[2])**2))
					n2i.append(sqrt((w1[0]-x1)**2+(w1[1]-y1)**2+(w1[2]-z1)**2))
			
			for k in xrange(0,len(x)):
				if d2n[k]<dm:
					dm=d2n[k]
					xn=x[k]
					yn=y[k]
					zn=h[k]
					gamman=gamma[k]
					phin=phi[k]
					vn=V[k]
					tn=time[k]
					Enn=(h[k]+V[k]**2/(2*g))
				if (x2+0.1)>x[k] and x[k]>(x2-0.1) and (y2+0.1)>y[k] and y[k]>(y2-0.1) and (z2+0.1)>h[k] and h[k]>(z2-0.1):
					check=1
					break
# 			print "x",x[k],x2,"y",y[k],y2,"h",h[k],z2
			if check==1:
				break
# 		self.Vc=V[k]
# 		x=dist[k]*cos(beta)+x1
# 		y=dist[k]*sin(beta)+y1
		if check!=1:
			Val=[xn,yn,zn,gamman,phin,vn,tn,Enn]
# 			print 'b',b,'initial phi=',initial[4],'final phi=', phin,'a=',a,'initial gamma=',initial[3],'final gamma=',gamman,'d2n=',dm
		else:
			En=(h[k]+V[k]**2/(2*g))
			Val=[x[k],y[k],h[k],gamma[k],phi[k],V[k],time[k],En]
		return Val
		
	def metric(self,n1,n2):
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
		x1=float(x1)
		y1=float(y1)
		x2=float(x2)
		y2=float(y2)
		z1=float(z1)
		z2=float(z2)
		px=(x2-x1)**2
		py=(y2-y1)**2
		pz=(z2-z1)**2
		d=(px+py+pz)**0.5
		#distance to goal
		pgx=(xg-x1)**2
		pgy=(yg-y1)**2
		pgz=(zg-z1)**2
		dg=(pgx+pgy+pgz)**0.5		
# 		gamma=rad2deg(arcsin((z2-z1)/d))
		Wz=self.yv[n1]
		if (self.fail[n1]+self.fail[n2]!=0):f=1-((self.con[n1]+self.con[n2])/(self.fail[n1]+self.fail[n2]))
		else: f=1
		metric = d*self.Gd/524.05-Wz*self.Gv/5.557+self.Gc*float(self.children[n1])/G.number_of_nodes() +dg/524.05*self.Gd*0.25 +f*self.Gf-(z1-z2)/100*self.Gz
# 		print "connection=",self.Gc*float(self.children[n1])/G.number_of_nodes(), "distance =",d*self.Gd/524.05, "updrafts =",-Wz*self.Gv/5.557, "Height =",z1/100*self.Gz,"Failure =",f*self.Gf
# 		print x1,y1,z1,metric
		return metric
	
	#cumulative metric along path	
	def cumetric(self,n1,n2):
		cost = 0
		for i in xrange(n1,n2):
			cost = cost + self.metric(self.path[i+1],self.path[i])
		return cost		
		
	def energy(self,n1,n2):
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
		Wz=self.yv[n1]
		gamma=self.gamma[n1]
		phi=self.phi[n1]
		# dx=x2-x1
# 		dy=y2-y1
		# if dx>0:
# 			b=rad2deg(atan2(dy,dx))
# 		elif dx<0:
# 			if dy>0:
# 				b=90+rad2deg(atan2(dx,dy))
# 			elif dy<0:
# 				b=-90-rad2deg(atan2(dx,dy))
# 		elif dx==0:
# 			if dy>0:
# 				b=90
# 			elif dy<0:
# 				b=-90	
# 		if phi>90 and b<-90:
# 			bDiff=(180-phi)-(-180-b)
# 		elif phi<-90 and b>90:
# 			bDiff=-(180-b)-(-180-phi)
# 		else: 
# 			bDiff=b-phi
		
	# 	Vx = self.Vi[n1]*cos(deg2rad(gamma))
# 		alpha=rad2deg(atan2(Wz,Vx))
# 		a=rad2deg(atan2((z2-z1),sqrt((y2-y1)**2+(x2-x1)**2)))
		
		# if -25>a or a>25 or -25>bDiff or bDiff>25 alpha>20 or alpha<-12:
		# 	return [0,0,0,0,0,0,0,0]
# 		else:	
		return self.dof([x1,y1,z1,gamma,phi,self.Vi[n1],Wz],[x2,y2,z2])
		
	def dist(self,n1,n2):
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
		x1=float(x1)
		y1=float(y1)
		x2=float(x2)
		y2=float(y2)
		z1=float(z1)
		z2=float(z2)
		px=(x2-x1)**(2)
		py=(y2-y1)**(2)
		pz=(z2-z1)**(2)
		metric = (px+py+pz)**(0.5)
		return metric
			
	#expand a random point
	#calls subroutines to find nearest node and connect it
	def expand (self):
		#add random node
		x = random.uniform (E.xmin, E.xmax)
		y = random.uniform (E.ymin, E.ymax)
		z = random.uniform (E.zmin, E.zmax)
		n= self.number_of_nodes() #new node 
		self.add_node(n,x,y,z)
		if E.isfree(Polygons)!=0:
			#find nearest node
			nnear = self.near(n)
			#find new node based on step size
			self.step(nnear,n)
		else:
			print 'reject obstacle 1'

		
	def bias (self):
		#add random node
		n= self.number_of_nodes() #new node
		self.add_gnode(n) #test goal region
		
		#find nearest node
		nnear = self.near(n)
		#find new node based on step size
		self.step(nnear,n)

	
	#nearest node
	def near(self,n):
		#find a near node
		dmin = self.metric(0,n)
		nnear = 0
		for i in xrange(0,n):
			d = self.metric(i,n)
# 			print "i =",i,"metric =",d
			if d < dmin:			
				dmin=d
				nnear = i
		return nnear
		
#step size
	def step(self,nnear,nrand):
		d = self.dist(nnear,nrand)
		(xrand,yrand,zrand)= (self.x[nrand],self.y[nrand],self.z[nrand]) 
		(xnear,ynear,znear)= (self.x[nnear],self.y[nnear],self.z[nnear]) 
		# (px,py,pz)=(xrand-xnear,yrand-ynear,zrand-znear)
# 		theta = math.atan2(py,px)
# 		alpha = math.asin(pz/d)
		
# 		Distance check
		# if d>dmax:
# 			self.remove_node(nrand)
# 			z=znear+dmax*math.sin(alpha+self.gamma[nnear])
# 			dxy=dmax*math.cos(alpha+self.gamma[nnear])
# 			x=xnear+dxy*math.cos(theta+self.phi[nnear])
# 			y=ynear+dxy*math.sin(theta+self.phi[nnear])
# 			self.add_node(nrand,x,y,z) #this is a new node between rand and near
# 			result=self.energy(nnear,nrand)
# 			self.add_result(nrand,result[3],result[4],result[5],result[6])
# 			self.remove_node(nrand)
# 			self.add_node(nrand,result[0],result[1],result[2])
# 		else:
		
		Wz=self.yv[nnear]
		gamma=self.gamma[nnear]
		Vx = self.Vi[nnear]*cos(deg2rad(gamma))
 		alpha=rad2deg(atan2(Wz,Vx))
		
		result=self.energy(nnear,nrand)
		self.remove_node(nrand)
		self.add_node(nrand,result[0],result[1],result[2])
		self.add_result(nrand,result[3],result[4],result[5],result[6])
		self.connect(nnear,nrand)

# 		New node find
		# if self.stall==1:
# 			self.remove_node(nrand)
# 			i=random.uniform(-20,20)
# 			j=random.uniform(-20,20)
# 			k=random.uniform(-20,20)
# # 			Need z global value
# 			z=znear+dmax*math.sin(deg2rad(i))
# # 			Distance in 2d plane
# 			dxy=dmax*math.cos(deg2rad(i))
# 			x=xnear+dxy*math.cos(deg2rad(j)+deg2rad(self.phi[nnear]))
# 			y=ynear+dxy*math.sin(deg2rad(k)+deg2rad(self.phi[nnear]))
# 			self.add_node(nrand,x,y,z) #this is a new node between rand and near
# 			self.remove_result(nrand)
# 			result=self.energy(nnear,nrand)
# 			self.add_result(nrand,result[3],result[4],result[5],result[6])
# 			self.remove_node(nrand)
# 			self.add_node(nrand,result[0],result[1],result[2])
		

			
				

#connect two nodes (local planner)
	def connect(self,n1,n2):
		(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
		(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
		n= G.number_of_nodes()-1
		#subdivide path into 100 small segments and ensure each segment is collision free
		check=0
		for i in xrange(0,n-1):
			if self.x[i]==x2 and self.y[i]==y2 and self.z[i]==z2:
				print "reject same node"
				self.remove_node(n2)
				self.remove_result(n2)
				self.fail[n1]=self.fail[n1]+1
				check=1
				break
		if check==0:
			if E.inobstacle(x1,y1,z1,x2,y2,z2,Polygons)==0:
				self.remove_node(n2)
				self.remove_result(n2)
				self.fail[n1]=self.fail[n1]+1
				print 'reject obstacle 2'
			else:
				self.add_edge(n1,n2)
				print "connect",x2,y2,z2

# add results
	def add_result(self,n,gamma,phi,V,t):
		self.Vi.insert(n, V)
		self.gamma.insert(n, gamma)
		self.phi.insert(n, phi)
		self.time.insert(n,t)
		
# remove results		
	def remove_result(self,n):
		self.Vi.pop(n)
		self.gamma.pop(n)
		self.phi.pop(n)
		self.time.pop(n)
			
#add node
	def add_node(self,n,x,y,z):
		self.x.insert(n, x)
		self.y.insert(n, y)
		self.z.insert(n, z)
		#self.yv.insert(n,self.file.cfdmetric(self.x[n],self.y[n],self.z[n]))
		self.yv.insert(n,E.updraft(x,y,z))
		self.fail.insert(n,0)
		self.con.insert(n,0)
		self.children.insert(n,0)
		
#add node
	def add_gnode(self,n):
		self.x.insert(n, xg)
		self.y.insert(n, yg)
		self.z.insert(n, zg)
		self.yv.insert(n,0)	
		self.fail.insert(n,0)
		self.con.insert(n,0)
		self.children.insert(n,0)	

#remove node
	def remove_node(self,n):
		self.x.pop(n)
		self.y.pop(n)
		self.z.pop(n)
		self.yv.pop(n)
		self.fail.pop(n)
		self.con.pop(n)
		self.children.pop(n)

#add edge
	def add_edge(self,parent,child):
		self.parent.insert(child,parent)
		self.con[parent]=self.con[parent]+1.0
		self.con[child]=self.con[child]+1.0
		self.children[parent]=self.children[parent]+1.0
		
#remove node		
	def remove_edge(self,n):
		self.parent.pop(n)
		self.children[parent]=self.children[parent]-1
		
#clear
	def clear(self,nstart):
		(x,y)=nstart
		self.x=[]
		self.y=[]
		self.z=[]
		self.yv=[]
		self.parent=[]
		self.x.append(x)
		self.y.append(y)
		self.z.append(z)
		self.yv.append(self.file.cfdmetric(x,y,z))
		self.parent.append(0)
		
#number of nodes
	def number_of_nodes(self):
		return len(self.x)
		
#Possible goal path
	def goalcheck(self):
		n= G.number_of_nodes()-1
		(x,y,z)= (self.x[n], self.y[n],self.z[n]) 
		(xg,yg,zg)= ((xgmax-10),(ygmax-10),(zgmax-10))
		gammaMin=atan2(1,13.24184338)
		a=-(rad2deg(atan2((zg-z),sqrt((yg-y)**2+(xg-x)**2))))
		d=sqrt((xg-x)**2+(yg-y)**2+(zg-z)**2)
		t=d/self.Vi[n]
		dx=xg-x
		dy=yg-y
		if dx>0:
			b=rad2deg(atan2(dy,dx))
		elif dx<0:
			if dy>0:
				b=90-rad2deg(atan2(dx,dy))
			elif dy<0:
				b=-90+rad2deg(atan2(dx,dy))
			elif dy==0:
				b=-180
		elif dx==0:
			if dy>0:
				b=90
			elif dy<0:
				b=-90
			elif dy==0:
				b=0
		
		if a>10:
			if E.inobstacle(x,y,z,xg,yg,zg,Polygons)==1:
				self.add_node(self.number_of_nodes(),xg,yg,zg)
				en=(z+self.Vi[n]**2/(2*9.81))
				self.add_result(self.number_of_nodes(),-a,b,sqrt(en*(2*9.81)-zg),t)
				self.connect(n,n+1)
				return 1
			else:
				return 0
		else:
			return 0
		
#path to goal
	def path_to_goal(self):
		self.goalstate = G.number_of_nodes()
		for i in xrange (G.number_of_nodes()):
			(x,y,z)= (self.x[i],self.y[i],self.z[i])
			if (x>=xgmin) and (x<=xgmax) and (y>=ygmin) and (y<=ygmax) and (z>=zgmin) and (z<=zgmax) :
				self.goalstate = i
				break
		#add goal state to and its parent node to the path	
		self.path=[]
		self.path.append(i)
		newpos=self.parent[i]
		#keep adding parents	
		while (newpos!=0):
			self.path.append(newpos)
			newpos=self.parent[newpos]	
		#add start state
		self.path.append(0)
				
	def prun(self):	
		#initial query nodes in the path
		#we already know 0-1 is collision free
		#start by checking 0-2
		s=0
		e=2
		self.tpath=[]
		self.tpath.append(self.path[s])
		for e in xrange(2,len(self.path)-1):
			(x1,y1,z1)=(self.x[self.path[s]],self.y[self.path[s]],self.z[self.path[s]])
			(x2,y2,z2)=(self.x[self.path[e]],self.y[self.path[e]],self.z[self.path[e]])
			if E.inobstacle(x1,y1,z1,x2,y2,z2,Polygons)==0 or self.metric(self.path[s],self.path[e]) < self.cumetric(s,e): #CC is detected
				c=0
				self.tpath.append(self.path[e-1])
				s=e-1

		self.tpath.append(self.path[-1])			

	#draw tree
	def showtree(self,k):
		for i in xrange (0,self.number_of_nodes()):
			par=self.parent[i]
			x=[self.x[i],self.x[par]]
			y=[self.y[i],self.y[par]]
			z=[self.z[i],self.z[par]]
			ax.plot(x,y,z,k,lw=0.5)
			
			
	#draw path 
	def showpath(self,k):
		for i in xrange (len(self.path)-1):
			 n1=self.path[i]
			 n2=self.path[i+1]
			 x=[self.x[n1],self.x[n2]]
			 y=[self.y[n1],self.y[n2]]
			 z=[self.z[n1],self.z[n2]]
			 ax.plot(x,y,z,k,lw=1,markersize=3)
			 
 	def plotStats(self,k1,k2,k3,k4):
		d=[0]
 		h=[self.z[0]]
 		E=[self.z[0]+self.Vi[0]**2/(2*9.81)]
 		V=[self.Vi[0]]
 		Wv=[self.yv[0]]
 		x=[self.x[0]]
 		y=[self.y[0]]
 		gamma=[self.gamma[0]]
 		phi=[self.phi[0]]
 		time=[0]
 		self.path.reverse()
 		for i in xrange(0,len(self.path)-1):
 			n1=self.path[i]
 			n2=self.path[i+1]
			(x1,y1,z1)= (self.x[n1],self.y[n1],self.z[n1])
			(x2,y2,z2)= (self.x[n2],self.y[n2],self.z[n2])
			h.append(z2)
			d.append(d[-1]+self.dist(n1,n2))
			result=self.energy(n1,n2)
			E.append(self.z[n2]+self.Vi[n2]**2/(2*9.81))
			V.append(self.Vi[n2])
			Wv.append(self.yv[n2])
			x.append(x2)
			y.append(y2)
			time.append(time[-1]+self.time[n2])
			gamma.append(self.gamma[n2])
			phi.append(self.phi[n2])
		
		plt.figure(2)
		plt.subplot(611)
		plt.plot(d,E,k1,markersize=10)
		# plt.subplot(611)
# 		plt.plot(d,E2,k2,markersize=10)
		plt.xlabel('Travelled Distance [m]')
		plt.ylabel('Energy[J]')
		plt.grid(True)
		plt.subplot(612)
		plt.plot(d,h,k2,markersize=10)
		plt.xlabel('Travelled Distance[m]')
		plt.ylabel('Height[m]')
		plt.grid(True)
		plt.subplot(613)
		plt.plot(d,V,k3,markersize=10)
		plt.xlabel('Travelled Distance [m]')
		plt.ylabel('Aircraft Velocity [ms-1]')
		plt.grid(True) 		
		plt.subplot(614)
		plt.plot(d,Wv,k4,markersize=10)
		plt.xlabel('Travelled Distance [m]')
		plt.ylabel('Updraft Velocity [ms-1]')
		plt.grid(True) 
		plt.subplot(615)
		plt.plot(d,gamma,k4,markersize=10)
		plt.xlabel('Travelled Distance [m]')
		plt.ylabel('gamma')
		plt.grid(True) 
		plt.subplot(616)
		plt.plot(d,phi,k4,markersize=10)
		plt.xlabel('Travelled Distance [m]')
		plt.ylabel('phi')
		plt.grid(True) 		
		
		plt.figure(3)
		plt.subplot(611)
		plt.plot(time,E,k1,markersize=10)
		# plt.subplot(611)
# 		plt.plot(time,E2,k2,markersize=10)
		plt.xlabel('Time [sec]')
		plt.ylabel('Energy[J]')
		plt.grid(True)
		plt.subplot(612)
		plt.plot(time,h,k2,markersize=10)
		plt.xlabel('Time [sec]')
		plt.ylabel('Height[m]')
		plt.grid(True)
		plt.subplot(613)
		plt.plot(time,V,k3,markersize=10)
		plt.xlabel('Time [sec]')
		plt.ylabel('Aircraft Velocity [ms-1]')
		plt.grid(True) 		
		plt.subplot(614)
		plt.plot(time,Wv,k4,markersize=10)
		plt.xlabel('Time [sec]')
		plt.ylabel('Updraft Velocity [ms-1]')
		plt.grid(True) 
		plt.subplot(615)
		plt.plot(time,gamma,k4,markersize=10)
		plt.xlabel('Time [sec]')
		plt.ylabel('gamma')
		plt.grid(True) 
		plt.subplot(616)
		plt.plot(time,phi,k4,markersize=10)
		plt.xlabel('Time [sec]')
		plt.ylabel('phi')
		plt.grid(True) 	
	
		filename='Ver10 D=%s V=%s C=%s.csv' %(self.Gd,self.Gv,self.Gc)
		f = open(filename, "w")
		f.write("{},{},{},{},{},{},{},{}\n".format('time','x','y','z','d','V','Wv','E'))
		for i in xrange(len(d)-1):
			f.write("{},{},{},{},{},{},{},{}\n".format(time[i],x[i],y[i],h[i],d[i],V[i],Wv[i],E[i]))
		f.close()
	
	#draw path to be executed
	def showtpath(self,k):
		for i in xrange (len(self.tpath)-1):
			 n1=self.tpath[i]
			 n2=self.tpath[i+1]
			 x=[self.x[n1],self.x[n2]]
			 y=[self.y[n1],self.y[n2]]
			 z=[self.z[n1],self.z[n2]]
			 ax.plot(x,y,z,k,lw=2,markersize=5)
			 
	#print gps coordinates
	def printgps(self):
		print 'GPS Coordinates:'
		#Initial coordinates
		lati=-37.68057
		loni=145.061333
		offset=8
		print self.path
		self.path.reverse()
		print self.path
		d=sqrt(square(self.x[self.path[0]])+square(self.y[self.path[0]]))
		bearing=degrees(atan2(self.x[self.path[0]],self.y[self.path[0]]))+offset
		#Earth radius
		R=6378100
		lat1 = degrees(asin(sin(radians(lati))*cos(d/R)+cos(radians(lati))*sin(d/R)*cos(radians(bearing))))
		lon1 = loni + degrees(atan2(sin(radians(bearing))*sin(d/R)*cos(radians(lati)),cos(d/R)-sin(radians(lati))*sin(radians(lat1))))
		alt=self.z[self.path[0]]
		for i in xrange (len(self.path)-1):
			n1=self.path[i]
			n2=self.path[i+1]
			x=[self.x[n1],self.x[n2]]
			y=[self.y[n1],self.y[n2]]
			z=[self.z[n1],self.z[n2]]
			print lat1,",",lon1,",",alt
			d=sqrt(square(self.x[n2]-self.x[n1])+square(self.y[n2]-self.y[n1]))
			bearing=degrees(atan2((self.x[n2]-self.x[n1]),(self.y[n2]-self.y[n1])))+offset
			lat2=degrees(asin(sin(radians(lat1))*cos(d/R)+cos(radians(lat1))*sin(d/R)*cos(radians(bearing))))
			lon2 = lon1 + degrees(atan2(sin(radians(bearing))*sin(d/R)*cos(radians(lat1)),cos(d/R)-sin(radians(lat1))*sin(radians(lat2))))
			lon1=lon2
			lat1=lat2
			alt=self.z[n2]
		print lat2,",",lon2,",",alt
#--------------------------------------Global Definitions---------------------------------
#node limit
nmax = 20000
#goal region
xg=300
yg=-120
zg=10
epsilon=10
xgmin=xg-epsilon
xgmax=xg+epsilon
ygmin=yg-epsilon
ygmax=yg+epsilon
zgmin=zg-epsilon
zgmax=zg+epsilon

#extend step size
# dmax =1
#start the root of the tree
nstart =(-130,170,20) 

#create an RRT tree with a start node
G=RRT3d(nstart)

#environment instance
E=env3d(-130,300,-120,170,0,100)

#obstacles
Polygons=E.read()
	
#draw setup
fig = plt.figure(1)
ax = fig.gca(projection='3d')



#--------------------------------------Functions------------------------------------------
#draw trees and environment
def draw ():

	#draw 
	
	#goal region
	gx=[xgmin,xgmin,xgmax,xgmax,xgmin]
	gy=[ygmin,ygmax,ygmax,ygmin,ygmin]
	gzl=[zg-epsilon]
	gzh=[zg+epsilon]
	E.cubedraw(gx,gy,gzl,gzh,'g')

	#draw tree
	G.showtree('0.45')
		 
	#draw path
	G.showpath('k')
	G.showtpath('r')
	
	#draw obstacles
	for Polygon in Polygons:
		E.cubedraw(Polygon[0],Polygon[1],Polygon[2],Polygon[3],'k')
	
	#aspect ratio
	apx=[0,0]
	apy=[0,0]
	apz=[300,300]	
	ax.plot(apx,apy,apz,lw=1,markersize=0.1)
	
	#stats
	G.plotStats('b','r','g','c')
	
	plt.show()

#--------------------------------------RRT Implementation---------------------------------
def main():
	#balance between extending and biasing	
	startTime = time.time()
	for i in xrange(0,nmax):
		print i
		if i%20!=0: G.expand()
		else: G.bias()
		# G.expand()
	#check if sample is in goal, if so STOP!		
# 		if E.ingoal()==1:
		if G.goalcheck()==1 or E.ingoal():
                        deltatime = time.time()-startTime
			print 'found in',time.strftime("%H:%M:%S", time.gmtime(deltatime))
			break
	#add reach goal function?!!
	G.path_to_goal()
	G.prun()
		
	#display initial plan under limited sensing
	draw()

# run main when RRT is called
if __name__ == '__main__':
    main()