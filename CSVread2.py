import csv
import numpy
import scipy.interpolate as intp
import pprint

class CSVReader:

	def __init__(self,file):
		print "Reading data..."
		csvFile = open(file)
		self.data= numpy.loadtxt(csvFile, delimiter=',')
		self.coordinate=numpy.delete(self.data, numpy.s_[3:], 1)

	def cfdmetric(self,x,y,z):
		print 'Processing a query..'
		print x,y,z
# 		for j in range(0,len(self.data)):
# 			if self.data[j][0]>=(x-epsilon) and self.data[j][0]<=(x+epsilon) and self.data[j][1]>=(y-epsilon) and self.data[j][1]<=(y+epsilon) and self.data[j][2]>=(z-epsilon) and self.data[j][2]<=(z+epsilon):
# 				region.append(self.data[j])	
# 		xdata=[]
# 		ydata=[]
# 		zdata=[]
# 		vdata=[]
		point=numpy.array([x,y,z])
		d = numpy.sqrt(((self.coordinate-point)**2).sum(axis=1))
		ndx = d.argsort()
		xdata=self.data[ndx[:10],[1]]
		ydata=self.data[ndx[:10],[0]]*-1
		zdata=self.data[ndx[:10],[2]]
		vdata=self.data[ndx[:10],[3]]
		
# 		if len(region)<=4:
# 			vq=0
# 		else:
# 			for k in range(0,len(region)):
# 				xdata.append(region[k][0])
# 				ydata.append(region[k][1])
# 				zdata.append(region[k][2])
# 				vdata.append(region[k][3])
# 		
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


