import csv
import numpy
import scipy.interpolate as intp
import ckdtree

class CSVReader:

	def __init__(self,file):
		print "Reading data..."
		csvFile = open(file)
		self.data= numpy.loadtxt(csvFile, delimiter=',')
		self.coordinate=numpy.delete(self.data, numpy.s_[3:], 1)

	def cfdmetric(self,x,y,z):
		point=numpy.array([x,y,z])
		tree = ckdtree.cKDTree(self.coordinate, leafsize=self.coordinate.shape[0]+1)
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


