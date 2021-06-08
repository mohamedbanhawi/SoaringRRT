import csv
import scipy.interpolate as intp


class CReader:

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
