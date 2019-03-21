import PID

euler_pidx = PID.PID()
euler_pidx.clear
euler_pidy = PID.PID()
euler_pidy.clear
euler_pidz = PID.PID()
euler_pidz.clear

class ctrl:
	
	
	def __init__(self,rx0=0.0,ry0=0.0,rz0=0.0):
		self.rollx = rx0
		self.rolly = ry0
		self.rollz = rz0
		
		self.clear()
	
	def clear(self):
		self.dP=[1,1,1]
		self.motorP=[60,60,60]
		
	def motor(self,rx,ry,rz):
		
		self.euler=[rx,ry,rz]
		
		euler_pidx.update(rx)
		d_rx=euler_pidx.output
		euler_pidy.update(ry)
		d_ry=euler_pidy.output
		euler_pidy.update(rz)
		d_rz=euler_pidz.output
		
		self.deuler=[d_rx,d_ry,d_rz]
		
		kx=0.5
		ky=0.5
		#kz=0.1
		
		"""
		motor[0] (1,0)
		motor[1] (-1/2,root3/2)
		motor[2] (-1/2,-root3/2)
		x回転の調整　[0,1,-1]
		y回転の調整　[-1/2,1,1]
		z回転の調整　[0,0,0] (当てにしない）
		"""
		"""
		self.motorP[0]=self.motorP[0]-kx*self.deuler[0]*self.dP[0]+ky*self.deuler[1]*self.dP[1]-kz*self.deuler[2]*self.dP[2]
		self.motorP[1]=self.motorP[1]-kx*self.deuler[0]*self.dP[0]-ky*self.deuler[1]*self.dP[1]+kz*self.deuler[2]*self.dP[2]
		self.motorP[2]=self.motorP[2]+kx*self.deuler[0]*self.dP[0]-ky*self.deuler[1]*self.dP[1]-kz*self.deuler[2]*self.dP[2]
		"""
		
		self.motorP[0]=self.motorP[0]-kx*0*self.deuler[0]*self.dP[0]+ky*1/2*self.deuler[1]*self.dP[1]
		self.motorP[1]=self.motorP[1]-kx*1*self.deuler[0]*self.dP[0]-ky*1*self.deuler[1]*self.dP[1]
		self.motorP[2]=self.motorP[2]+kx*1*self.deuler[0]*self.dP[0]-ky*1*self.deuler[1]*self.dP[1]
		
		for i in range(3):
			if self.motorP[i] > 66:
				self.motorP[i]= 66
			elif self.motorP[i]<54:
				self.motorP[i]=54
		

