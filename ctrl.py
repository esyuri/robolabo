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
		self.motorP=[50,50,50,50]
		
	def motor(self,rx,ry,rz):
		
		self.euler=[rx,ry,rz]
		
		euler_pidx.update(rx)
		d_rx=euler_pidx.output
		euler_pidy.update(ry)
		d_ry=euler_pidy.output
		euler_pidy.update(rz)
		d_rz=euler_pidz.output
		
		self.deuler=[d_rx,d_ry,d_rz]
		"""
		motor[n]は第ｎ+1象限とする
		motor0,motor2はroll_zと回転の方向が同じであるとする
		x回転の調整　[+,+,-,-]
		y回転の調整　[-,+,+,-]
		z回転の調整　[+,-,+,-]
		"""
		self.motorP[0]=self.motorP[0]-self.deuler[0]*self.dP[0]+self.deuler[1]*self.dP[1]-self.deuler[2]*self.dP[2]
		self.motorP[1]=self.motorP[1]-self.deuler[0]*self.dP[0]-self.deuler[1]*self.dP[1]+self.deuler[2]*self.dP[2]
		self.motorP[2]=self.motorP[2]+self.deuler[0]*self.dP[0]-self.deuler[1]*self.dP[1]-self.deuler[2]*self.dP[2]
		self.motorP[3]=self.motorP[3]+self.deuler[0]*self.dP[0]+self.deuler[1]*self.dP[1]+self.deuler[2]*self.dP[2]
		#print(self.motorP)
		
		
		
		

        
	
