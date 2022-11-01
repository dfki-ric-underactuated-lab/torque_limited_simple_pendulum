import numpy as np

from simple_pendulum.controllers.lqr.lqr import lqr

def transform(q):
    return np.asarray([(q[0]%(2*np.pi)-np.pi),q[1]])

class analytic_roa():
    def __init__(self, mass=1.0, length=0.5, damping=0.1,
                 gravity=9.81, torque_limit=np.inf, Q_11 = 1, Q_22 = 1, R = 1):
        self.m = mass
        self.len = length
        self.b = damping
        self.g = gravity
        self.torque_limit = torque_limit

        self.A = np.array([[0, 1],
                           [self.g/self.len, -self.b/(self.m*self.len**2.0)]])
        self.B = np.array([[0, 1./(self.m*self.len**2.0)]]).T
        self.Q = np.diag((Q_11, Q_22))
        self.R = np.array([[R]])

        self.K, self.S, _ = lqr(self.A, self.B, self.Q, self.R)
        
        self.D = (((self.K[0,1]+self.b)/(self.m*self.len*self.len))**2 - 4 *
                  (self.K[0,0]/(self.m*self.len*self.len)-(self.g/self.len)))
        self.Droot = np.sqrt(self.D)
        bdash = (self.K[0,1]+self.b)/(self.m*self.len*self.len)
        self.kappa = np.array([(-bdash+self.Droot)/2,(-bdash-self.Droot)/2])
        self.Cmat = (np.array([[-self.kappa[1],1],[self.kappa[0],-1]])
                     /(self.kappa[0]-self.kappa[1]))
    
    
    def tstar(self, q):
        c = np.dot(self.Cmat,q)
        a = -(self.K[0,0]+self.K[0,1]*self.kappa)*self.kappa*c
        if not a[1] == 0:
            arg = -(a[0]/a[1])
            if arg > 0:
                return -np.log(arg) / self.Droot
            else:
                return -1
        else:
            return -1
    
    
    def u(self, t, q):
        c = np.dot(self.Cmat,q)
        coeff0 = -(self.K[0,0]+self.K[0,1]*self.kappa[0])*c[0]
        coeff1 = -(self.K[0,0]+self.K[0,1]*self.kappa[1])*c[1]
        out = coeff0 * np.exp(self.kappa[0]*t) + coeff1 * np.exp(self.kappa[1]*t)        
        return out
    
    def satisfies_theory(self, y):
        q = transform(y)
        if np.abs(q[0]-np.sin(q[0])) < self.torque_limit/(self.m*self.g*self.len):
            if self.D>0 and (self.kappa<0).all():
                uz = np.abs(-self.K.dot(q))
                ts = self.tstar(q)
                us = np.abs(self.u(ts, q))
                return all([uz <= self.torque_limit, (ts < 0 or us <=self.torque_limit)])
            else:
                return False
        else:
            return False
        
    def satisfies_theory_full(self, y):
        q = transform(y)
        if self.D>0 and (self.kappa<0).all():
            uz = np.abs(-self.K.dot(q))
            ts = self.tstar(q)
            us = np.abs(self.u(ts, q))
            return all([uz <= self.torque_limit, (ts < 0 or us <=self.torque_limit)])
        else:
            return False