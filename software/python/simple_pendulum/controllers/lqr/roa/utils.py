from termios import TIOCSWINSZ
from scipy.spatial.transform import Rotation as R
from scipy import linalg
from scipy.special import gamma, factorial
import numpy as np
from pydrake.all import (MathematicalProgram, Solve, Variables, Jacobian)
from pydrake.symbolic import TaylorExpand, Evaluate
from pydrake.all import Variable



def direct_sphere(d,r_i=0,r_o=1):
    """Direct Sampling from the d Ball based on Krauth, Werner. Statistical Mechanics: Algorithms and Computations. Oxford Master Series in Physics 13. Oxford: Oxford University Press, 2006. page 42

    Parameters
    ----------
    d : int
        dimension of the ball
    r_i : int, optional
        inner radius, by default 0
    r_o : int, optional
        outer radius, by default 1

    Returns
    -------
    np.array
        random vector directly sampled from the solid d Ball
    """
    # vector of univariate gaussians:
    rand=np.random.normal(size=d)
    # get its euclidean distance:
    dist=np.linalg.norm(rand,ord=2)
    # divide by norm
    normed=rand/dist
    
    # sample the radius uniformly from 0 to 1 
    rad=np.random.uniform(r_i,r_o**d)**(1/d)
    # the r**d part was not there in the original implementation.
    # I added it in order to be able to change the radius of the sphere
    # multiply with vect and return
    return normed*rad

def sample_from_ellipsoid(M,rho,r_i=0,r_o=1):
    """sample directly from the ellipsoid defined by xT M x.

    Parameters
    ----------
    M : np.array
        Matrix M such that xT M x leq rho defines the hyperellipsoid to sample from
    rho : float
        rho such that xT M x leq rho defines the hyperellipsoid to sample from
    r_i : int, optional
        inner radius, by default 0
    r_o : int, optional
        outer radius, by default 1

    Returns
    -------
    np.array
        random vector from within the hyperellipsoid
    """
    lamb,eigV=np.linalg.eigh(M/rho) 
    d=len(M)
    xy=direct_sphere(d,r_i=r_i,r_o=r_o) #sample from outer shells
    T=np.linalg.inv(np.dot(np.diag(np.sqrt(lamb)),eigV.T)) #transform sphere to ellipsoid (refer to e.g. boyd lectures on linear algebra)
    return np.dot(T,xy.T).T

def quad_form(M,x):
    """
    Helper function to compute quadratic forms such as x^TMx
    """
    return np.dot(x,np.dot(M,x))


def vol_ellipsoid(rho,M):
    """
    Calculate the Volume of a Hyperellipsoid
    Volume of the Hyperllipsoid according to https://math.stackexchange.com/questions/332391/volume-of-hyperellipsoid/332434
    Intuition: https://textbooks.math.gatech.edu/ila/determinants-volumes.html
    Volume of n-Ball https://en.wikipedia.org/wiki/Volume_of_an_n-ball
    """
    
    # For a given hyperellipsoid, find the transformation that when applied to the n Ball yields the hyperellipsoid
    lamb,eigV=np.linalg.eigh(M/rho) 
    A=np.dot(np.diag(np.sqrt(lamb)),eigV.T) #transform ellipsoid to sphere
    detA=np.linalg.det(A)
    
    # Volume of n Ball (d dimensions)
    d=M.shape[0] # dimension 
    volC=(np.pi**(d/2))/(gamma((d/2)+1))

    # Volume of Ellipse
    volE=volC/detA

    return volE

def rhoVerification(rho, pendulum, controller):
    """SOS Verification of the Lyapunov conditions for a given rho in order to obtain an estimate  of the RoA for the closed loop dynamics.
       This method is described by Russ Tedrake in "Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation", 
       Course Notes for MIT 6.832, 2022, "http://underactuated.mit.edu", sec. 9.3.2: "Basic region of attraction formulation".

    Parameters
    ----------
    rho: float
        value of rho to be verified
    pendulum : simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller : simple_pendulum.controllers.lqr.lqr_controller
        configured lqr controller object

    Returns
    -------
    result : boolean
        result of the verification
    """

    #K and S matrices from LQR control
    K = controller.K
    S = controller.S

    # Pendulum parameters
    m = pendulum.m
    l = pendulum.l
    g = pendulum.g
    b = pendulum.b
    torque_limit = pendulum.torque_limit

    # non-linear dyamics
    prog = MathematicalProgram()
    xbar = prog.NewIndeterminates(2, "x")
    xg = [np.pi, 0]  # reference
    x = xbar + xg
    ubar = -K.dot(xbar)[0] # control input with reference
    Tsin = -(x[0]-xg[0]) + (x[0]-xg[0])**3/6 - (x[0]-xg[0])**5/120 + (x[0]-xg[0])**7/5040
    fn = [x[1], (ubar-b*x[1]-Tsin*m*g*l)/(m*l*l)]

    # cost-to-go of LQR as Lyapunov candidate
    V = (xbar).dot(S.dot(xbar))
    Vdot = Jacobian([V], xbar).dot(fn)[0]

    # Saturation for fn and Vdot
    u_minus = - torque_limit
    u_plus = torque_limit
    fn_minus = [x[1], (u_minus-b*x[1]-Tsin*m*g*l)/(m*l*l)] 
    Vdot_minus = V.Jacobian(xbar).dot(fn_minus)
    fn_plus = [x[1], (u_plus-b*x[1]-Tsin*m*g*l)/(m*l*l)] 
    Vdot_plus = V.Jacobian(xbar).dot(fn_plus)

    # Define the Lagrange multipliers.
    lambda_1 = prog.NewSosPolynomial(Variables(xbar), 6)[0].ToExpression()
    lambda_2 = prog.NewSosPolynomial(Variables(xbar), 6)[0].ToExpression()
    lambda_3 = prog.NewSosPolynomial(Variables(xbar), 6)[0].ToExpression()
    lambda_4 = prog.NewSosPolynomial(Variables(xbar), 6)[0].ToExpression()
    lambda_5 = prog.NewSosPolynomial(Variables(xbar), 6)[0].ToExpression()
    lambda_6 = prog.NewSosPolynomial(Variables(xbar), 6)[0].ToExpression()
    lambda_7 = prog.NewSosPolynomial(Variables(xbar), 6)[0].ToExpression()
    epsilon=10e-10

    # Optimization constraints
    prog.AddSosConstraint(-Vdot_minus + lambda_1*(V-rho) + lambda_2*(-u_minus+ubar) - epsilon*xbar.dot(xbar))
    prog.AddSosConstraint(-Vdot + lambda_3*(V-rho) + lambda_4*(u_minus-ubar) + lambda_5*(-u_plus+ubar) - epsilon*xbar.dot(xbar))
    prog.AddSosConstraint(-Vdot_plus + lambda_6*(V-rho) + lambda_7*(u_plus-ubar) - epsilon*xbar.dot(xbar))

    # Solve the problem
    result = Solve(prog).is_success()
    return result

class PendulumPlantApprox:
    def __init__(self, mass=1.0, length=0.5, damping=0.1, gravity=9.81,
                 coulomb_fric=0.0, inertia=None, torque_limit=np.inf, taylorApprox_order = 1):

        """
        The PendulumPlantApprox class contains the taylor-approximated dynamics
        of the simple pendulum.

        The state of the pendulum in this class is described by
            state = [angle, angular velocity]
            (array like with len(state)=2)
            in units: rad and rad/s
        The zero state of the angle corresponds to the pendulum hanging down.
        The plant expects an actuation input (tau) either as float or
        array like in units Nm.
        (in which case the first entry is used (which should be a float))

        Parameters
        ----------
        mass : float, default=1.0
            pendulum mass, unit: kg
        length : float, default=0.5
            pendulum length, unit: m
        damping : float, default=0.1
            damping factor (proportional to velocity), unit: kg*m/s
        gravity : float, default=9.81
            gravity (positive direction points down), unit: m/s^2
        coulomb_fric : float, default=0.0
            friction term, (independent of magnitude of velocity), unit: Nm
        inertia : float, default=None
            inertia of the pendulum (defaults to point mass inertia)
            unit: kg*m^2
        torque_limit: float, default=np.inf
            maximum torque that the motor can apply, unit: Nm
        taylorApprox_order: int, default=1
            order of the taylor approximation of the sine term
        """

        self.m = mass
        self.l = length
        self.b = damping
        self.g = gravity
        self.coulomb_fric = coulomb_fric
        if inertia is None:
            self.inertia = mass*length*length
        else:
            self.inertia = inertia

        self.torque_limit = torque_limit

        self.dof = 1
        self.n_actuators = 1
        self.base = [0, 0]
        self.n_links = 1
        self.workspace_range = [[-1.2*self.l, 1.2*self.l],
                                [-1.2*self.l, 1.2*self.l]]
        self.order = taylorApprox_order

    def forward_dynamics(self, state, tau):

        """
        Computes forward dynamics

        Parameters
        ----------
        state : array like
            len(state)=2
            The state of the pendulum [angle, angular velocity]
            floats, units: rad, rad/s
        tau : float
            motor torque, unit: Nm

        Returns
        -------
            - float, angular acceleration, unit: rad/s^2
        """

        torque = np.clip(tau, -np.asarray(self.torque_limit),
                         np.asarray(self.torque_limit))

        # Taylor approximation of the sine term
        x0 = Variable("theta")
        Tsin_exp = TaylorExpand(np.sin(x0), {x0: np.pi},self.order)
        Tsin = Tsin_exp.Evaluate({x0 : state[0]})

        accn = (torque - self.m * self.g * self.l * Tsin -
                self.b * state[1] -
                np.sign(state[1]) * self.coulomb_fric) / self.inertia
        return accn

    def rhs(self, t, state, tau):

        """
        Computes the integrand of the equations of motion.

        Parameters
        ----------
        t : float
            time, not used (the dynamics of the pendulum are time independent)
        state : array like
            len(state)=2
            The state of the pendulum [angle, angular velocity]
            floats, units: rad, rad/s
        tau : float or array like
            motor torque, unit: Nm

        Returns
        -------
        res : array like
              the integrand, contains [angular velocity, angular acceleration]
        """

        if isinstance(tau, (list, tuple, np.ndarray)):
            torque = tau[0]
        else:
            torque = tau

        accn = self.forward_dynamics(state, torque)

        res = np.zeros(2*self.dof)
        res[0] = state[1]
        res[1] = accn
        return res