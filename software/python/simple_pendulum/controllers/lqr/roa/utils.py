from scipy.spatial.transform import Rotation as R
from scipy import linalg
from scipy.special import gamma, factorial
import numpy as np
from pydrake.all import (MathematicalProgram, Solve, Variables, Jacobian)



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