from pydrake.all import (MathematicalProgram, Solve, Variables, Jacobian)
from simple_pendulum.controllers.lqr.roa.utils import rhoVerification
import numpy as np

def SOSequalityConstrained(pendulum, controller):
    """Estimate the RoA for the closed loop dynamics using the method described by Russ Tedrake in "Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation", 
       Course Notes for MIT 6.832, 2022, "http://underactuated.mit.edu", sec. 9.3.2: "The equality-constrained formulation".
       This is discussed a bit more by Shen Shen and Russ Tedrake in "Sampling Quotient-Ring Sum-of-Squares Programs for Scalable Verification of Nonlinear Systems", 
       Proceedings of the 2020 59th IEEE Conference on Decision and Control (CDC) , 2020., http://groups.csail.mit.edu/robotics-center/public_papers/Shen20.pdf, pg. 2-3. 

    Parameters
    ----------
    pendulum : simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller : simple_pendulum.controllers.lqr.lqr_controller
        configured lqr controller object

    Returns
    -------
    rho : float
        estimated value of rho
    S : np.array
        S matrix from the lqr controller
    """

    # K and S matrices from LQR control
    K = controller.K
    S = controller.S

    # Pendulum parameters
    m = pendulum.m
    l = pendulum.l
    g = pendulum.g
    b = pendulum.b
    torque_limit = pendulum.torque_limit

    # Opt. problem definition
    prog = MathematicalProgram()
    xbar = prog.NewIndeterminates(2, "x") # shifted system state
    rho = prog.NewContinuousVariables(1)[0]

    # Symbolic and polynomial dynamic linearized using Taylor approximation
    xg = [np.pi, 0]  # goal state to stabilize
    x = xbar + xg
    ubar = -K.dot(xbar)[0]
    Tsin = -(x[0]-xg[0]) + (x[0]-xg[0])**3/6 - (x[0]-xg[0])**5/120 + (x[0]-xg[0])**7/5040 
    fn = [x[1], (ubar-b*x[1]-Tsin*m*g*l)/(m*l*l)]

    # Optimal cost-to-go from LQR as Lyapunov candidate
    V = (xbar).dot(S.dot(xbar))
    Vdot = Jacobian([V], xbar).dot(fn)[0]

    # Free multipliers
    lambda_a = prog.NewFreePolynomial(Variables(xbar), 4).ToExpression()
    lambda_b = prog.NewFreePolynomial(Variables(xbar), 4).ToExpression()
    lambda_c = prog.NewFreePolynomial(Variables(xbar), 4).ToExpression()

    # Boundaries due to the saturation 
    u_minus = - torque_limit
    u_plus = torque_limit
    fn_minus = [x[1], (u_minus-b*x[1]-Tsin*m*g*l)/(m*l*l)] 
    Vdot_minus = V.Jacobian(xbar).dot(fn_minus)
    fn_plus = [x[1], (u_plus-b*x[1]-Tsin*m*g*l)/(m*l*l)] 
    Vdot_plus = V.Jacobian(xbar).dot(fn_plus)

    # Define the Lagrange multipliers.
    lambda_1 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_2 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_3 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_4 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()

    # Optimization constraints and cost
    prog.AddSosConstraint(((xbar.T).dot(xbar)**2)*(V - rho) + lambda_a*(Vdot_minus) + lambda_1*(-u_minus+ubar))
    prog.AddSosConstraint(((xbar.T).dot(xbar)**2)*(V - rho) + lambda_b*(Vdot) + lambda_2*(u_minus-ubar) + lambda_3*(-u_plus+ubar))
    prog.AddSosConstraint(((xbar.T).dot(xbar)**2)*(V - rho) + lambda_c*(Vdot_plus) + lambda_4*(u_plus-ubar))
    prog.AddConstraint(rho >= 0)
    prog.AddCost(-rho)

    # Solve the problem
    result = Solve(prog)
    return [result.GetSolution(rho), S]

def SOSlineSearch(pendulum, controller):
    """Simple line search(bisection method) on rho that search for the maximum rho which satisfies the Lyapunov conditions in order to obtain an estimate  of the RoA for the closed loop dynamics.

    Parameters
    ----------
    pendulum : simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller : simple_pendulum.controllers.lqr.lqr_controller
        configured lqr controller object

    Returns
    -------
    rho : float
        estimated value of rho
    S : np.array
        S matrix from the lqr controller
    """

    rho_max = 100
    rho_min = 0
    max_iter = 20

    for i in range(max_iter):
        rho_check = rho_min + (rho_max-rho_min)/2
        # Verification of the Lyapunov conditions
        result = rhoVerification(rho_check, pendulum, controller)

        if (result):
            rho_min = rho_check
        else: rho_max = rho_check

    return [rho_check, controller.S]