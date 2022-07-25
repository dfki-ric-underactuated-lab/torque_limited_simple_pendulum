import numpy as np
from pydrake.all import Variables, Solve, MathematicalProgram
from pydrake.symbolic import Polynomial as simb_poly
from pydrake.symbolic import sin, TaylorExpand

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

def projectedEllipseFromCostToGo(s0Idx,s1Idx,rho,M):
    """
    Returns ellipses in the plane defined by the states matching the indices s0Idx and s1Idx for funnel plotting.
    """
    ellipse_widths=[]
    ellipse_heights=[]
    ellipse_angles=[]
    
    #loop over all values of rho
    for idx, rho in enumerate(rho):
        #extract 2x2 matrix from S
        S=M[idx]
        ellipse_mat=np.array([[S[s0Idx][s0Idx],S[s0Idx][s1Idx]],
                              [S[s1Idx][s0Idx],S[s1Idx][s1Idx]]])*(1/rho)
        
        #eigenvalue decomposition to get the axes
        w,v=np.linalg.eigh(ellipse_mat) 

        try:
            #let the smaller eigenvalue define the width (major axis*2!)
            ellipse_widths.append(2/float(np.sqrt(w[0])))
            ellipse_heights.append(2/float(np.sqrt(w[1])))

            #the angle of the ellipse is defined by the eigenvector assigned to the smallest eigenvalue (because this defines the major axis (width of the ellipse))
            ellipse_angles.append(np.rad2deg(np.arctan2(v[:,0][1],v[:,0][0])))
        except:
            continue
    return ellipse_widths,ellipse_heights,ellipse_angles

def getEllipseContour(S,rho,xg):
    """
    Returns a certain number(nSamples) of sampled states from the contour of a given ellipse.

    Parameters
    ----------
    S : np.array
        Matrix S that define one ellipse
    rho : np.array
        rho value that define one ellipse
    xg : np.array
        center of the ellipse

    Returns
    -------
    c : np.array
        random vector of states from the contour of the ellipse
    """
    nSamples = 1000 
    c = sample_from_ellipsoid(S,rho,r_i = 0.99) +xg
    for i in range(nSamples-1):
        xBar = sample_from_ellipsoid(S,rho,r_i = 0.99)
        c = np.vstack((c,xBar+xg))
    return c

#########################
# SOS Funnels computation
#########################

def TVrhoSearch(pendulum, controller, knot, time, h_map, rho_t):
    """
    V step of the bilinear alternationturn used in the SOS funnel estimation.

    Parameters
    ----------
    pendulum: simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller: simple_pendulum.controllers.tvlqr.tvlqr
        configured tvlqr controller object
    knot: int
        number of considered knot point
    time: np.array
        time array related to the nominal trajectory
    h_map: Dict[pydrake.symbolic.Monomial, pydrake.symbolic.Expression]
        map of the coefficients of the multiplier obtained from the multiplier step
    rho_t: np.array
        array that contains the evolving estimation of the rho values for each knot point

    Returns
    -------
    fail : boolean
        gives info about the correctness of the optimization problem
    rho_opt: float
        optimized rho value for this knot point
    """

    # Sampled constraints
    t_iplus1 = time[knot]
    t_i = time[knot-1]
    dt = t_iplus1 - t_i

    # Pendulum parameters
    m = pendulum.m
    l = pendulum.l
    g = pendulum.g
    b = pendulum.b
    torque_limit = pendulum.torque_limit

    # Opt. problem definition
    prog = MathematicalProgram()
    xbar = prog.NewIndeterminates(2, "x") # shifted system state
    rho_i = prog.NewContinuousVariables(1)[0]
    rho_dot_i = (rho_t[knot] - rho_i)/dt
    prog.AddCost(-rho_i)
    prog.AddConstraint(rho_i >= 0)

    # Dynamics definition
    u0 = controller.tvlqr.u0.value(t_i)[0][0]
    K_i = controller.tvlqr.K.value(t_i)[0]
    ubar = - K_i.dot(xbar)
    u = (u0 + ubar) #input

    x0 = controller.tvlqr.x0.value(t_i)
    x = (xbar + x0)[0]
    Tsin_x = TaylorExpand(sin(x[0]),{xbar[0]: 0}, 5)
    fn = [xbar[1], (ubar -b*xbar[1]-(Tsin_x-np.sin(x0[0]))*m*g*l)/(m*l*l)] # shifted state dynamics

    # Lyapunov function and its derivative
    S0_t = controller.tvlqr.S
    S0_i = S0_t.value(t_i)
    S0_iplus1 = S0_t.value(t_iplus1)
    S0dot_i = (S0_iplus1-S0_i)/dt
    V_i = (xbar).dot(S0_i.dot(xbar))
    Vdot_i_x = V_i.Jacobian(xbar).dot(fn)
    Vdot_i_t = xbar.dot(S0dot_i.dot(xbar))
    Vdot_i = Vdot_i_x + Vdot_i_t

    # Boundaries due to the saturation 
    u_minus = - torque_limit -u0
    u_plus = torque_limit -u0
    fn_minus = [xbar[1], (u_minus -b*xbar[1]-(Tsin_x-np.sin(x0[0]))*m*g*l)/(m*l*l)]
    Vdot_minus = Vdot_i_t + V_i.Jacobian(xbar).dot(fn_minus)
    fn_plus = [xbar[1], (u_plus -b*xbar[1]-(Tsin_x-np.sin(x0[0]))*m*g*l)/(m*l*l)]
    Vdot_plus = Vdot_i_t + V_i.Jacobian(xbar).dot(fn_plus)

    # Multipliers definition
    lambda_1 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_2 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_3 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_4 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()

    # Retriving the mu result 
    h = prog.NewFreePolynomial(Variables(xbar), 4)
    ordered_basis = list(h.monomial_to_coefficient_map().keys())
    zip_iterator = zip(ordered_basis, list(h_map.values()))
    h_dict = dict(zip_iterator)
    h = simb_poly(h_dict)
    h.RemoveTermsWithSmallCoefficients(4)
    mu_ij = h.ToExpression()

    # Optimization constraints 
    eps = 0
    constr_minus = eps - (Vdot_minus) +rho_dot_i - mu_ij*(V_i - rho_i) + lambda_1*(-u_minus+ubar) 
    constr = eps - (Vdot_i) + rho_dot_i - mu_ij*(V_i - rho_i) + lambda_2*(u_minus-ubar) + lambda_3*(-u_plus+ubar) 
    constr_plus = eps - (Vdot_plus) +rho_dot_i - mu_ij*(V_i - rho_i) + lambda_4*(u_plus-ubar) 

    for c in [constr_minus, constr, constr_plus]:
        prog.AddSosConstraint(c)

    # Solve the problem
    result = Solve(prog)
    rho_opt = result.GetSolution(rho_i)

    # failing checker
    fail = not result.is_success()
    if fail:
        print("rho step Error")

    return fail, rho_opt

def TVmultSearch(pendulum, controller, knot, time, rho_t):
    """
    Multiplier step of the bilinear alternationturn used in the SOS funnel estimation.

    Parameters
    ----------
    pendulum: simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller: simple_pendulum.controllers.tvlqr.tvlqr
        configured tvlqr controller object
    knot: int
        number of considered knot point
    time: np.array
        time array related to the nominal trajectory
    rho_t: np.array
        array that contains the evolving estimation of the rho values for each knot point

    Returns
    -------
    fail : boolean
        gives info about the correctness of the optimization problem
    h_map: Dict[pydrake.symbolic.Monomial, pydrake.symbolic.Expression]
        map of the coefficients of the multiplier obtained from the multiplier step
    eps: float
        optimal cost of the optimization problem that can be useful in the V step
    """

    # Sampled constraints
    t_iplus1 = time[knot]
    t_i = time[knot-1]
    dt = t_iplus1 - t_i

    # Pendulum parameters
    m = pendulum.m
    l = pendulum.l
    g = pendulum.g
    b = pendulum.b
    torque_limit = pendulum.torque_limit

    # Opt. problem definition
    prog = MathematicalProgram()
    xbar = prog.NewIndeterminates(2, "x") # shifted system state
    gamma = prog.NewContinuousVariables(1)[0]
    prog.AddCost(gamma)
    prog.AddConstraint(gamma <= 0)

    # Dynamics definition
    u0 = controller.tvlqr.u0.value(t_i)[0][0]
    K_i = controller.tvlqr.K.value(t_i)[0]
    ubar = - K_i.dot(xbar)
    u = (u0 + ubar) #input

    x0 = controller.tvlqr.x0.value(t_i)
    x = (xbar + x0)[0]
    Tsin_x = TaylorExpand(sin(x[0]),{xbar[0]: 0}, 5)
    fn = [xbar[1], (ubar -b*xbar[1]-(Tsin_x-np.sin(x0[0]))*m*g*l)/(m*l*l)] # shifted state dynamics

    # Lyapunov function and its derivative
    S0_t = controller.tvlqr.S
    S0_i = S0_t.value(t_i)
    S0_iplus1 = S0_t.value(t_iplus1)
    S0dot_i = (S0_iplus1-S0_i)/dt
    V_i = (xbar).dot(S0_i.dot(xbar))
    Vdot_i_x = V_i.Jacobian(xbar).dot(fn)
    Vdot_i_t = xbar.dot(S0dot_i.dot(xbar))
    Vdot_i = Vdot_i_x + Vdot_i_t

    # Boundaries due to the saturation 
    u_minus = - torque_limit -u0
    u_plus = torque_limit -u0
    fn_minus = [xbar[1], (u_minus -b*xbar[1]-(Tsin_x-np.sin(x0[0]))*m*g*l)/(m*l*l)]
    Vdot_minus = Vdot_i_t + V_i.Jacobian(xbar).dot(fn_minus)
    fn_plus = [xbar[1], (u_plus -b*xbar[1]-(Tsin_x-np.sin(x0[0]))*m*g*l)/(m*l*l)]
    Vdot_plus = Vdot_i_t + V_i.Jacobian(xbar).dot(fn_plus)

    # Multipliers definition
    h = prog.NewFreePolynomial(Variables(xbar), 4)
    mu_ij = h.ToExpression()
    lambda_1 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_2 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_3 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()
    lambda_4 = prog.NewSosPolynomial(Variables(xbar), 4)[0].ToExpression()

    # rho dot definition
    rho_i = rho_t[knot-1]
    rho_iplus1 = rho_t[knot]
    rho_dot_i = (rho_iplus1 - rho_i)/dt

    # Optimization constraints 
    constr_minus = gamma - (Vdot_minus) +rho_dot_i - mu_ij*(V_i - rho_i) + lambda_1*(-u_minus+ubar)
    constr = gamma - (Vdot_i) + rho_dot_i - mu_ij*(V_i - rho_i) + lambda_2*(u_minus-ubar) + lambda_3*(-u_plus+ubar)
    constr_plus = gamma - (Vdot_plus) +rho_dot_i - mu_ij*(V_i - rho_i) + lambda_4*(u_plus-ubar)

    for c in [constr_minus, constr, constr_plus]:
        prog.AddSosConstraint(c)

    # Solve the problem and store the polynomials
    result_mult = Solve(prog)  
    h_map = result_mult.GetSolution(h).monomial_to_coefficient_map()
    eps = result_mult.get_optimal_cost()

    # failing checker
    fail = not result_mult.is_success()

    return fail, h_map, eps