import numpy as np

from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.tvlqr.roa.utils import sample_from_ellipsoid, quad_form

def TVprobRhoComputation(pendulum, controller, x0_t, time, N, nSimulations, rhof):
    '''
    Function to perform the time-variant RoA estimation. This implementation has been inpired by 
    "Feedback-Motion-Planning with Simulation-Based LQR-Trees" by Philipp Reist, Pascal V. Preiswerk 
    and Russ Tedrake (https://groups.csail.mit.edu/robotics-center/public_papers/Reist15.pdf).

    Parameters
    ----------
    pendulum: simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller: simple_pendulum.controllers.tvlqr.tvlqr
        configured tvlqr controller object
    x0_t: np.array 
        pre-computed nominal trajectory
    time: np.array
        time array related to the nominal trajectory
    N: int
        number of considered knot points
    nSimulations: int
        max number of simulations for each ellipse estimation
    rhof: float
        final rho value, from the time-invariant RoA estimation

    Returns
    -------
    rho : np.array
        array that contains the estimated rho value for all the knot points
    S: np.array
        array that contains the S matrix in each knot point
    '''

    rho00 = 1000                        # set initial guess for rho00, also inf can work
    rho=np.ones(N)*rho00               
    rho[-1]= rhof                       # set final rho from tilqr                     

    ctg=np.ones(N)*np.inf              # Place to store the cost to go

    S_t = controller.tvlqr.S                                # piecewise polynomial from TVLQR
    dt = 0.001                                              # simulation integration step size
    max_succ = nSimulations

    S = np.matrix(S_t.value(time[N-1])).flatten().T
    for l in range(1,N):                                  
        k = N-l-1  # going backward
        S = np.hstack((np.matrix(S_t.value(time[k])).flatten().T,S))                                          
        for j in range(nSimulations):                                                  
            xBark=sample_from_ellipsoid(S_t.value(time[k]),rho[k])        # sample new initial state
            xk=xBark+np.array(x0_t).T[k]                                               

            sim = Simulator(plant=pendulum) # init the simulation

            ctg[k]= quad_form(S_t.value(time[k]),xBark)                            
            termReason=0                                     # Assume that we do not need to shrink by default

            T, X, U = sim.simulate(time[k], xk, time[k+1], dt, controller) # simulating this interval   
            xkPlus1= X[-1]
            xBarkPlus1=xkPlus1-np.array(x0_t).T[k+1]                  

            ctg[k+1]= quad_form(S_t.value(time[k+1]),xBarkPlus1)     # compute cost to go after simulation

            if ctg[k+1] > rho[k+1]: # check if we exit from the next ellipse
                termReason=1
                rho[k]= min(ctg[k],rho[k])
                max_succ = nSimulations
            else:
                max_succ = max_succ-1

            if max_succ == 0 : # enough successes, the estimation is done
                break

            print(f"knot point {k}, simulation {j}")
            print("rhoHist :")
            print(rho)
            print("termination reason:"+str(termReason))
            print("---")

    return (rho, S)