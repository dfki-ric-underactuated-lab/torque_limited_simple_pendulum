import numpy as np
import matplotlib.pyplot as plt

from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.controllers.tvlqr.roa.utils import sample_from_ellipsoid, quad_form
from simple_pendulum.controllers.tvlqr.roa.plot import get_ellipse_patch, plotFunnel3d

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
    ctg: np.array
        array that contains the optimal cost to go calculated in each knot point
    '''

    rho00 = 1000                        # set initial guess for rho00, also inf can work
    rho=np.ones(N)*rho00               
    rho[-1]= rhof                       # set final rho from tilqr                     

    ctg=np.ones(N)*np.inf              # Place to store the cost to go

    S_t = controller.tvlqr.S                                # piecewise polynomial from TVLQR
    dt = 0.001                                              # simulation integration step size
    max_succ = 10

    for l in range(1,N):                                  
        k = N-l-1                                             # going backward
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
                max_succ = 10
            else:
                max_succ = max_succ-1

            if max_succ == 0 : # enough successes, the estimation is done
                break

            print(f"knot point {k}, simulation {j}")
            print("rhoHist :")
            print(rho)
            print("termination reason:"+str(termReason))
            print("---")

    return (rho, ctg)

def TVprobRhoVerification(pendulum, controller, rho, x0_t, time, nSimulations, ver_idx):
    '''
    Function to verify the time-variant RoA estimation. This implementation permitts also to choose
    which knot has to be tested. Furthermore the a 3d funnel plot has been implemented.

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
    nSimulations: int
        number of simulations for the verification
    ver_idx: int
        knot point to be verified
    '''

    # figure initialization
    fig = plt.figure(figsize=(18,8))
    fig.suptitle("Verification of RoA guarantee certificate")
    gs = fig.add_gridspec(1, 2)
    ax = fig.add_subplot(gs[0,0])
    ax.set_xlabel("x")
    ax.set_ylabel(r"$\dot{x}$")
 
    dt = 0.001 # simulation time interval

    # plot of the verified ellipse
    S_t = controller.tvlqr.S
    p = get_ellipse_patch(np.array(x0_t).T[ver_idx][0],np.array(x0_t).T[ver_idx][1],rho[ver_idx],S_t.value(time[ver_idx]),linec= "black")
    ax.add_patch(p)
    ax.grid(True)
    plt.title(f"Verified ellipse, knot {ver_idx}")

    ax2 = fig.add_subplot(gs[0,1], projection='3d') 
    plotFunnel3d(rho, controller.tvlqr.S, x0_t, time, ax2) # 3d funnel plot
    ax2.plot(time, x0_t[0],x0_t[1]) # plot the nominal traj

    one_green = False
    one_red = False
    for j in range(1,nSimulations+1):                                                                                                              

        xBar0=sample_from_ellipsoid(S_t.value(time[ver_idx]),rho[ver_idx]) # sample new initial state inside the estimated RoA
        x_i=xBar0+np.array(x0_t).T[ver_idx] 

        sim = Simulator(plant=pendulum) # init the simulation

        T, X, U = sim.simulate(time[ver_idx], x_i, time[-1], dt, controller) # simulating this interval 

        # plotting the checked initial states and resulting trajectories, the color depends on the result  
        finalJ = quad_form(S_t.value(time[-1]),X[-1]-np.array(x0_t).T[-1])
       
        if (finalJ < rho[-1]):
            greenDot = ax.scatter([x_i[0]],[x_i[1]],color="green",marker="o")
            ax2.plot(T, np.array(X).T[0],np.array(X).T[1], color = "green")
            one_green = True
        else:
            redDot = ax.scatter([x_i[0]],[x_i[1]],color="red",marker="o")
            ax2.plot(T, np.array(X).T[0],np.array(X).T[1], color = "red")
            one_red = True

    # managing the dynamic legend of the plot
    if (one_green and one_red):
        ax.legend(handles = [greenDot,redDot,p], 
                    labels = ["successfull initial state","failing initial state", "Initial RoA"])
    elif ((not one_red) and one_green): 
        ax.legend(handles = [greenDot,p], 
                    labels = ["successfull initial state","Initial RoA"])
    else:
        ax.legend(handles = [redDot,p], 
                    labels = ["failing initial state","Initial RoA"])