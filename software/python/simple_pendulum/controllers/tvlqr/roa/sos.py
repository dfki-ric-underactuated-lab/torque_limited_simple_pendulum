from simple_pendulum.controllers.tvlqr.roa.utils import TVrhoSearch, TVmultSearch
import numpy as np

#########################################################################################################################################
# Time-variyng Region of Attraction estimation
#########################################################################################################################################

def TVsosRhoComputation(pendulum, controller, time, N, rhof):
    """
    Bilinear alternationturn used for the SOS funnel estimation.

    Parameters
    ----------
    pendulum: simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller: simple_pendulum.controllers.tvlqr.tvlqr
        configured tvlqr controller object
    time: np.array
        time array related to the nominal trajectory
    N: int
        number of considered knot points
    rhof: float
        final rho value, from the time-invariant RoA estimation

    Returns
    -------
    rho_t : np.array
        array that contains the estimated rho value for all the knot points
    S: np.array
        array that contains the S matrix in each knot point
    """

    # Initial rho(t) definition (exponential init)
    c = 2
    rho_t = rhof*np.exp(c*(time-time[-1])/time[-1])
    cost_prev = np.inf
    
    # Bilinear SOS alternation for improving the first guess
    convergence = False
    while(not convergence):
        h_maps = []
        gamma_min = 0
        for knot in np.flip([i for i in range(1,round(N))]):
            print("---------------")
            print(f"Multiplier step in knot {knot-1}:")

            fail = True
            while(fail):
                # Search for a multiplier, fixing rho
                (fail, h_map, gamma_i) = TVmultSearch(pendulum, controller, knot, time, rho_t)
                if fail:
                    rho_t[knot-1] = 0.8*rho_t[knot-1]
            print(f"The feasible rho is {rho_t[knot-1]}")
            h_maps = np.append(h_map, h_maps)
            gamma_min = min(gamma_min, gamma_i) # Not used now, but can be useful for the V step

        for knot in np.flip([i for i in range(1,round(N))]):
            print("---------------")
            print(f"V step in knot {knot-1}:")

            fail = True
            while(fail):
                # Search for rho, fixing the multiplier       
                (fail, rho_opt) = TVrhoSearch(pendulum, controller, knot, time, h_maps[knot-1], rho_t)
                if fail:
                    gamma_min = 0.75*gamma_min
                else:
                    print("RHO IMPROVES!!")
            rho_t[knot-1] = rho_opt
        
        # Check for convergence
        eps = 0.1
        if((cost_prev - np.sum(rho_t))/cost_prev < eps): 
            convergence = True  
        cost_prev = np.sum(rho_t)
    
    print("---------------")
    return (rho_t, controller.tvlqr.S)