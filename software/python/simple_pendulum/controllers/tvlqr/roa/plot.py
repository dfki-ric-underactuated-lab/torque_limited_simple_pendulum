import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import matplotlib as mpl
import mpl_toolkits.mplot3d.art3d as art3d
mpl.use('WebAgg')
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from matplotlib.collections import LineCollection

from simple_pendulum.controllers.tvlqr.roa.utils import projectedEllipseFromCostToGo, getEllipseContour

def get_ellipse_params(rho,M):
    """
    Returns ellipse params (excl center point)
    """

    #eigenvalue decomposition to get the axes
    w,v=np.linalg.eigh(M/rho) 

    try:
        #let the smaller eigenvalue define the width (major axis*2!)
        width = 2/float(np.sqrt(w[0]))
        height = 2/float(np.sqrt(w[1]))
        
        #the angle of the ellipse is defined by the eigenvector assigned to the smallest eigenvalue (because this defines the major axis (width of the ellipse))
        angle = np.rad2deg(np.arctan2(v[:,0][1],v[:,0][0]))

    except:
        print("paramters do not represent an ellipse.")

    return width,height,angle

def get_ellipse_patch(px,py,rho,M,alpha_val=1,linec="red",facec="none",linest="solid"):
    """
    return an ellipse patch
    """
    w,h,a = get_ellipse_params(rho,M)
    return patches.Ellipse((px,py), w, h, a, alpha=alpha_val,ec=linec,facecolor=facec,linestyle=linest)

def plot_ellipse(px,py,rho, M, save_to=None, show=True):
    p=get_ellipse_patch(px,py,rho,M)
    
    fig, ax = plt.subplots()
    ax.add_patch(p)
    l=np.max([p.width,p.height])

    ax.set_xlim(px-l/2,px+l/2)
    ax.set_ylim(py-l/2,py+l/2)

    ax.grid(True)

    if not (save_to is None):
        plt.savefig(save_to)
    if show:
        plt.show()

def plotFirstLastEllipses(rho, x0, goal, x0_traj, S_t, time):
    fig,ax = plt.subplots(1,2, figsize=(18,8))
    p0 = get_ellipse_patch(np.array(x0_traj).T[0][0],np.array(x0_traj).T[0][1],rho[0],S_t.value(time[0]),linec= "black")
    ax[0].scatter([x0[0]],[x0[1]],color="black",marker="o")
    ax[0].add_patch(p0)
    ax[0].grid(True)
    ax[0].set_xlabel("x")
    ax[0].set_ylabel(r"$\dot{x}$")
    ax[0].title.set_text('First ellipse')
    pl = get_ellipse_patch(np.array(x0_traj).T[-1][0],np.array(x0_traj).T[-1][1],rho[-1],S_t.value(time[-1]),linec= "black")
    ax[1].scatter([goal[0]],[goal[1]],color="black",marker="x")
    ax[1].add_patch(pl)
    ax[1].grid(True)
    ax[1].set_xlabel("x")
    ax[1].set_ylabel(r"$\dot{x}$")
    ax[1].title.set_text('Last ellipse')

def plotRhoEvolution(rho, x0_traj, time, N):
    fig = plt.figure()
    plt.title("rho evolution")
    ax = fig.add_subplot()
    ax.plot(np.arange(N),rho,color="yellow", label = "final rho")
    ax2=ax.twinx()
    d = round(len(time)/N)
    ang = np.abs([x0_traj[0][d*p] for p in range(N)])
    vel = np.abs([x0_traj[1][d*p] for p in range(N)])
    ax2.plot(np.arange(N),ang,color="blue", label = "nominal traj, angle")
    ax2.plot(np.arange(N),vel,color="red", label = "nominal traj, velocity")
    ax.legend(loc = "upper left")
    ax2.legend(loc = "upper right")
    ax.set_xlabel("Number of steps")

##############
# Funnels plot
##############

def plotFunnel3d(rho, S, x0, time, ax):
    '''
    Function to draw a discrete 3d funnel plot. Basically we are plotting a 3d ellipse patch in each 
    knot point.

    Parameters
    ----------
    rho : np.array
        array that contains the estimated rho value for all the knot points
    S: np.array
        array of matrices that define ellipses in all the knot points, from tvlqr controller.
    x0: np.array 
        pre-computed nominal trajectory
    time: np.array
        time array related to the nominal trajectory
    ax: matplotlib.axes
        axes of the plot where we want to add the 3d funnel plot, useful in the verification function.
    '''

    for i in range(len(rho)):
        # Drawing the main ellipse
        ctg=np.asarray(S.value(time[i]))
        labels=["theta [rad]","theta_dot [rad/s]"]
        s0=0
        s1=1

        w,h,a=projectedEllipseFromCostToGo(s0,s1,[rho[i]],[ctg])

        elliIn=patches.Ellipse((x0[s0][i],x0[s1][i]), 
                                w[0], 
                                h[0],
                                a[0],ec="black",linewidth=1.25, color = "green", alpha = 0.1)
        ax.add_patch(elliIn)
        art3d.pathpatch_2d_to_3d(elliIn, z=time[i], zdir="x") # 3d plot of a patch

    plt.title("3d resulting Funnel")
    ax.set_xlabel("time [s]")
    ax.set_ylabel(labels[s0])
    ax.set_zlabel(labels[s1])
    ax.set_xlim(0, time[-1])
    ax.set_ylim(-6, 6)
    ax.set_zlim(-6, 6)

def plotFunnel(rho, S, x0, time):
    '''
    Function to draw a continue 2d funnel plot. This implementation makes use of the convex hull concept
    as done in the MATLAB code of the Robot Locomotion Group (https://groups.csail.mit.edu/locomotion/software.html).

    Parameters
    ----------
    rho : np.array
        array that contains the estimated rho value for all the knot points
    S: np.array
        array of matrices that define ellipses in all the knot points, from tvlqr controller.
    x0: np.array 
        pre-computed nominal trajectory
    time: np.array
        time array related to the nominal trajectory
    '''

    # figure initialization
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.title("2d resulting Funnel")
    plt.grid(True)
    labels=["theta [rad]","theta_dot [rad/s]"]
    s0=0
    s1=1
    ax.set_xlabel(labels[s0])
    ax.set_ylabel(labels[s1])
    ax.set_xlim(-2, 4)
    ax.set_ylim(-20, 20)

    ax.plot(x0[s0],x0[s1]) # plot of the nominal trajectory

    for i in range(len(rho)-1):
        c_prev = getEllipseContour(S.value(time[i]),rho[i], np.array(x0).T[i]) # get the contour of the previous ellipse
        c_next = getEllipseContour(S.value(time[i+1]),rho[i+1], np.array(x0).T[i+1]) # get the contour of the next ellipse
        points = np.vstack((c_prev,c_next))

        # plot the convex hull of the two contours
        hull = ConvexHull(points) 
        line_segments = [hull.points[simplex] for simplex in hull.simplices]
        ax.add_collection(LineCollection(line_segments,
                                     colors='green',
                                     linestyle='solid'))