import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches

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
