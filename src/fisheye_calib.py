"""
Created on Wed Sep 27 10:54:23 2017

@author:    Carlo Sferrazza
            PhD Candidate
            Institute for Dynamics Systems and Control
            ETH Zurich
            
All this functions are a translation of the C++ code provided with the Scaramuzza's 
OCamCalib Toolbox and are better described here: 
https://sites.google.com/site/scarabotix/ocamcalib-toolbox
"""
import numpy as np

#===================================================================================
# Function to Reads file containing the ocamcalib parameters exported from the 
# Matlab toolbox
# Input: filename path
# Output: Ocamcalib model
#===================================================================================
def get_ocam_model(filename):
    ocam_model = {}

    with open(filename) as f:
        lines = [l for l in f]
        
        # Polynomial coefficients for the DIRECT mapping function (ocam_model.ss in
        # MATLAB). These are used by cam2world
        l = lines[2]
        data = l.split()
        ocam_model['length_pol'] = int(data[0])
        ocam_model['pol'] = [float(d) for d in data[1:]]
        
        # Polynomial coefficients for the inverse mapping function 
        # (ocam_model.invpol in MATLAB). These are used by world2cam
        l = lines[6]
        data = l.split()
        ocam_model['length_invpol'] = int(data[0])
        ocam_model['invpol'] = [float(d) for d in data[1:]]
        
        # Center
        l = lines[10]
        data = l.split()
        ocam_model['xc'] = float(data[0])
        ocam_model['yc'] = float(data[1])
        
        # affine parameters "c", "d", "e"
        l = lines[14]
        data = l.split()
        ocam_model['c'] = float(data[0])
        ocam_model['d'] = float(data[1])
        ocam_model['e'] = float(data[2])
                
        # Image size
        l = lines[18]
        data = l.split()
        ocam_model['height'] = int(data[0])
        ocam_model['width'] = int(data[1])

    return ocam_model

#===================================================================================
# Function to back-projects a pixel point 'point2D' onto the unit sphere 'vector_3D' 
# M=[X;Y;Z] is a 3xN matrix with which contains the coordinates of the vectors 
# emanating from the single-effective-viewpoint to the unit sphere, therefore, 
# X^2 + Y^2 + Z^2 = 1.
# Input: - Pixel point: 2x1 matrix containing the pixel coordinate of the image
#                       point [x,y]
#        - Ocam model: Contains the model of the calibrated camera
# Output: 3D unit vector
#===================================================================================    
def cam2world(point2D, ocam_model):
    vector_3D = []
    # Calculate hypothetical image plane points from the fisheye camera model
    invdet = 1.0/(ocam_model['c']-ocam_model['d']*ocam_model['e'])
    xp = invdet*((point2D[0] - ocam_model['xc']) - \
                  ocam_model['d']*(point2D[1] - ocam_model['yc']))
    yp = invdet*(-ocam_model['e']*(point2D[0]-ocam_model['cx']) + \
                  ocam_model['c']*(point2D[1]-ocam_model['cy']))
    r = np.linalg.norm([xp,yp])
    zp = ocam_model['pol'][0]
    r_i = 1.0
    
    # Calculate the 3D coordinates of its correspondent optical ray given an image
    # point
    for i in range(1,ocam_model['length_pol']):
        r_i *= r
        zp += r_i*ocam_model['pol'][i]

    # Normalizes coordinates so that they have unit length (projection onto the
    # unit sphere)
    invnorm = 1.0/np.linalg.norm([xp,yp,zp])
    vector_3D.append(invnorm*xp)
    vector_3D.append(invnorm*yp)
    vector_3D.append(invnorm*zp)
        
    return vector_3D

#===================================================================================
# Function to back-projects a pixel point 'point2D' onto real undistort pixel  
# coordinate
# Input: - Pixel point: 2x1 matrix containing the pixel coordinate of the image
#                       point [x,y]
#        - Ocam model: Contains the model of the calibrated camera
# Output: 2D vector
#===================================================================================    
def undistort_point(point2D, ocam_model):
    vector2D = []
    # Calculate hypothetical image plane points from the fisheye camera model
    invdet = 1.0/(ocam_model['c']-ocam_model['d']*ocam_model['e'])
    
    xp = invdet*((point2D[0] - ocam_model['cx']) - \
                  ocam_model['d']*(point2D[1] - ocam_model['cy']))

    yp = invdet*(-ocam_model['e']*(point2D[0]-ocam_model['cx']) + \
                  ocam_model['c']*(point2D[1]-ocam_model['cy']))
    
    vector2D.append(xp)
    vector2D.append(yp)
            
    return vector2D

#===================================================================================
# Own function to map a pixel point 'point2D' onto new pixel point(photodetector) 
# from camera center frame
# Input: - Pixel point: 2x1 matrix containing the pixel coordinate of the image
#                       point [x,y]
#        - Ocam model: Contains the model of the calibrated camera
# Output: 2D vector
#=================================================================================== 

def new_pixel_position(point2D, ocam_model):
    vector2D = []

    xp = point2D[0] - ocam_model['cx'] #+ ocam_model['pdx']
    yp = point2D[1] - ocam_model['cy'] #+ ocam_model['pdy']

    vector2D.append(xp)
    vector2D.append(yp)
            
    return vector2D