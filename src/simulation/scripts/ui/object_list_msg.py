'''
    this module contains global dicts
'''

obj_list_msg = {
    'obj_id' : [],
    'geometric' : ['x', 'y', 'vx', 'vy', 'ax', 'ay', 'yaw', 'yawrate'],
#     'covariance' : [],
    'dimension' : ['length', 'width', 'height'],
    'prop_existence' : [],
    'prop_mov' : [],
    'classification' : ['car', 'truck', 'motorcycle', 'bicycle', 'pedestrian', 'stacionary', 'other'],
    'features' : ['FL', 'FM', 'FR', 'MR', 'RR', 'RM', 'RL', 'ML']
    }

values_units = {
    'x' : 'distance[m]',
    'y' : 'distance[m]',
    'vx' : 'velocity[m/s]',
    'vy' : 'velocity[m/s]',
    'ax' : 'acceleration[m/s^2]',
    'ay' : 'acceleration[m/s^2]',
    'yaw' : 'angle[Deg]',
    'yawrate' : 'angular frequency [Deg/m]',
    'length' : 'dimension[m]',
    'width' : 'dimension[m]',
    'height' : 'dimension[m]',
    }

units = {
    'x' : '[m]',
    'y' : '[m]',
    'vx' : '[m/s]',
    'vy' : '[m/s]',
    'ax' : '[m/s^2]',
    'ay' : '[m/s^2]',
    'yaw' : '[Deg]',
    'yawrate' : '[Deg/m]',
    'length' : '[m]',
    'width' : '[m]',
    'height' : '[m]',
    }


    
    
    
    
    