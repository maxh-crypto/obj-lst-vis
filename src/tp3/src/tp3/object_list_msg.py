'''
    this module contains the categories and 
    attributes from ObjectList.msg as enums
'''

obj_list_msg = {
    'obj_id' : [],
    'geometric' : ['x', 'y', 'vx', 'vy', 'ax', 'ay', 'yaw', 'yawrate'],
    'covariance' : [],
    'dimension' : ['length', 'width', 'height'],
    'prop_existence' : [],
    'prop_mov' : [],
    'classification' : ['car', 'truck', 'motorcycle', 'bicycle', 'pedestrian', 'stacionary', 'other'],
    'features' : ['FL', 'FM', 'FR', 'MR', 'RR', 'RM', 'RL', 'ML']
    }

units_tex = {
    'x' : 'm',
    'y' : 'm',
    'vx' : 'm/s',
    'vy' : 'm/s',
    'ax' : 'm/s^2',
    'ay' : 'm/s^2',
    'yaw' : '\N{DEGREE_SIGN}',
    'yawrate' : '\N{DEGREE_SIGN}/s',
    'length' : 'm',
    'width' : 'm',
    'height' : 'm',
    }

units = {
    'x' : 'm',
    'y' : 'm',
    'vx' : 'm/s',
    'vy' : 'm/s',
    'ax' : 'm/s^2',
    'ay' : 'm/s^2',
    'yaw' : 'degree',
    'yawrate' : 'degree/s',
    'length' : 'm',
    'width' : 'm',
    'height' : 'm',
    }

    
    
    
    
    