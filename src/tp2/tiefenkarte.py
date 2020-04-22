import glob
import os
import sys
import numpy as np
import cv2
import carla
import random
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass




IM_WIDTH = 800
IM_HEIGTH = 600


def processimage(image):
    xrange = np.array([390,410])
    yrange = np.array([390,410])
    # Data processing
    array = np.array(image.raw_data)
    i = array.reshape(IM_HEIGTH,IM_WIDTH,4)
    i2 = i[:,:,:3]
    pixel = i2[400,400]

    # Entfernung aus GBR (nicht RGB!) Daten berechnen
    normalized = (pixel[2] + pixel[1] * 256 + pixel[0] * 256 * 256) / (256 * 256 * 256 - 1)
    in_meters = 1000 * normalized

    poi = i2[290:510, 290:510]
    poi2 = cv2.cvtColor(poi, cv2.COLOR_BGR2GRAY)
    gaus = cv2.adaptiveThreshold(poi2, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115,1)
    #print('distance of pixel at frame ',image.frame_number, ' is ', in_meters, ' meters')

    # Visualisierung des Pixel of Interest
    #blue = [255,0,0]
    #i2[390:410, 390:410] = blue
    
    # Graphische Ausgabe des Sensorbilds
    cv2.imshow('', gaus)
    cv2.waitKey(1)
    #speed(in_meters)
    return in_meters


actor_list = []

try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Blueprints erstellen
    # Blueprints = Fahrzeuge und Fußgänger
    blueprint_library = world.get_blueprint_library()
    #bp_1 = random.choice(blueprint_library.filter('vehicle'))
    bp_1 = blueprint_library.filter('model3')[0]
    bp_2 = blueprint_library.filter('model3')[0]

    if bp_1.has_attribute('color'):
            color = random.choice(bp_1.get_attribute('color').recommended_values)
            bp_1.set_attribute('color', color)
    
    if bp_2.has_attribute('color'):
            color = random.choice(bp_2.get_attribute('color').recommended_values)
            bp_2.set_attribute('color', color)
    

    # Spwan Points festlegen und Fahrzeugeigenschaften festlegen
    # Koordination durch Probieren
    # spawn_point = random.choice(world.get_map().get_spawn_points())
    spawn_point_bp_1 = carla.Transform(carla.Location(x=130,y=-6,z=1), carla.Rotation(yaw=180))
    spawn_point_bp_2 = carla.Transform(carla.Location(x=140,y=-6,z=1), carla.Rotation(yaw=180))
    vehicle_1 = world.spawn_actor(bp_1, spawn_point_bp_1)
    vehicle_2 = world.spawn_actor(bp_2, spawn_point_bp_2)
    #vehicle_1.set_autopilot(True)
    vehicle_1.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    vehicle_2.apply_control(carla.VehicleControl(throttle=0.8, steer=0.0))
    actor_list.append(vehicle_1)
    actor_list.append(vehicle_2)

    cam_bp_2 = blueprint_library.find('sensor.camera.rgb')
    #cam_bp_2.set_attribute('fov', '110')
    cam_bp_2.set_attribute('image_size_x', '800')
    cam_bp_2.set_attribute('image_size_y', '600')
    transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    sensor = world.spawn_actor(cam_bp_2, transform, attach_to=vehicle_2)
    

    cc = carla.ColorConverter.Depth
    #sensor.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame, cc))
    sensor.listen(lambda image: (processimage(image)))

    time.sleep(50)



finally:
    for actor in actor_list:
        actor.destroy()
        print("All cleaned up!")
