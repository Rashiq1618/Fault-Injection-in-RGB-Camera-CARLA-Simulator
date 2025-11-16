import carla
import pygame
import numpy as np
import random
import time
import math
import socket
import json
import cv2
import sys
from threading import Thread


pedestrian_collision = False   

# UDP configs
UDP_IP = ""   
UDP_PORT_SEND = 9000      
UDP_PORT_RECEIVE = 9001   

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("", UDP_PORT_RECEIVE))
recv_sock.setblocking(False)

# CARLA setup
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_lib = world.get_blueprint_library()

tm = client.get_trafficmanager()
traffic_manager = client.get_trafficmanager(8000)
tm.set_synchronous_mode(False)
tm.set_global_distance_to_leading_vehicle(2.5)


# Weather setup
weather = world.get_weather()
weather.sun_altitude_angle = -15.0
weather.sun_azimuth_angle = 180.0
weather.cloudiness = 90.0
weather.precipitation = 0.0
weather.precipitation_deposits = 0.0
weather.fog_density = 90.0
weather.fog_distance = 5.0
weather.fog_falloff = 1.5
world.set_weather(weather)


#collision callback
def collision_callback(event):
    global pedestrian_collision
    other = event.other_actor

    if other is None:
        return

    # Checking if the collision was with a pedestrian or not
    if "walker.pedestrian" in other.type_id:
        print("\n ⚠️[COLLISION] Pedestrian collision detected! ")
        pedestrian_collision = True



# Vehicle spawn
def spawn_vehicle():
    for v in world.get_actors().filter('vehicle.*'):
        try:
            v.destroy()
        except Exception:
            pass

    try:
        veh_bp = blueprint_lib.find('vehicle.tesla.model3')
        if 'color' in [a.id for a in veh_bp]:
            veh_bp.set_attribute('color', '0,0,255')
    except Exception:
        veh_bp = random.choice(blueprint_lib.filter('vehicle.*'))

    spawn_pt = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(veh_bp, spawn_pt)
    vehicle.set_autopilot(True, tm.get_port())

    tm.ignore_lights_percentage(vehicle, 100.0)
    tm.ignore_walkers_percentage(vehicle, 100.0)


    return vehicle

#Spawn ego vehicle
vehicle = spawn_vehicle()

# collision sensor

col_bp = blueprint_lib.find('sensor.other.collision')
collision_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=vehicle)
collision_sensor.listen(lambda event: collision_callback(event))


# Camera setup
cam_bp = blueprint_lib.find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', '1280')
cam_bp.set_attribute('image_size_y', '720')
cam_bp.set_attribute('fov', '90')
cam_bp.set_attribute('exposure_mode', 'manual')

try:
    cam_bp.set_attribute('slope', '0.95')
    cam_bp.set_attribute('toe', '0.7')
    cam_bp.set_attribute('shoulder', '0.2')
    cam_bp.set_attribute('temp', '5400.0')
    cam_bp.set_attribute('enable_postprocess_effects', 'True')
    cam_bp.set_attribute('blur_radius', '2.5')
    cam_bp.set_attribute('motion_blur_intensity', '0.4')
except Exception:
    pass

exposure_levels = [-3.0, -2.85, -2.75, -2.65, -2.50, -2.0]
current_exposure = exposure_levels[0]

cam_transform = carla.Transform(carla.Location(x=1.6, y=0.0, z=1.4))
camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)

pygame.init()
screen = pygame.display.set_mode((1280, 720))
pygame.display.set_caption("Front Camera View - Extreme Darkness + Fog + Flicker")
surface = None

def process_img(image):
    global surface
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = np.reshape(arr, (image.height, image.width, 4))
    arr = arr[:, :, :3][:, :, ::-1]
    surface = pygame.surfarray.make_surface(arr.swapaxes(0, 1))

camera.listen(lambda image: process_img(image))

# Light flicker
try:
    light_manager = world.get_lightmanager()
    light_manager.set_day_night_cycle(False)
    street_lights = light_manager.get_all_lights(carla.LightGroup.Street)
except Exception:
    light_manager = None
    street_lights = []

flicker_indices = []
lights_on = []
if len(street_lights) > 0:
    flicker_indices = random.sample(range(len(street_lights)), k=int(len(street_lights) * 0.30))
    lights_on = [True] * len(street_lights)

# Pedestrian detection
MAX_DETECTION_RANGE = 10.0
BRAKE_RANGE = 6.0

def detect_pedestrian(ego_transform, walkers, current_exposure_val):
    if current_exposure_val < -2.50:
        return False, None

    closest_distance = None
    for walker in walkers:
        loc = walker.get_location()
        direction = loc - ego_transform.location
        distance = direction.length()
        if distance > MAX_DETECTION_RANGE:
            continue

        forward = ego_transform.get_forward_vector()
        try:
            direction_unit = direction.make_unit_vector()
        except Exception:
            continue

        dot = forward.x * direction_unit.x + forward.y * direction_unit.y + forward.z * direction_unit.z
        if dot > 0.7:
            if closest_distance is None or distance < closest_distance:
                closest_distance = distance
    return (closest_distance is not None), closest_distance



# Light manager
def apply_light_states():
    if light_manager is None or len(street_lights) == 0:
        return
    try:
        light_manager.set_active(street_lights, lights_on)
    except Exception:
        pass



# UDP command listener
def udp_command_listener():
    while True:
        try:
            msg, _ = recv_sock.recvfrom(1024)
            cmd = json.loads(msg.decode())
            if not isinstance(cmd, dict):
                continue
            command = cmd.get("cmd")
            dist = cmd.get("distance", None)

            if command == "brake":
                print("[UDP] Brake command received.")
                try:
                    vehicle.set_autopilot(False, tm.get_port())
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                except Exception:
                    pass

            elif command == "slowdown" and dist is not None:
                throttle = 0.05 + (BRAKE_RANGE) * (0.35 - 0.05) / (MAX_DETECTION_RANGE - BRAKE_RANGE)
                throttle = max(0.05, min(0.35, throttle))
                print(f"[UDP] Slowdown: throttle={throttle:.2f} dist={dist:.1f}")
                try:
                    vehicle.set_autopilot(True, tm.get_port())
                except Exception:
                    pass

            elif command == "resume":
                print("[UDP] Resume command received.")
                try:
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
                    time.sleep(0.1)
                    vehicle.set_autopilot(True, tm.get_port())
                except Exception:
                    pass

        except BlockingIOError:
            time.sleep(0.01)
        except Exception as e:
            print("[UDP listener] Error:", e)
            time.sleep(0.1)

udp_thread = Thread(target=udp_command_listener, daemon=True)
udp_thread.start()


#Loop
clock = pygame.time.Clock()
flicker_timer = 0.0
flicker_interval = 0.1

# >>> ADDED FOR FOG DISTANCE FLICKER
fogdist_timer = 0.0
fogdist_interval = 2.0

base_fog = 90.0
fog_amplitude = 5.0
t = 0.0
timer = 0.0
exposure_idx = 0

print("\n Metrics of weather and camera")
print("Fog(%) | Exposure | FogDist | Clouds | Rain")

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    print("Respawning ego vehicle")
                    try:
                        if vehicle is not None:
                            try:
                                vehicle.destroy()
                            except Exception:
                                pass
                        vehicle = spawn_vehicle()
                        try:
                            camera.stop()
                            camera.destroy()
                        except Exception:
                            pass
                        camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
                        camera.listen(lambda image: process_img(image))

                        #recreation of collision sensor on new vehilce
                        collision_sensor.destroy()
                        collision_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=vehicle)
                        collision_sensor.listen(lambda event: collision_callback(event))

                    except Exception as e:
                        print("Respawn error:", e)

        dt = clock.get_time() / 1000.0

        fog_values = [5, 8, 10, 12, 15]
        last_fog = weather.fog_distance

        fogdist_timer += dt
        if fogdist_timer > fogdist_interval:
            possible = [v for v in fog_values if v != last_fog]
            weather.fog_distance = random.choice(possible)
            last_fog = weather.fog_distance
            fogdist_timer = 0.0




        world.set_weather(weather)

        timer += dt
        if timer > 3.0:
            exposure_idx = random.randint(0, len(exposure_levels) - 1)
            current_exposure = exposure_levels[exposure_idx]
            try:
                camera.set_attribute('exposure_compensation', str(current_exposure))
            except Exception:
                pass
            timer = 0.0

        flicker_timer += dt
        if flicker_timer > flicker_interval:
            for i in flicker_indices:
                lights_on[i] = not lights_on[i]
            apply_light_states()
            flicker_timer = 0.0

        if surface is not None:
            screen.blit(surface, (0, 0))
        pygame.display.flip()
        clock.tick(30)

        vel = vehicle.get_velocity()
        speed = 3.6 * (vel.x**2 + vel.y**2 + vel.z**2)**0.5
        ego_transform = vehicle.get_transform()
        walkers = world.get_actors().filter('walker.pedestrian.*')
        detected, distance = detect_pedestrian(ego_transform, walkers, current_exposure)

        #telemetry
        payload = {
            "speed": round(speed, 2),
            "pedestrian_detected": bool(detected),
            "distance": round(distance, 2) if distance is not None else None,
            "pedestrian_collision": pedestrian_collision,
            "weather": {
                "fog_density": round(weather.fog_density, 2),
                "fog_distance": round(weather.fog_distance, 2),
                "precipitation": round(weather.precipitation, 2)
            },
            "camera": {
                "image_size_x": int(cam_bp.get_attribute("image_size_x").as_int() if cam_bp.has_attribute("image_size_x") else 1280),
                "image_size_y": int(cam_bp.get_attribute("image_size_y").as_int() if cam_bp.has_attribute("image_size_y") else 720),
                "fov": float(cam_bp.get_attribute("fov").as_float() if cam_bp.has_attribute("fov") else 90.0),
                "exposure_compensation": float(current_exposure)
            },
            "fog_details": {
                "base_fog": base_fog,
                "fog_amplitude": fog_amplitude
            }
        }

        try:
            send_sock.sendto(json.dumps(payload).encode(), (UDP_IP, UDP_PORT_SEND))
        except Exception as e:
            print("[UDP send] error:", e)

        pedestrian_collision = False

        print(f"{weather.fog_density:6.1f} | {current_exposure:8.2f} | {weather.fog_distance:7.1f} | "
              f"{weather.cloudiness:7.1f} | {weather.precipitation:4.1f}", end='\r')

except KeyboardInterrupt:
    print("\nStopping simulation.")

finally:
    try:
        camera.stop()
        camera.destroy()
    except Exception:
        pass
    try:
        if 'collision_sensor' in locals():
            collision_sensor.destroy()
    except:
        pass
    try:
        if 'vehicle' in locals() and vehicle is not None:
            vehicle.destroy()
    except:
        pass
    pygame.quit()
    print("\nExited.")
