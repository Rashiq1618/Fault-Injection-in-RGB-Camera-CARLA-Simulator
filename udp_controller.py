import socket
import json
import time

# UDP configs
UDP_IP = "0.0.0.0"
UDP_PORT_RECEIVE = 9000    
UDP_PORT_SEND = 9001       
CARLA_IP = ""  
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind((UDP_IP, UDP_PORT_RECEIVE))
recv_sock.setblocking(True)

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

braking_active = False
brake_sent_time = None
BRAKE_RANGE = 6.0
SLOWDOWN_RANGE = 10.0
RESUME_DELAY = 0  

print("Controller listening on port", UDP_PORT_RECEIVE)

while True:
    try:
        msg, addr = recv_sock.recvfrom(65536)
        try:
            data = json.loads(msg.decode())
        except Exception as e:
            print("Bad JSON:", e)
            continue

        speed = data.get("speed")
        detected = data.get("pedestrian_detected")
        distance = data.get("distance")
        exposure = data.get("camera", {}).get("exposure_compensation", 0)

        weather = data.get("weather", {})
        fog_density = weather.get("fog_density")
        fog_distance = weather.get("fog_distance")
        precipitation = weather.get("precipitation")

        
        print(f"Speed: {speed} km/h | Pedestrian: {detected} | Dist. to Pedestrian: {distance} m | "
              f"Fog: {fog_density} | Exposure: {exposure} | FogDist: {fog_distance} | Rain: {precipitation}")

        # Decision logic
        
        if detected and distance is not None:
            
            if distance <= BRAKE_RANGE and not braking_active:
                payload = json.dumps({"cmd": "brake"}).encode()
                send_sock.sendto(payload, (CARLA_IP, UDP_PORT_SEND))
                print("[Controller] Brake command sent.")
                braking_active = True
                brake_sent_time = time.time()

            
            elif BRAKE_RANGE < distance <= SLOWDOWN_RANGE and not braking_active:
                payload = json.dumps({"cmd": "slowdown", "distance": distance}).encode()
                send_sock.sendto(payload, (CARLA_IP, UDP_PORT_SEND))
                print("[Controller] Slowdown command sent.")

        
        if not detected and braking_active:
            if brake_sent_time and (time.time() - brake_sent_time >= RESUME_DELAY):
                payload = json.dumps({"cmd": "resume"}).encode()
                send_sock.sendto(payload, (CARLA_IP, UDP_PORT_SEND))
                print("[Controller] Resume command sent (after brake).")
                braking_active = False
                brake_sent_time = None

    except Exception as e:
        print("Controller error:", e)
        time.sleep(0.1)
        continue
