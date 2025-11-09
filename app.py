from flask import Flask, render_template, jsonify, request
import datetime
import os
import requests
from pymavlink import mavutil
import threading
import time
import json

app = Flask(__name__)

UAVS = {}
MAVLINK_CONNECTIONS = {}
uavs_lock = threading.Lock()
myip = 127.0.0.1 #Замените на ваш IP TailScale Repka Pi
def connect_to_uav(port):
    """Подключение к БВС на указанном порту"""
    try:
        connection_string = f"udp:{myip}:{port}"
        master = mavutil.mavlink_connection(connection_string)
        
        print(f"Пытаемся подключиться к порту {port}...")
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        
        if msg:
            uav_id = f"uav_{port}"
            with uavs_lock:
                UAVS[uav_id] = {
                    "id": uav_id,
                    "name": f"БВС-{port-219}",
                    "lat": 0.0,
                    "lon": 0.0,
                    "alt": 0.0,
                    "heading": 0,
                    "ground_speed": 0,
                    "port": port,
                    "status": "online",
                    "mission": [],
                    "last_heartbeat": datetime.datetime.utcnow().isoformat(),
                    "connected": True,
                    "gps_fix": 0,
                    "satellites": 0
                }
                MAVLINK_CONNECTIONS[uav_id] = master
            print(f"Успешно подключились к БВС на порту {port}")
            return True
        else:
            print(f"Нет heartbeat на порту {port}")
            
    except Exception as e:
        print(f"Ошибка подключения к порту {port}: {e}")
    
    return False

def listen_to_uav(uav_id):
    """Прослушивание данных от БВС"""
    while True:
        try:
            with uavs_lock:
                if uav_id not in UAVS or not UAVS[uav_id].get("connected", True):
                    break
                master = MAVLINK_CONNECTIONS.get(uav_id)
            
            if not master:
                break
                
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg:
                if msg.get_type() == 'HEARTBEAT':
                    with uavs_lock:
                        if uav_id in UAVS:
                            UAVS[uav_id]["last_heartbeat"] = datetime.datetime.utcnow().isoformat()
                            UAVS[uav_id]["status"] = "online"
                
                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    with uavs_lock:
                        if uav_id in UAVS:
                            UAVS[uav_id]["lat"] = msg.lat / 1e7
                            UAVS[uav_id]["lon"] = msg.lon / 1e7
                            UAVS[uav_id]["alt"] = msg.relative_alt / 1000.0
                            UAVS[uav_id]["heading"] = msg.hdg / 100
                
                elif msg.get_type() == 'VFR_HUD':
                    with uavs_lock:
                        if uav_id in UAVS:
                            UAVS[uav_id]["ground_speed"] = msg.groundspeed
                
                elif msg.get_type() == 'GPS_RAW_INT':
                    with uavs_lock:
                        if uav_id in UAVS:
                            UAVS[uav_id]["gps_fix"] = msg.fix_type
                            UAVS[uav_id]["satellites"] = msg.satellites_visible
            
        except Exception as e:
            print(f"Ошибка при прослушивании {uav_id}: {e}")
            with uavs_lock:
                if uav_id in UAVS:
                    UAVS[uav_id]["status"] = "offline"
            time.sleep(1)

def discover_uavs():
    ports = range(14550, 14551)
    
    print(f"Сканирование портов {list(ports)}...")
    
    for port in ports:
        uav_id = f"uav_{port}"
        with uavs_lock:
            if uav_id in UAVS and UAVS[uav_id].get("connected", False):
                continue
        
        if connect_to_uav(port):
            thread = threading.Thread(target=listen_to_uav, args=(uav_id,), daemon=True)
            thread.start()

def cleanup_disconnected_uavs():
    """Очистка отключенных БВС"""
    with uavs_lock:
        disconnected_uavs = []
        for uav_id, uav in UAVS.items():
            if not uav.get("connected", False):
                disconnected_uavs.append(uav_id)
        
        for uav_id in disconnected_uavs:
            if uav_id in MAVLINK_CONNECTIONS:
                try:
                    MAVLINK_CONNECTIONS[uav_id].close()
                except:
                    pass
                del MAVLINK_CONNECTIONS[uav_id]
            del UAVS[uav_id]
            print(f"Удален отключенный БВС: {uav_id}")

def check_heartbeats():
    """Проверка активности БВС"""
    while True:
        current_time = datetime.datetime.utcnow()
        with uavs_lock:
            for uav_id, uav in list(UAVS.items()):
                if uav.get("last_heartbeat"):
                    last_heartbeat = datetime.datetime.fromisoformat(uav["last_heartbeat"])
                    if (current_time - last_heartbeat).total_seconds() > 10:
                        uav["status"] = "offline"
                        uav["connected"] = False
        time.sleep(5)

print("Запуск обнаружения БВС на портах 220-225...")
discover_thread = threading.Thread(target=discover_uavs, daemon=True)
discover_thread.start()

heartbeat_thread = threading.Thread(target=check_heartbeats, daemon=True)
heartbeat_thread.start()

def periodic_cleanup():
    while True:
        time.sleep(30)
        cleanup_disconnected_uavs()
        discover_uavs()

cleanup_thread = threading.Thread(target=periodic_cleanup, daemon=True)
cleanup_thread.start()

def get_serializable_uavs():
    """Возвращает сериализуемую версию данных БВС"""
    with uavs_lock:
        uavs_list = []
        for uav_id, uav_data in UAVS.items():
            serializable_uav = uav_data.copy()
            uavs_list.append(serializable_uav)
        
        uavs_list.sort(key=lambda x: x["port"])
        return uavs_list

@app.route("/")
def index():
    uavs_list = get_serializable_uavs()
    first_mission = uavs_list[0]["mission"] if uavs_list else []
    
    return render_template(
        "index.html",
        uavs=uavs_list,
        first_mission=first_mission,
    )

@app.route("/uavs")
def get_uavs():
    uavs_list = get_serializable_uavs()
    return jsonify(uavs_list)

@app.route("/uavs/<uav_id>/mission", methods=["GET", "POST"])
def mission(uav_id):
    with uavs_lock:
        if uav_id not in UAVS:
            return jsonify({"error": "not found"}), 404

    if request.method == "GET":
        with uavs_lock:
            return jsonify({"items": UAVS[uav_id]["mission"]})

    data = request.get_json(silent=True) or {}
    with uavs_lock:
        UAVS[uav_id]["mission"] = data.get("items", [])
    return jsonify({"status": "ok"})

@app.route("/weather")
def weather():
    lat = request.args.get("lat", type=float)
    lon = request.args.get("lon", type=float)
    if lat is None or lon is None:
        return jsonify({"error": "lat/lon required"}), 400

    api_key = os.getenv("OPENWEATHER_API_KEY")
    if not api_key:
        return jsonify({
            "name": "0",
            "temp": 0,
            "description": "нет API ключа",
            "wind_speed": 0
        })

    try:
        url = (
            "https://api.openweathermap.org/data/2.5/weather"
            f"?lat={lat}&lon={lon}&appid={api_key}&units=metric&lang=ru"
        )
        r = requests.get(url, timeout=5)
        data = r.json()
        return jsonify({
            "name": data.get("name"),
            "temp": data["main"]["temp"],
            "description": data["weather"][0]["description"],
            "wind_speed": data["wind"]["speed"],
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/refresh_uavs")
def refresh_uavs():
    """Принудительное обновление списка БВС"""
    discover_uavs()
    uavs_list = get_serializable_uavs()
    return jsonify({
        "active_uavs": len(uavs_list), 
        "uavs": [uav["name"] for uav in uavs_list]
    })

@app.route("/uavs/<uav_id>/disconnect", methods=["POST"])
def disconnect_uav(uav_id):
    """Принудительное отключение БВС"""
    with uavs_lock:
        if uav_id in UAVS:
            UAVS[uav_id]["connected"] = False
            UAVS[uav_id]["status"] = "offline"
            # Закрываем соединение
            if uav_id in MAVLINK_CONNECTIONS:
                try:
                    MAVLINK_CONNECTIONS[uav_id].close()
                except:
                    pass
                del MAVLINK_CONNECTIONS[uav_id]
            return jsonify({"status": "disconnected"})
    return jsonify({"error": "not found"}), 404

if __name__ == "__main__":
    app.run(host="localhost", port=5555, debug=True)
