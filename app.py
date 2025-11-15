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

# IP Repka Pi по Tailscale (адрес внутри Headscale-сети)
REPKA_IP = "100.64.0.1"


def connect_to_uav(port: int) -> bool:
    """
    Подключение к БВС через MAVProxy по UDP.
    Предполагается, что на Repka Pi запущен MAVProxy с форвардингом на этот порт.
    """
    try:
        connection_string = f"udpout:{REPKA_IP}:{port}"

        master = mavutil.mavlink_connection(
            connection_string,
            source_system=250,   # ID нашей GCS
            source_component=1   # компонент GCS
        )

        print(f"[CONNECT] Пытаемся подключиться к {REPKA_IP}:{port} (udpout)...")

        # Шлём heartbeat, чтобы автопилот понимал, что есть GCS
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )

        # Ждём heartbeat от борта
        msg = master.wait_heartbeat(timeout=5)

        if msg:
            uav_id = f"uav_{port}"
            with uavs_lock:
                UAVS[uav_id] = {
                    "id": uav_id,
                    "name": f"БВС-{port - 219}",
                    "lat": 0.0,
                    "lon": 0.0,
                    "alt": 0.0,
                    "heading": 0,
                    "ground_speed": 0.0,
                    "port": port,
                    "status": "online",
                    "mission": [],
                    "last_heartbeat": datetime.datetime.utcnow().isoformat(),
                    "connected": True,
                    "gps_fix": 0,
                    "satellites": 0,
                    # Питание
                    "battery_percent": None,   # в %
                    "battery_voltage": None,   # в В
                }
                MAVLINK_CONNECTIONS[uav_id] = master

            print(f"[CONNECT] Успешно получили HEARTBEAT от БВС на порту {port}")
            return True
        else:
            print(f"[CONNECT] Нет HEARTBEAT от БВС на порту {port}")

    except Exception as e:
        print(f"[CONNECT] Ошибка подключения к порту {port}: {e}")

    return False


def listen_to_uav(uav_id: str) -> None:
    """Фоновый поток: приём MAVLink-сообщений и обновление состояния БВС."""
    while True:
        try:
            with uavs_lock:
                if uav_id not in UAVS or not UAVS[uav_id].get("connected", True):
                    break
                master = MAVLINK_CONNECTIONS.get(uav_id)

            if not master:
                break

            msg = master.recv_match(blocking=True, timeout=1)

            if not msg:
                continue

            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                with uavs_lock:
                    if uav_id in UAVS:
                        UAVS[uav_id]["last_heartbeat"] = datetime.datetime.utcnow().isoformat()
                        UAVS[uav_id]["status"] = "online"

            elif msg_type == 'GLOBAL_POSITION_INT':
                with uavs_lock:
                    if uav_id in UAVS:
                        UAVS[uav_id]["lat"] = msg.lat / 1e7
                        UAVS[uav_id]["lon"] = msg.lon / 1e7
                        UAVS[uav_id]["alt"] = msg.relative_alt / 1000.0
                        UAVS[uav_id]["heading"] = msg.hdg / 100

            elif msg_type == 'VFR_HUD':
                with uavs_lock:
                    if uav_id in UAVS:
                        UAVS[uav_id]["ground_speed"] = float(msg.groundspeed)

            elif msg_type == 'GPS_RAW_INT':
                with uavs_lock:
                    if uav_id in UAVS:
                        UAVS[uav_id]["gps_fix"] = msg.fix_type
                        UAVS[uav_id]["satellites"] = msg.satellites_visible

            elif msg_type == 'SYS_STATUS':
                # Статус системы, в т.ч. батарея
                # msg.voltage_battery — в мВ, msg.battery_remaining — в %
                with uavs_lock:
                    if uav_id in UAVS:
                        percent = msg.battery_remaining
                        voltage = msg.voltage_battery
                        if percent is not None and percent >= 0:
                            UAVS[uav_id]["battery_percent"] = int(percent)
                        if voltage is not None and voltage > 0:
                            UAVS[uav_id]["battery_voltage"] = round(voltage / 1000.0, 2)

        except Exception as e:
            print(f"[LISTEN] Ошибка при прослушивании {uav_id}: {e}")
            with uavs_lock:
                if uav_id in UAVS:
                    UAVS[uav_id]["status"] = "offline"
                    UAVS[uav_id]["connected"] = False
            time.sleep(1)


def discover_uavs() -> None:
    """Обнаружение БВС на фиксированном диапазоне портов."""
    ports = range(14550, 14551)  # пока один порт, можно расширить

    print(f"[DISCOVER] Сканирование портов {list(ports)}...")

    for port in ports:
        uav_id = f"uav_{port}"
        with uavs_lock:
            if uav_id in UAVS and UAVS[uav_id].get("connected", False):
                continue

        if connect_to_uav(port):
            thread = threading.Thread(target=listen_to_uav, args=(uav_id,), daemon=True)
            thread.start()


def cleanup_disconnected_uavs() -> None:
    """Очистка отключенных БВС и закрытие соединений."""
    with uavs_lock:
        disconnected_uavs = [uav_id for uav_id, uav in UAVS.items()
                             if not uav.get("connected", False)]

        for uav_id in disconnected_uavs:
            if uav_id in MAVLINK_CONNECTIONS:
                try:
                    MAVLINK_CONNECTIONS[uav_id].close()
                except Exception:
                    pass
                del MAVLINK_CONNECTIONS[uav_id]
            del UAVS[uav_id]
            print(f"[CLEANUP] Удален отключенный БВС: {uav_id}")


def check_heartbeats() -> None:
    """Периодическая проверка активности БВС по времени последнего HEARTBEAT."""
    while True:
        current_time = datetime.datetime.utcnow()
        with uavs_lock:
            for uav_id, uav in list(UAVS.items()):
                last = uav.get("last_heartbeat")
                if last:
                    last_dt = datetime.datetime.fromisoformat(last)
                    if (current_time - last_dt).total_seconds() > 10:
                        uav["status"] = "offline"
                        uav["connected"] = False
        time.sleep(5)


def periodic_cleanup() -> None:
    """Фоновый поток: периодическая очистка и повторное сканирование."""
    while True:
        time.sleep(30)
        cleanup_disconnected_uavs()
        discover_uavs()


def get_serializable_uavs():
    """Сериализация структуры UAVS в список объектов, готовых к JSON."""
    with uavs_lock:
        uavs_list = []
        for _, uav_data in UAVS.items():
            serializable_uav = dict(uav_data)
            uavs_list.append(serializable_uav)

        uavs_list.sort(key=lambda x: x["port"])
        return uavs_list


def parse_qgc_plan(plan_data):
    """
    Разбор .plan (QGroundControl) в список waypoints и удобный формат для UI.
    Возвращает:
      items: [{seq, command, params, lat, lon, alt}, ...]
      waypoints: [[lat, lon], ...] для отрисовки на карте
    """
    mission = plan_data.get("mission", {})
    items_raw = mission.get("items", [])

    items = []
    waypoints = []

    for item in items_raw:
        if item.get("type") != "SimpleItem":
            continue

        cmd = item.get("command")
        params = item.get("params", [])

        lat = None
        lon = None
        alt = None

        if len(params) >= 7 and params[4] is not None and params[5] is not None:
            lat = float(params[4])
            lon = float(params[5])
            alt = float(params[6]) if params[6] is not None else float(item.get("Altitude", 0))
            waypoints.append([lat, lon])

        items.append({
            "seq": int(item.get("doJumpId", len(items) + 1)),
            "command": int(cmd) if cmd is not None else 0,
            "params": params,
            "lat": lat,
            "lon": lon,
            "alt": alt,
        })

    return items, waypoints


def send_mission_to_uav(uav_id: str):
    """
    Отправка текущей миссии из UAVS[uav_id]["mission"] на борт через MAVLink.
    Упрощённая реализация протокола MISSION_* (без полной обработки MISSION_REQUEST).
    """
    with uavs_lock:
        uav = UAVS.get(uav_id)
        master = MAVLINK_CONNECTIONS.get(uav_id)
        mission = uav.get("mission", []) if uav else []

    if not master:
        raise RuntimeError("Нет подключения к БВС")

    if not mission:
        raise RuntimeError("Миссия пуста")

    # Очистка старой миссии
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(0.5)

    # Сообщаем количество точек
    master.mav.mission_count_send(master.target_system, master.target_component, len(mission))

    # Упрощённая отправка точек
    for i, wp in enumerate(mission):
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        cmd = int(wp.get("command", 16))

        params = wp.get("params", [])
        params = list(params) + [0.0] * (7 - len(params))
        p1, p2, p3, p4, x, y, z = params

        lat = wp.get("lat")
        lon = wp.get("lon")
        alt = wp.get("alt") if wp.get("alt") is not None else z

        if lat is not None:
            x = int(lat * 1e7)
        if lon is not None:
            y = int(lon * 1e7)
        if alt is not None:
            z = float(alt)

        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            i,
            frame,
            cmd,
            0,  # current
            1,  # auto-continue
            float(p1),
            float(p2),
            float(p3),
            float(p4),
            int(x),
            int(y),
            float(z)
        )

        # Небольшая пауза
        time.sleep(0.05)

    # Запуск миссии (MAV_CMD_MISSION_START)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0, 0, 0, 0, 0, 0, 0
    )


def stop_mission_on_uav(uav_id: str):
    """
    Остановка миссии: перевод в LOITER (если доступен) или навсегда зависнуть.
    """
    with uavs_lock:
        master = MAVLINK_CONNECTIONS.get(uav_id)

    if not master:
        raise RuntimeError("Нет подключения к БВС")

    try:
        mode_mapping = master.mode_mapping()
        if "LOITER" in mode_mapping:
            master.set_mode("LOITER")
        else:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
    except Exception as e:
        raise RuntimeError(f"Ошибка при смене режима: {e}")


# --- Старт фоновых потоков ---
print("[INIT] Запуск обнаружения БВС на портах 14550...")
discover_thread = threading.Thread(target=discover_uavs, daemon=True)
discover_thread.start()

heartbeat_thread = threading.Thread(target=check_heartbeats, daemon=True)
heartbeat_thread.start()

cleanup_thread = threading.Thread(target=periodic_cleanup, daemon=True)
cleanup_thread.start()


# --- ROUTES ---


@app.route("/")
def index():
    uavs_list = get_serializable_uavs()
    first_mission = uavs_list[0]["mission"] if uavs_list else []
    return render_template("index.html", uavs=uavs_list, first_mission=first_mission)


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


@app.route("/uavs/<uav_id>/mission/upload", methods=["POST"])
def upload_mission(uav_id):
    """Загрузка .plan или JSON-миссии с фронта."""
    with uavs_lock:
        if uav_id not in UAVS:
            return jsonify({"error": "UAV not found"}), 404

    if "file" not in request.files:
        return jsonify({"error": "No file uploaded"}), 400

    file = request.files["file"]
    try:
        plan_data = json.load(file)
    except Exception as e:
        return jsonify({"error": f"Failed to parse JSON: {e}"}), 400

    items, waypoints = parse_qgc_plan(plan_data)

    with uavs_lock:
        UAVS[uav_id]["mission"] = items

    return jsonify({
        "status": "ok",
        "items": items,
        "waypoints": waypoints
    })


@app.route("/uavs/<uav_id>/mission/start", methods=["POST"])
def start_mission(uav_id):
    try:
        send_mission_to_uav(uav_id)
        return jsonify({"status": "started"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/uavs/<uav_id>/mission/stop", methods=["POST"])
def stop_mission(uav_id):
    try:
        stop_mission_on_uav(uav_id)
        return jsonify({"status": "stopped"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/uavs/<uav_id>/disconnect", methods=["POST"])
def disconnect_uav(uav_id):
    """Принудительное отключение БВС."""
    with uavs_lock:
        if uav_id in UAVS:
            UAVS[uav_id]["connected"] = False
            UAVS[uav_id]["status"] = "offline"
            if uav_id in MAVLINK_CONNECTIONS:
                try:
                    MAVLINK_CONNECTIONS[uav_id].close()
                except Exception:
                    pass
                del MAVLINK_CONNECTIONS[uav_id]
            return jsonify({"status": "disconnected"})
    return jsonify({"error": "not found"}), 404


@app.route("/refresh_uavs")
def refresh_uavs():
    """Принудительное обновление списка БВС."""
    discover_uavs()
    uavs_list = get_serializable_uavs()
    return jsonify({
        "active_uavs": len(uavs_list),
        "uavs": [uav["name"] for uav in uavs_list]
    })

@app.route("/weather")
def weather():
    """
    Бесплатная погода: Open-Meteo + обратное геокодирование (Nominatim OSM),
    чтобы "name" был похож на город/регион по координатам.
    """
    lat = request.args.get("lat", type=float)
    lon = request.args.get("lon", type=float)
    if lat is None or lon is None:
        return jsonify({"error": "lat/lon required"}), 400

    try:
        # 1) Погода без ключа
        w_url = (
            "https://api.open-meteo.com/v1/forecast"
            f"?latitude={lat}&longitude={lon}&current_weather=true"
        )
        w = requests.get(w_url, timeout=5)
        w_data = w.json()
        current = w_data.get("current_weather", {}) or {}

        # 2) Название места
        location_name = "Неизвестное место"
        try:
            loc = requests.get(
                "https://nominatim.openstreetmap.org/reverse"
                f"?format=json&lat={lat}&lon={lon}&zoom=10&addressdetails=1",
                headers={"User-Agent": "RepkaPi-Weather/1.0"},
                timeout=5,
            ).json()
            addr = loc.get("address", {}) or {}
            location_name = (
                addr.get("city") or addr.get("town") or addr.get("village")
                or addr.get("state") or "Неизвестное место"
            )
        except Exception:
            pass

        return jsonify({
            "name": location_name,
            "temp": current.get("temperature", 0),
            "description": f"ветер {current.get('windspeed', 0)} м/с",
            "wind_speed": current.get("windspeed", 0),
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == "__main__":
    # Для боевого режима лучше использовать gunicorn/uwsgi
    app.run(host="0.0.0.0", port=5555, debug=True)
