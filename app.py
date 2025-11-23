from flask import Flask, render_template, jsonify, request
import datetime
import requests
from pymavlink import mavutil
import threading
import time
import json
from typing import Optional

app = Flask(__name__)

UAVS = {}
MAVLINK_CONNECTIONS = {}
uavs_lock = threading.Lock()

# IP Repka Pi по Tailscale (адрес внутри Headscale-сети)
REPKA_IP = "localhost"


def connect_to_uav(port: int) -> bool:
    """
    Подключение к БВС через MAVProxy по UDP.
    Предполагается, что на Repka Pi запущен MAVProxy с форвардингом на этот порт.
    """
    try:
        connection_string = f"udp:{REPKA_IP}:{port}"

        master = mavutil.mavlink_connection(
            connection_string,
            source_system=250,   # ID нашей GCS
            source_component=1   # компонент GCS
        )

        print(f"[CONNECT] Пытаемся подключиться к {REPKA_IP}:{port} (udp)...")

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
                    # для миссий
                    "plan_raw": None,          # полный .plan
                    "telemetry_enabled": True, # флаг, если захочешь паузить телеметрию

                    # состояние миссии
                    "mission_status": "idle",        # idle|starting|running|completed|stopped|error
                    "mission_phase": None,           # uploading|in_progress|completed|error|stopped|timeout|...
                    "mission_total": 0,              # всего элементов в миссии (включая home)
                    "mission_current_seq": -1,       # текущий индекс из MISSION_CURRENT
                    "mission_progress": 0.0,         # 0..1
                    "last_mission_update": None,

                    # блокировка при загрузке/старте миссии, чтобы телеметрия не читала сокет
                    "mission_comm_lock": False,
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
                telemetry_enabled = UAVS[uav_id].get("telemetry_enabled", True)
                mission_lock = UAVS[uav_id].get("mission_comm_lock", False)

            if not master:
                break

            # Если идёт загрузка/арминг миссии — не читаем из сокета, чтобы не мешать протоколу
            if not telemetry_enabled or mission_lock:
                time.sleep(0.05)
                continue

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
                with uavs_lock:
                    if uav_id in UAVS:
                        percent = msg.battery_remaining
                        voltage = msg.voltage_battery
                        if percent is not None and percent >= 0:
                            UAVS[uav_id]["battery_percent"] = int(percent)
                        if voltage is not None and voltage > 0:
                            UAVS[uav_id]["battery_voltage"] = round(voltage / 1000.0, 2)

            elif msg_type == 'MISSION_CURRENT':
                current_wp = msg.seq
                with uavs_lock:
                    if uav_id not in UAVS:
                        continue
                    uav = UAVS[uav_id]
                    status = uav.get("mission_status", "idle")

                    # Если миссия остановлена — игнорируем дальнейшие MISSION_CURRENT
                    if status == "stopped":
                        # Можно раскомментировать, если хочешь видеть это в логах:
                        # print(f"[MISSION] {uav_id} stopped, ignoring MISSION_CURRENT seq={current_wp}")
                        continue

                    total = uav.get("mission_total") or len(uav.get("mission", [])) or 0
                    progress = 0.0
                    if total > 0:
                        progress = min(1.0, max(0.0, (current_wp + 1) / total))
                    uav["mission_current_seq"] = int(current_wp)
                    uav["mission_progress"] = progress
                    uav["last_mission_update"] = datetime.datetime.utcnow().isoformat()

                    # лог для отладки
                    print(f"[MISSION] MISSION_CURRENT {uav_id}: seq={current_wp}/{total}")

                    # если дошли до конца — считаем миссию завершённой
                    if total > 0 and current_wp >= total - 1:
                        uav["mission_status"] = "completed"
                        uav["mission_phase"] = "completed"
                        print(f"[MISSION] {uav_id} completed by MISSION_CURRENT")

            elif msg_type == 'STATUSTEXT':
                text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else str(msg.text)
                print(f"[STATUSTEXT] {uav_id}: {text}")
                low = text.lower()
                if "mission complete" in low or "landed" in low:
                    with uavs_lock:
                        if uav_id in UAVS:
                            uav = UAVS[uav_id]
                            # не переходим в completed, если уже stopped — стоп важнее
                            if uav.get("mission_status") != "stopped":
                                uav["mission_status"] = "completed"
                                uav["mission_phase"] = "completed"
                                uav["last_mission_update"] = datetime.datetime.utcnow().isoformat()
                                print(f"[MISSION] {uav_id} completed by STATUSTEXT")

        except Exception as e:
            print(f"[LISTEN] Ошибка при прослушивании {uav_id}: {e}")
            # Только статус, соединение не рвём — пусть поток ещё попробует
            with uavs_lock:
                if uav_id in UAVS:
                    UAVS[uav_id]["status"] = "offline"
            time.sleep(1)


def discover_uavs() -> None:
    """Одноразовое обнаружение БВС на фиксированном диапазоне портов."""
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


def check_heartbeats() -> None:
    """
    Периодическая проверка активности БВС по времени последнего HEARTBEAT.
    ВАЖНО: только меняем status, флаг connected не трогаем и соединение не рвём.
    """
    TIMEOUT_OFFLINE = 60  # секунд без heartbeat, после чего считаем оффлайн

    while True:
        current_time = datetime.datetime.utcnow()
        with uavs_lock:
            for uav_id, uav in list(UAVS.items()):
                last = uav.get("last_heartbeat")
                if last:
                    last_dt = datetime.datetime.fromisoformat(last)
                    if (current_time - last_dt).total_seconds() > TIMEOUT_OFFLINE:
                        uav["status"] = "offline"
        time.sleep(5)


def get_serializable_uavs():
    """Сериализация структуры UAVS в список объектов, готовых к JSON."""
    with uavs_lock:
        uavs_list = []
        for _, uav_data in UAVS.items():
            serializable_uav = dict(uav_data)
            uavs_list.append(serializable_uav)

        uavs_list.sort(key=lambda x: x["port"])
        return uavs_list


def update_mission_state(uav_id: str, **kwargs) -> None:
    """Атомарно обновляет поля состояния миссии в UAVS[uav_id]."""
    with uavs_lock:
        uav = UAVS.get(uav_id)
        if not uav:
            return
        uav.update(kwargs)
        uav["last_mission_update"] = datetime.datetime.utcnow().isoformat()


# ==========================
#   ПАРСИНГ .PLAN ДЛЯ UI
# ==========================

def parse_qgc_plan(plan_data):
    """
    Разбор .plan (QGroundControl) в список waypoints и удобный формат для UI.
    Возвращает:
      items: [{seq, command, frame, autoContinue, params, lat, lon, alt}, ...]
      waypoints: [[lat, lon], ...] для отрисовки маршрута на карте

    Особенности:
    - координаты (0,0) считаем "нет координат";
    - команды LAND/RTL без координат не добавляют точку (0,0);
    - если есть LAND/RTL без координат и задан plannedHomePosition —
      в конце маршрута дорисовываем возврат домой.
    """
    mission = plan_data.get("mission", {})
    items_raw = mission.get("items", [])

    # Домашняя позиция из .plan
    planned_home = mission.get("plannedHomePosition") or []
    home_lat = home_lon = None
    if isinstance(planned_home, (list, tuple)) and len(planned_home) >= 2:
        try:
            home_lat = float(planned_home[0])
            home_lon = float(planned_home[1])
        except Exception:
            home_lat = home_lon = None

    items = []
    waypoints = []
    need_return_home = False  # флаг: был ли LAND/RTL без координат

    eps = 1e-7

    for item in items_raw:
        if item.get("type") != "SimpleItem":
            continue

        cmd = int(item.get("command") or 0)
        frame = int(item.get("frame", mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT))
        auto_continue = bool(item.get("autoContinue", True))

        params = list(item.get("params", []))
        params = params + [0.0] * (7 - len(params))

        lat = None
        lon = None
        alt = None

        # Координаты из params[4], [5], [6]
        if params[4] is not None and params[5] is not None:
            lat_candidate = float(params[4])
            lon_candidate = float(params[5])

            # (0,0) считаем "нет координат"
            if abs(lat_candidate) > eps and abs(lon_candidate) > eps:
                lat = lat_candidate
                lon = lon_candidate
                if params[6] is not None:
                    alt = float(params[6])

        # fallback по Altitude
        if alt is None and item.get("Altitude") is not None:
            try:
                alt = float(item.get("Altitude"))
            except Exception:
                pass

        # Для маршрута на карте:
        # - если есть нормальные координаты — добавляем точку
        # - если это LAND/RTL без координат — помечаем, что нужен возврат домой
        if lat is not None and lon is not None:
            waypoints.append([lat, lon])
        else:
            if cmd in (20, 82):  # LAND или RTL без координат
                need_return_home = True

        items.append({
            "seq": int(item.get("doJumpId", len(items) + 1)),
            "command": cmd,
            "frame": frame,
            "autoContinue": auto_continue,
            "params": params,
            "lat": lat,
            "lon": lon,
            "alt": alt,
        })

    # Если есть домашняя позиция и был LAND/RTL без координат —
    # дорисуем возврат домой в конец маршрута
    if home_lat is not None and home_lon is not None and waypoints:
        if need_return_home:
            last_lat, last_lon = waypoints[-1]
            if abs(last_lat - home_lat) > eps or abs(last_lon - home_lon) > eps:
                waypoints.append([home_lat, home_lon])

    return items, waypoints


# ==========================
#   ЛОГИКА МИССИЙ (упрощённый upload_mission.py)
# ==========================

TIMEOUT_REQUEST = 10.0
TIMEOUT_ACK = 5.0

COORDLESS_COMMANDS = {
    20,   # MAV_CMD_NAV_LAND
    21,   # MAV_CMD_NAV_TAKEOFF
    82,   # MAV_CMD_NAV_RETURN_TO_LAUNCH
    177,  # MAV_CMD_DO_JUMP
}

def safe_float(value) -> float:
    if value is None:
        return 0.0
    try:
        return float(value)
    except (ValueError, TypeError):
        return 0.0

def extract_lat_lon_alt(item: dict):
    params = item.get("params", [])
    if len(params) >= 7:
        lat, lon, alt = params[4:7]
        lat_val = safe_float(lat) if lat is not None else None
        lon_val = safe_float(lon) if lon is not None else None
        alt_val = safe_float(alt) if alt is not None else None
        if lat_val is not None and abs(lat_val) > 1e-7:
            return lat_val, lon_val, alt_val

    for lat_key in ["x", "lat", "latitude"]:
        if lat_key in item and item[lat_key] is not None:
            lat_val = safe_float(item[lat_key])
            lon_val = None
            alt_val = None
            for lon_key in ["y", "lon", "longitude"]:
                if lon_key in item and item[lon_key] is not None:
                    lon_val = safe_float(item[lon_key])
                    break
            for alt_key in ["Altitude", "alt", "z", "altitude"]:
                if alt_key in item and item[alt_key] is not None:
                    alt_val = safe_float(item[alt_key])
                    break
            if lat_val is not None and abs(lat_val) > 1e-7:
                return lat_val, lon_val, alt_val

    return None, None, None

def get_frame_for_item(item: dict, is_home: bool = False) -> int:
    return (mavutil.mavlink.MAV_FRAME_GLOBAL if is_home
            else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)

def build_mission_items_from_plan(plan_mission: dict, include_home: bool = True):
    items = plan_mission.get("items", [])
    mission_items = list(items)

    planned_home = None
    ph = plan_mission.get("plannedHomePosition")
    if include_home and isinstance(ph, (list, tuple)) and len(ph) >= 3:
        try:
            lat_h = safe_float(ph[0])
            lon_h = safe_float(ph[1])
            alt_h = safe_float(ph[2])

            if abs(lat_h) > 1e-7 and abs(lon_h) > 1e-7:
                planned_home = (lat_h, lon_h, alt_h)
                home_item = {
                    "Altitude": alt_h,
                    "autoContinue": True,
                    "command": 16,  # MAV_CMD_NAV_WAYPOINT
                    "frame": mavutil.mavlink.MAV_FRAME_GLOBAL,
                    "params": [0, 0, 0, 0, lat_h, lon_h, alt_h],
                }
                mission_items = [home_item] + mission_items
                print(f"Added home position: lat={lat_h:.6f}, lon={lon_h:.6f}, alt={alt_h:.1f}m")
        except Exception as e:
            print(f"Warning: Failed to parse home position: {e}")

    return mission_items, planned_home

def clear_existing_mission(master):
    print("Clearing existing mission...")
    master.mav.mission_clear_all_send(
        master.target_system,
        master.target_component
    )
    time.sleep(1)

def upload_mission_to_autopilot(master, mission_items, planned_home=None):
    target_system = master.target_system
    target_component = master.target_component

    print(f"Uploading mission with {len(mission_items)} items")
    clear_existing_mission(master)

    master.mav.mission_count_send(target_system, target_component, len(mission_items))

    for _ in range(len(mission_items)):
        msg = master.recv_match(
            type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
            blocking=True,
            timeout=TIMEOUT_REQUEST
        )
        if msg is None:
            raise TimeoutError("No MISSION_REQUEST received")

        req_seq = msg.seq
        if req_seq >= len(mission_items):
            raise ValueError(f"Requested sequence {req_seq} beyond mission length {len(mission_items)}")

        item = mission_items[req_seq]
        cmd = int(item.get('command', 0))
        frame = get_frame_for_item(item, is_home=(req_seq == 0 and planned_home is not None))

        params = item.get('params', [])
        p1 = safe_float(params[0]) if len(params) > 0 else 0.0
        p2 = safe_float(params[1]) if len(params) > 1 else 0.0
        p3 = safe_float(params[2]) if len(params) > 2 else 0.0
        p4 = safe_float(params[3]) if len(params) > 3 else 0.0

        lat, lon, alt = extract_lat_lon_alt(item)
        alt_val = safe_float(alt) if alt is not None else 0.0

        if cmd in COORDLESS_COMMANDS:
            x_int, y_int = 0, 0
        elif lat is None or lon is None:
            if cmd == 16:  # MAV_CMD_NAV_WAYPOINT
                raise ValueError(f"Waypoint {req_seq} requires coordinates but none found")
            else:
                x_int, y_int = 0, 0
                print(f"Warning: Item {req_seq} (cmd={cmd}) has no coordinates, using (0,0)")
        else:
            x_int = int(round(lat * 1e7))
            y_int = int(round(lon * 1e7))

        master.mav.mission_item_int_send(
            target_system, target_component, req_seq, frame, cmd,
            0, 1,
            p1, p2, p3, p4,
            x_int, y_int, alt_val
        )
        print(f"Sent item {req_seq}: cmd={cmd}, lat={lat}, lon={lon}, alt={alt_val}")

    ack = master.recv_match(type=['MISSION_ACK'], timeout=TIMEOUT_ACK)
    print(f"[MISSION] MISSION_ACK received: {ack}")

    if not ack:
        print("⚠️ No MISSION_ACK received, assuming mission is loaded (all items sent)")
        return True

    if getattr(ack, "type", None) == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("✓ Mission upload accepted")
        return True
    else:
        print(f"⚠️ Mission ACK type={ack.type}, not MAV_MISSION_ACCEPTED")
        return True


# --- запуск миссии (AUTO + MISSION_START) ---

def check_armed(master, timeout: int = 3) -> bool:
    """Проверка по HEARTBEAT, что борт армлен."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            return (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    return False

def set_mode(master, mode: str, timeout: int = 10) -> bool:
    print(f"Setting mode to {mode}...")
    mode_mapping = master.mode_mapping()
    if mode not in mode_mapping:
        print(f"Unknown mode: {mode}")
        return False

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_mapping[mode]
    )

    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
        if msg and msg.custom_mode == mode_mapping[mode]:
            print(f"✓ Mode {mode} set successfully")
            return True
    print(f"[MODE] Timeout while setting {mode}")
    return False

def arm_copter(master, arm: bool = True, timeout: int = 10) -> bool:
    """
    Арминг/дизарминг с выводом STATUSTEXT.
    Не полагаемся только на COMMAND_ACK, ждём HEARTBEAT с изменённым флагом.
    """
    action = "arm" if arm else "disarm"
    print(f"Attempting to {action} copter...")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1 if arm else 0, 0, 0, 0, 0, 0, 0
    )

    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(
            type=['HEARTBEAT', 'STATUSTEXT', 'COMMAND_ACK'],
            blocking=True,
            timeout=1
        )
        if not msg:
            continue

        mtype = msg.get_type()

        if mtype == 'HEARTBEAT':
            armed_now = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if armed_now == arm:
                print(f"✓ Copter successfully {'armed' if arm else 'disarmed'} (by HEARTBEAT)!")
                return True

        elif mtype == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            print(f"[ARM] COMMAND_ACK: result={msg.result}")

        elif mtype == 'STATUSTEXT':
            text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else str(msg.text)
            print(f"[ARM] STATUSTEXT: {text}")

    print(f"[ARM] Timeout, failed to {action} copter")
    return False

def start_mission_auto(master, mission_items, takeoff_altitude: float = 10.0, uav_id: Optional[str] = None) -> bool:
    """
    Запуск миссии:
    1) Если не армлен — включаем безопасный режим (GUIDED/LOITER/STABILIZE),
       армим.
    2) Ставим AUTO.
    3) Отправляем MAV_CMD_MISSION_START.
    Прогресс отслеживается в listen_to_uav по MISSION_CURRENT/STATUSTEXT.
    """
    print("Starting mission in AUTO mode using MAV_CMD_MISSION_START...")

    mode_mapping = master.mode_mapping()
    pre_arm_mode = None
    for candidate in ["GUIDED", "LOITER", "STABILIZE", "ALT_HOLD"]:
        if candidate in mode_mapping:
            pre_arm_mode = candidate
            break
    if pre_arm_mode is None and mode_mapping:
        pre_arm_mode = list(mode_mapping.keys())[0]

    if not check_armed(master):
        print(f"[AUTO] Drone is not armed, switching to {pre_arm_mode} before arming...")
        if pre_arm_mode and not set_mode(master, pre_arm_mode, timeout=10):
            print(f"[AUTO] Failed to set pre-arm mode {pre_arm_mode}")
            if uav_id is not None:
                update_mission_state(uav_id, mission_status="error", mission_phase="mode_error")
            return False

        if not arm_copter(master, arm=True, timeout=20):
            print("[AUTO] Failed to arm copter")
            if uav_id is not None:
                update_mission_state(uav_id, mission_status="error", mission_phase="arm_error")
            return False
    else:
        print("[AUTO] Drone already armed")

    if not set_mode(master, "AUTO", timeout=10):
        print("[AUTO] Failed to set AUTO mode")
        if uav_id is not None:
            update_mission_state(uav_id, mission_status="error", mission_phase="mode_auto_error")
        return False

    print("[AUTO] Sending MAV_CMD_MISSION_START...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0,
        0, 0, 0, 0, 0, 0
    )

    if uav_id is not None:
        update_mission_state(
            uav_id,
            mission_status="running",
            mission_phase="in_progress",
        )
    return True


def upload_and_start_mission_for_uav(uav_id: str, takeoff_altitude: float = 10.0) -> bool:
    """
    Загружает .plan и запускает миссию, используя уже существующее MAVLink-соединение.
    На время загрузки/арминга ставит mission_comm_lock=True, чтобы телеметрия не читала сокет.
    """
    with uavs_lock:
        uav = UAVS.get(uav_id)
        if not uav:
            raise RuntimeError("БВС не подключен")

        plan_data = uav.get("plan_raw")
        if not plan_data:
            raise RuntimeError("Для БВС не загружен .plan")

        master = MAVLINK_CONNECTIONS.get(uav_id)
        if not master:
            raise RuntimeError("Нет MAVLink-соединения для БВС")

        # Включаем блокировку чтения в телеметрийном потоке
        uav["mission_comm_lock"] = True

    try:
        update_mission_state(
            uav_id,
            mission_status="starting",
            mission_phase="uploading",
            mission_current_seq=-1,
            mission_progress=0.0,
        )

        mission_items, planned_home = build_mission_items_from_plan(
            plan_data.get("mission", {})
        )
        print(f"[MISSION] Loaded {len(mission_items)} mission items for {uav_id}")

        update_mission_state(
            uav_id,
            mission_total=len(mission_items),
        )

        if not upload_mission_to_autopilot(master, mission_items, planned_home):
            update_mission_state(uav_id, mission_status="error", mission_phase="upload_error")
            raise RuntimeError("Ошибка загрузки миссии")

        print("=" * 50)
        print("STARTING MISSION IN AUTO (MISSION_START)")
        print("=" * 50)

        ok = start_mission_auto(master, mission_items, takeoff_altitude, uav_id=uav_id)
        if not ok:
            update_mission_state(uav_id, mission_status="error")
        return ok

    finally:
        # Снимаем блокировку чтения — телеметрия снова начинает читать сокет
        with uavs_lock:
            if uav_id in UAVS:
                UAVS[uav_id]["mission_comm_lock"] = False
        print("[MISSION] mission_comm_lock released")


def stop_mission_on_uav(uav_id: str):
    """
    Остановка миссии: перевод в LOITER/BRAKE/ALT_HOLD и пометка миссии как stopped.
    После этого listen_to_uav перестаёт учитывать MISSION_CURRENT.
    """
    with uavs_lock:
        master = MAVLINK_CONNECTIONS.get(uav_id)
        uav = UAVS.get(uav_id)

    if not master or not uav:
        raise RuntimeError("Нет подключения к БВС")

    try:
        mode_mapping = master.mode_mapping()
        # пробуем по приоритету от более «резких» к более мягким
        for candidate in ["BRAKE", "LOITER", "ALT_HOLD"]:
            if candidate in mode_mapping:
                print(f"[STOP] Switching {uav_id} to {candidate}")
                # тут ОСОЗНАННО используем master.set_mode (без ожидания),
                # чтобы не спорить с телеметрийным потоком за HEARTBEAT
                master.set_mode(candidate)
                break
        else:
            # если ни один из режимов не найден — fallback на LOITER_UNLIM командой
            print(f"[STOP] {uav_id}: no BRAKE/LOITER/ALT_HOLD, sending MAV_CMD_NAV_LOITER_UNLIM")
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )

        update_mission_state(
            uav_id,
            mission_status="stopped",
            mission_phase="stopped",
        )
        print(f"[STOP] Mission for {uav_id} marked as stopped")

    except Exception as e:
        raise RuntimeError(f"Ошибка при смене режима: {e}")


def mission_runner(uav_id: str, takeoff_altitude: float) -> None:
    """Фоновый поток: загрузка и запуск миссии."""
    try:
        ok = upload_and_start_mission_for_uav(uav_id, takeoff_altitude)
        if not ok:
            update_mission_state(
                uav_id,
                mission_status="error",
            )
    except Exception as e:
        print(f"[MISSION] mission_runner error for {uav_id}: {e}")
        update_mission_state(
            uav_id,
            mission_status="error",
            mission_phase="exception",
        )


# --- Старт фоновых потоков ---
print("[INIT] Запуск одноразового обнаружения БВС на портах 14550...")
discover_thread = threading.Thread(target=discover_uavs, daemon=True)
discover_thread.start()

heartbeat_thread = threading.Thread(target=check_heartbeats, daemon=True)
heartbeat_thread.start()


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
    """Загрузка .plan с фронта (multipart/form-data, поле 'file')."""
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

    eps = 1e-7
    with uavs_lock:
        uav = UAVS[uav_id]
        uav_lat = uav.get("lat") or 0.0
        uav_lon = uav.get("lon") or 0.0
        UAVS[uav_id]["mission"] = items
        UAVS[uav_id]["plan_raw"] = plan_data

    # Добавляем текущую позицию БВС в начало маршрута (если она не (0,0) и не совпадает с первой точкой)
    if waypoints and abs(uav_lat) > eps and abs(uav_lon) > eps:
        first_lat, first_lon = waypoints[0]
        if abs(first_lat - uav_lat) > eps or abs(first_lon - uav_lon) > eps:
            waypoints = [[uav_lat, uav_lon]] + waypoints

    return jsonify({
        "status": "ok",
        "items": items,
        "waypoints": waypoints
    })


@app.route("/uavs/<uav_id>/mission/start", methods=["POST"])
def start_mission(uav_id):
    data = request.get_json(silent=True) or {}
    takeoff_alt = float(data.get("takeoff_altitude", 10.0)) if isinstance(data, dict) else 10.0

    with uavs_lock:
        if uav_id not in UAVS:
            return jsonify({"error": "UAV not found"}), 404

        status = UAVS[uav_id].get("mission_status", "idle")
        if status in ("starting", "running"):
            return jsonify({"error": "mission already in progress"}), 400

    update_mission_state(
        uav_id,
        mission_status="starting",
        mission_phase="uploading",
        mission_current_seq=-1,
        mission_progress=0.0,
    )

    thread = threading.Thread(
        target=mission_runner,
        args=(uav_id, takeoff_alt),
        daemon=True,
    )
    thread.start()

    return jsonify({"status": "started"})


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
    """
    Принудительное обновление списка БВС.
    Никаких пересканирований — просто возвращаем текущее состояние.
    """
    uavs_list = get_serializable_uavs()
    return jsonify({
        "active_uavs": len(uavs_list),
        "uavs": [uav["name"] for uav in uavs_list],
        "items": uavs_list,
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
        w_url = (
            "https://api.open-meteo.com/v1/forecast"
            f"?latitude={lat}&longitude={lon}&current_weather=true"
        )
        w = requests.get(w_url, timeout=5)
        w_data = w.json()
        current = w_data.get("current_weather", {}) or {}

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
    app.run(host="0.0.0.0", port=5555, debug=True)
