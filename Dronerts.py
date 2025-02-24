import time
import threading
import queue
import numpy as np
import csv
import cv2
import cv2.aruco as aruco  # Добавлен импорт для ArUco
from datetime import datetime
from pyzbar import pyzbar
from pion.pion import Pion  # Для БВС (Scout и Transport)
from rzd import *  # Для видеопотока дронов
from omegabot_poligon77 import Robot  # Для РТС
import requests

# Глобальная очередь для передачи координат QR-кодов и ArUco-меток
qr_locations = queue.Queue()

# Координаты объектов (без изменений)
START_POS_SCOUT_0 = (0, 0, 0)
START_POS_SCOUT_1 = (0, 4, 0)
START_POS_TRANS_0 = (4, 0, 0)
START_POS_TRANS_1 = (4, 4, 0)
START_POS_RTS_0 = (2, 0, 0)
START_POS_RTS_1 = (2, 4, 0)
DEST_POS_1 = (-0.88, -3.1)
DEST_POS_2 = (-3.82, 0.84)
DEST_POS_3 = (5, 5)
STONE_DEST = (1.8, 3.15)
WOOD_DEST = (3.5, -0.24)
RAILWAY_START = (-5, 5, 0.03)
RAILWAY_END = (5, -5, 0.03)
WAGON_POS = (0, 0, 0.2)

def is_near_railway(x, y):
    distance = abs(-x + y - 5) / np.sqrt(2)
    return distance <= 2

def detect_object(drone, detect_code):
    print(f"detect_object(), ip: {drone.ip}, detect_code: {detect_code}")
    try:
        requests.get("http://10.1.100.6:31556/detect_object",
                     params={"object": f"{detect_code.replace(' ', '_')}", "host": drone.ip[-3:]}).text
    except:
        print("Геймкор выключен")

def get_box(drone):
    print(f"get_box(), ip: {drone.ip}")
    try:
        requests.get("http://10.1.100.6:31556/get_box", params={"host": drone.ip[-3:]}).text
    except:
        print("Геймкор выключен")

def drop_box(drone):
    print(f"drop_box(), ip: {drone.ip}")
    try:
        requests.get("http://10.1.100.6:31556/drop_object", params={"host": drone.ip[-3:]}).text
    except:
        print("Геймкор выключен")

class CompetitionConfig:
    def __init__(self):
        self.field_size = (11, 11, 4)
        self.scout_drones = [
            {"id": 0, "start_pos": START_POS_SCOUT_0, "ip": "127.0.0.1", "mavlink_port": 8005, "camera_port": 18005},
            {"id": 1, "start_pos": START_POS_SCOUT_1, "ip": "127.0.0.1", "mavlink_port": 8006, "camera_port": 18006}
        ]
        self.trans_drones = [
            {"id": 0, "start_pos": START_POS_TRANS_0, "ip": "127.0.0.1", "mavlink_port": 8000, "camera_port": 18000},
            {"id": 1, "start_pos": START_POS_TRANS_1, "ip": "127.0.0.1", "mavlink_port": 8001, "camera_port": 18001}
        ]
        self.rts_units = [
            {"id": 0, "start_pos": START_POS_RTS_0, "ip": "127.0.0.1", "mavlink_port": 8004},
            {"id": 1, "start_pos": START_POS_RTS_1, "ip": "127.0.0.1", "mavlink_port": 8003}
        ]

def process_qr_code(drone_id, data):
    if data == "Груз БПЛА":
        print(f"Device {drone_id}: Обнаружен QR-код для погрузки БПЛА")
    else:
        print(f"Device {drone_id}: Обнаружен QR-код: {data}")
    save_qr_data_to_csv(drone_id, data)

def process_aruco_marker(drone_id, marker_id):
    if marker_id == 0:  # ID 0 для "Груз РТС"
        print(f"Device {drone_id}: Обнаружена ArUco-метка для погрузки РТС (ID: {marker_id})")
    else:
        print(f"Device {drone_id}: Обнаружена ArUco-метка с ID: {marker_id}")
    save_qr_data_to_csv(drone_id, f"ArUco_{marker_id}")  # Сохраняем как "ArUco_ID"

def save_qr_data_to_csv(drone_id, data, filename="qr_data.csv"):
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([f"Device {drone_id}", data, datetime.now().strftime("%Y-%m-%d %H:%M:%S")])

class ScoutDrone:
    def __init__(self, drone_info):
        self.id = drone_info["id"]
        self.drone = Pion(ip=drone_info["ip"], mavlink_port=drone_info["mavlink_port"])
        self.start_pos = drone_info["start_pos"]
        self.camera = SocketCamera(ip=drone_info["ip"], port=drone_info["camera_port"])
        self.frame_center = (320, 240)
        self.qr_found = set()
        self.running = True
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # Словарь ArUco
        self.aruco_params = aruco.DetectorParameters()  # Параметры детектора ArUco
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        print(f"Scout {self.id}: Инициализация камеры на {drone_info['ip']}:{drone_info['camera_port']}")
        if not self.check_camera_connection():
            print(f"Scout {self.id}: Не удалось подключиться к камере, завершаю инициализацию")
            self.running = False
            return
        self.video_thread = threading.Thread(target=self.show_video_stream)
        self.video_thread.start()

    def check_camera_connection(self):
        max_attempts = 10
        for attempt in range(max_attempts):
            try:
                frame = self.camera.get_cv_frame()
                if frame is not None:
                    print(f"Scout {self.id}: Камера подключена успешно (попытка {attempt + 1}/{max_attempts})")
                    return True
            except Exception as e:
                print(f"Scout {self.id}: Ошибка при подключении: {e} (попытка {attempt + 1}/{max_attempts})")
            time.sleep(2)
        print(f"Scout {self.id}: Не удалось подключиться после {max_attempts} попыток")
        return False

    def smart_takeoff(self):
        print(f"Scout {self.id}: Взлет")
        try:
            self.drone.arm()
            time.sleep(0.5)
            self.drone.takeoff()
            time.sleep(8)
        except Exception as e:
            print(f"Scout {self.id}: Ошибка при взлете: {e}")

    def detect_qr(self):
        try:
            frame = self.camera.get_cv_frame()
            if frame is None:
                print(f"Scout {self.id}: Не удалось получить кадр")
                return None

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Обнаружение QR-кодов
            qr_codes = pyzbar.decode(gray)
            if qr_codes:
                qr_code = qr_codes[0]
                data = qr_code.data.decode("utf-8")
                process_qr_code(self.id, data)
                points = np.array(qr_code.polygon)
                coords = self.calculate_coords(points)
                detect_object(self.drone, data)
                return {"key": data, "coords": coords}

            # Обнаружение ArUco-меток
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
            if ids is not None:
                marker_id = ids[0][0]
                process_aruco_marker(self.id, marker_id)
                points = np.array(corners[0])
                coords = self.calculate_coords(points)
                detect_object(self.drone, f"ArUco_{marker_id}")
                return {"key": f"ArUco_{marker_id}", "coords": coords}

            print(f"Scout {self.id}: Ни QR-код, ни ArUco-метка не обнаружены")
            return None
        except Exception as e:
            print(f"Scout {self.id}: Ошибка с камерой: {e}")
            return None

    def calculate_coords(self, points):
        pos = self.drone.position[:3] if self.drone.position is not None else [0, 0, 0]
        center = np.mean(points, axis=0)
        shift_x_px = center[0] - self.frame_center[0]
        shift_y_px = center[1] - self.frame_center[1]
        shift_x_m = (shift_x_px * pos[2]) / 700
        shift_y_m = (shift_y_px * pos[2]) / 700
        return [pos[0] + shift_x_m, pos[1] + shift_y_m, pos[2]]

    def show_video_stream(self):
        while self.running:
            try:
                frame = self.camera.get_cv_frame()
                if frame is not None:
                    display_frame = frame.copy()
                    gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)

                    # Отрисовка QR-кодов
                    qr_codes = pyzbar.decode(gray)
                    for qr_code in qr_codes:
                        data = qr_code.data.decode("utf-8")
                        (x, y, w, h) = qr_code.rect
                        cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(display_frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Отрисовка ArUco-меток
                    corners, ids, _ = self.aruco_detector.detectMarkers(display_frame)
                    if ids is not None:
                        for i in range(len(ids)):
                            marker_corners = corners[i][0]
                            for j in range(4):
                                start_point = tuple(map(int, marker_corners[j]))
                                end_point = tuple(map(int, marker_corners[(j + 1) % 4]))
                                cv2.line(display_frame, start_point, end_point, (0, 255, 0), 2)
                            marker_id = ids[i][0]
                            cv2.putText(display_frame, str(marker_id), tuple(map(int, marker_corners[0])),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    cv2.imshow(f"Scout {self.id} Stream", display_frame)
                    if cv2.waitKey(1) == 27:
                        break
            except Exception as e:
                print(f"Scout {self.id}: Ошибка с видеопотоком: {e}")
            time.sleep(0.1)

    def scout_mission(self):
        if not self.running:
            print(f"Scout {self.id}: Миссия невозможна из-за проблем с камерой")
            return
        self.smart_takeoff()
        scan_points = []
        height = 2.0
        if self.id == 0:
            for x in np.arange(-2, 3, 2):
                y_base = -x + 5
                for y in np.arange(max(0, y_base - 2), min(5, y_base + 2) + 1, 2):
                    if is_near_railway(x, y):
                        scan_points.append((x, y, height))
        else:
            for x in np.arange(-3, 2, 2):
                y_base = -x + 5
                for y in np.arange(max(0, y_base - 2), min(5, y_base + 2) + 1, 2):
                    if is_near_railway(x, y):
                        scan_points.append((x, y, height))

        print(f"Scout {self.id}: Всего точек для облета: {len(scan_points)}")
        found_codes = set()
        for x, y, z in scan_points:
            try:
                self.drone.goto_from_outside(x, y, z, 0)
                print(f"Scout {self.id}: Лечу к точке ({x}, {y}, {z})")
                while not self.drone.point_reached:
                    code_info = self.detect_qr()
                    if code_info:
                        code_key = code_info["key"]
                        if code_key in ["Box 2 1", "Box 2 2", "Box 1 1", "Box 1 2", "Stone_1", "Wood_1", "Stone_2",
                                        "Wood_2", "ArUco_0"]:
                            found_codes.add(code_key)
                            qr_locations.put(code_info)
                            print(f"Scout {self.id}: Код {code_key} найден, координаты переданы: {code_info['coords']}")
                        if len(found_codes) >= 4:
                            print(f"Scout {self.id}: Найдено достаточно кодов, возвращаюсь на старт")
                            self.return_to_start()
                            return
                    time.sleep(0.5)
                code_info = self.detect_qr()
                if code_info:
                    code_key = code_info["key"]
                    if code_key in ["Box 2 1", "Box 2 2", "Box 1 1", "Box 1 2", "Stone_1", "Wood_1", "Stone_2",
                                    "Wood_2", "ArUco_0"]:
                        found_codes.add(code_key)
                        qr_locations.put(code_info)
                        print(f"Scout {self.id}: Код {code_key} найден, координаты переданы: {code_info['coords']}")
                    if len(found_codes) >= 4:
                        print(f"Scout {self.id}: Найдено достаточно кодов, возвращаюсь на старт")
                        self.return_to_start()
                        return
            except Exception as e:
                print(f"Scout {self.id}: Ошибка при полете: {e}")
                break
        print(f"Scout {self.id}: Не найдено достаточно кодов, возвращаюсь на старт")
        self.return_to_start()

    def return_to_start(self):
        try:
            self.drone.goto_from_outside(self.start_pos[0], self.start_pos[1], 0, 0)
            while not self.drone.point_reached:
                time.sleep(0.5)
            self.drone.land()
            self.running = False
        except Exception as e:
            print(f"Scout {self.id}: Ошибка при возврате: {e}")

class TransportDrone:
    def __init__(self, drone_info):
        self.id = drone_info["id"]
        self.drone = Pion(ip=drone_info["ip"], mavlink_port=drone_info["mavlink_port"])
        self.start_pos = drone_info["start_pos"]
        self.camera = SocketCamera(ip=drone_info["ip"], port=drone_info["camera_port"])
        self.frame_center = (320, 240)
        self.running = True
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        print(f"Transport {self.id}: Инициализация камеры на {drone_info['ip']}:{drone_info['camera_port']}")
        if not self.check_camera_connection():
            print(f"Transport {self.id}: Не удалось подключиться к камере")
            self.running = False
            return
        self.video_thread = threading.Thread(target=self.show_video_stream)
        self.video_thread.start()

    def check_camera_connection(self):
        max_attempts = 10
        for attempt in range(max_attempts):
            try:
                frame = self.camera.get_cv_frame()
                if frame is not None:
                    print(f"Transport {self.id}: Камера подключена успешно (попытка {attempt + 1}/{max_attempts})")
                    return True
            except Exception as e:
                print(f"Transport {self.id}: Ошибка при подключении: {e} (попытка {attempt + 1}/{max_attempts})")
            time.sleep(2)
        print(f"Transport {self.id}: Не удалось подключиться после {max_attempts} попыток")
        return False

    def smart_takeoff(self):
        print(f"Transport {self.id}: Взлет")
        try:
            self.drone.arm()
            time.sleep(0.5)
            self.drone.takeoff()
            time.sleep(8)
        except Exception as e:
            print(f"Transport {self.id}: Ошибка при взлете: {e}")

    def detect_qr(self):
        try:
            frame = self.camera.get_cv_frame()
            if frame is None:
                print(f"Transport {self.id}: Не удалось получить кадр")
                return None

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Обнаружение QR-кодов
            qr_codes = pyzbar.decode(gray)
            if qr_codes:
                qr_code = qr_codes[0]
                data = qr_code.data.decode("utf-8")
                process_qr_code(self.id, data)
                points = np.array(qr_code.polygon)
                coords = self.calculate_coords(points)
                detect_object(self.drone, data)
                return {"key": data, "coords": coords}

            # Обнаружение ArUco-меток
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
            if ids is not None:
                marker_id = ids[0][0]
                process_aruco_marker(self.id, marker_id)
                points = np.array(corners[0])
                coords = self.calculate_coords(points)
                detect_object(self.drone, f"ArUco_{marker_id}")
                return {"key": f"ArUco_{marker_id}", "coords": coords}

            print(f"Transport {self.id}: Ни QR-код, ни ArUco-метка не обнаружены")
            return None
        except Exception as e:
            print(f"Transport {self.id}: Ошибка с камерой: {e}")
            return None

    def calculate_coords(self, points):
        pos = self.drone.position[:3] if self.drone.position is not None else [0, 0, 0]
        center = np.mean(points, axis=0)
        shift_x_px = center[0] - self.frame_center[0]
        shift_y_px = center[1] - self.frame_center[1]
        shift_x_m = (shift_x_px * pos[2]) / 700
        shift_y_m = (shift_y_px * pos[2]) / 700
        return [pos[0] + shift_x_m, pos[1] + shift_y_m, pos[2]]

    def show_video_stream(self):
        while self.running:
            try:
                frame = self.camera.get_cv_frame()
                if frame is not None:
                    display_frame = frame.copy()
                    gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)

                    # Отрисовка QR-кодов
                    qr_codes = pyzbar.decode(gray)
                    for qr_code in qr_codes:
                        data = qr_code.data.decode("utf-8")
                        (x, y, w, h) = qr_code.rect
                        cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(display_frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Отрисовка ArUco-меток
                    corners, ids, _ = self.aruco_detector.detectMarkers(display_frame)
                    if ids is not None:
                        for i in range(len(ids)):
                            marker_corners = corners[i][0]
                            for j in range(4):
                                start_point = tuple(map(int, marker_corners[j]))
                                end_point = tuple(map(int, marker_corners[(j + 1) % 4]))
                                cv2.line(display_frame, start_point, end_point, (0, 255, 0), 2)
                            marker_id = ids[i][0]
                            cv2.putText(display_frame, str(marker_id), tuple(map(int, marker_corners[0])),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    cv2.imshow(f"Transport {self.id} Stream", display_frame)
                    if cv2.waitKey(1) == 27:
                        break
            except Exception as e:
                print(f"Transport {self.id}: Ошибка с видеопотоком: {e}")
            time.sleep(0.1)

    def transport_mission(self):
        if not self.running:
            print(f"Transport {self.id}: Миссия невозможна из-за проблем с камерой")
            return
        print(f"Transport {self.id}: Ожидаю координаты (текущий размер очереди: {qr_locations.qsize()})")
        while qr_locations.qsize() == 0:
            time.sleep(0.5)

        self.smart_takeoff()
        qr_info = qr_locations.get()
        x, y, z = qr_info["coords"]
        try:
            print(f"Transport {self.id}: Лечу к коду на ({x}, {y}, {z})")
            self.drone.goto_from_outside(x, y, z, 0)
            while not self.drone.point_reached:
                time.sleep(0.1)
            self.drone.land()
            get_box(self.drone)
            time.sleep(3)

            qr_data = qr_info["key"]
            if "Box 1 1" in qr_data:
                dest_x, dest_y = DEST_POS_1
            elif "Box 2 1" in qr_data:
                dest_x, dest_y = DEST_POS_2
            elif "Box 2 2" in qr_data or "Box 1 2" in qr_data:
                dest_x, dest_y = DEST_POS_3
            elif "ArUco_0" in qr_data:  # Пример для ArUco
                dest_x, dest_y = DEST_POS_1  # Можно настроить
            else:
                dest_x, dest_y = DEST_POS_1

            self.smart_takeoff()
            print(f"Transport {self.id}: Транспортирую груз в ({dest_x}, {dest_y}, 0)")
            self.drone.goto_from_outside(dest_x, dest_y, 0, 0)
            while not self.drone.point_reached:
                time.sleep(0.1)
            self.drone.land()
            drop_box(self.drone)
            time.sleep(3)

            self.drone.goto_from_outside(self.start_pos[0], self.start_pos[1], 0, 0)
            while not self.drone.point_reached:
                time.sleep(0.1)
            self.drone.land()
            print(f"Transport {self.id}: Посадка на стартовой позиции")
        except Exception as e:
            print(f"Transport {self.id}: Ошибка при полете: {e}")
        self.running = False

class TransportRTS:
    def __init__(self, rts_info):
        self.id = rts_info["id"]
        self.robot = Robot(ip=rts_info["ip"], port=rts_info["mavlink_port"])
        self.start_pos = rts_info["start_pos"]
        self.running = True
        print(f"TransportRTS {self.id}: Инициализация")

    def move_to(self, x, y, z):
        print(f"TransportRTS {self.id}: Двигаюсь к точке ({x}, {y}, {z})")
        try:
            self.robot.move_to(x, y, z)
            while not self.robot.is_at_position(x, y, z):
                time.sleep(0.1)
        except Exception as e:
            print(f"TransportRTS {self.id}: Ошибка при движении: {e}")

    def rts_mission(self, is_obstacle_removal=False):
        print(f"TransportRTS {self.id}: Ожидаю координаты (текущий размер очереди: {qr_locations.qsize()})")
        while qr_locations.qsize() == 0:
            time.sleep(0.5)
        qr_info = qr_locations.get()
        x, y, z = qr_info["coords"]
        qr_data = qr_info["key"]
        self.move_to(x, y, z)
        detect_object(self.robot, qr_data)
        if is_obstacle_removal:
            if "Stone" in qr_data:
                dest_x, dest_y = STONE_DEST
            else:
                dest_x, dest_y = WOOD_DEST
            print(f"TransportRTS {self.id}: Перемещаю препятствие на ({dest_x}, {dest_y}, 0)")
        else:
            if "Wood_1" in qr_data:
                dest_x, dest_y = DEST_POS_1
            elif "Stone_1" in qr_data:
                dest_x, dest_y = DEST_POS_2
            elif "ArUco_0" in qr_data:
                dest_x, dest_y = DEST_POS_3  # Пример для ArUco
            else:
                dest_x, dest_y = DEST_POS_3
            print(f"TransportRTS {self.id}: Перемещаю крупный груз на ({dest_x}, {dest_y}, 0)")
        self.move_to(dest_x, dest_y, 0)
        if not is_obstacle_removal:
            drop_box(self.robot)
        self.move_to(self.start_pos[0], self.start_pos[1], 0)
        print(f"TransportRTS {self.id}: Возвращаюсь на старт")
        self.running = False

obstacles_map = {
    "lines": [],
    "obs_med": ["Stone_1", "Wood_1"],
    "obs_large": ["Stone_2", "Wood_2"],
    "sklads_l": [(STONE_DEST[0], STONE_DEST[1], 0)],
    "sklads_r": [(WOOD_DEST[0], WOOD_DEST[1], 0)],
    "vzaimosv": {
        "Stone_2": STONE_DEST,
        "Wood_2": WOOD_DEST,
        "Stone_1": DEST_POS_2,
        "Wood_1": DEST_POS_1,
        "ArUco_0": DEST_POS_3  # Добавлено для примера
    },
    "home_point_r": START_POS_RTS_0,
    "home_point_l": START_POS_RTS_1
}

def main():
    config = CompetitionConfig()
    scouts = [ScoutDrone(config.scout_drones[i]) for i in range(2)]
    transports = [TransportDrone(config.trans_drones[i]) for i in range(2)]
    rts_units = [TransportRTS(config.rts_units[i]) for i in range(2)]

    scout_threads = [threading.Thread(target=s.scout_mission) for s in scouts if s.running]
    transport_threads = [threading.Thread(target=t.transport_mission) for t in transports if t.running]
    rts_threads = [
        threading.Thread(target=rts_units[0].rts_mission, args=(False,)),
        threading.Thread(target=rts_units[1].rts_mission, args=(True,))
    ]

    for t in scout_threads:
        t.start()
    for t in scout_threads:
        t.join()

    print(f"Разведка завершена, размер очереди кодов: {qr_locations.qsize()}")

    for t in transport_threads + rts_threads:
        t.start()
    for t in transport_threads + rts_threads:
        t.join()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()