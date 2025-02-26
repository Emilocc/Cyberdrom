import time
import threading
import queue
import numpy as np
import csv
import cv2
from datetime import datetime
from pyzbar import pyzbar
from pion.pion import Pion  # Для БВС (Scout и Transport)
from rzd import *  # Для видеопотока дронов
from omegabot_poligon77 import Robot  # Для РТС
import requests

# Глобальная очередь для передачи координат QR-кодов и aruco-меток
qr_locations = queue.Queue()

# Координаты объектов
START_POS_SCOUT_0 = (-3.92, -2.14, 0)  # Новая стартовая позиция Scout 0
START_POS_SCOUT_1 = (3.50, 3.15, 0)  # Новая стартовая позиция Scout 1
START_POS_TRANS_0 = (4, 0, 0)  # Нижний правый угол (группа 1)
START_POS_TRANS_1 = (4, 4, 0)  # Верхний правый угол (группа 2)
START_POS_RTS_0 = (2, 0, 0)  # Для крупных грузов (группа 1)
START_POS_RTS_1 = (2, 4, 0)  # Для расчистки завала (группа 2)
DEST_POS_1 = (-0.88, -3.1, 0)  # Населённый пункт 1
DEST_POS_2 = (-3.82, 0.84, 0)  # Населённый пункт 2
DEST_POS_3 = (5, 5, 0)  # Населённый пункт 3
STONE_DEST = (1.8, 3.15, 0)  # Площадка для камней
WOOD_DEST = (3.5, -0.24, 0)  # Площадка для деревьев
RAILWAY_START = (-5, 5, 0.03)  # Начало железнодорожного полотна
RAILWAY_END = (5, -5, 0.03)  # Конец железнодорожного полотна
WAGON_POS = (0, 0, 0.2)  # Условно центр вагонов


# Функция проверки принадлежности точки треугольнику (барицентрические координаты)
def is_point_in_triangle(x, y, p1, p2, p3):
    def sign(x1, y1, x2, y2, x3, y3):
        return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3)

    d1 = sign(x, y, p1[0], p1[1], p2[0], p2[1])
    d2 = sign(x, y, p2[0], p2[1], p3[0], p3[1])
    d3 = sign(x, y, p3[0], p3[1], p1[0], p1[1])

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

    return not (has_neg and has_pos)


# Функции для взаимодействия с геймкором
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


# Класс для хранения параметров полигона и информации о дронах
class CompetitionConfig:
    def __init__(self):
        self.field_size = (11, 11, 4)  # Полигон 11x11x4 м
        self.scout_drones = [
            {"id": 0, "start_pos": START_POS_SCOUT_0, "ip": "10.1.100.215", "mavlink_port": 5656, "camera_port": 8554},
            {"id": 1, "start_pos": START_POS_SCOUT_1, "ip": "10.1.100.217", "mavlink_port": 5656, "camera_port": 8554}
        ]
        self.trans_drones = [
            {"id": 0, "start_pos": START_POS_TRANS_0, "ip": "10.1.100.206", "mavlink_port": 5656, "camera_port": 8554},
            {"id": 1, "start_pos": START_POS_TRANS_1, "ip": "10.1.100.211", "mavlink_port": 5656, "camera_port": 8554}
        ]
        self.rts_units = [
            {"id": 0, "start_pos": START_POS_RTS_0, "ip": "10.1.100.215", "mavlink_port": 5656},
            {"id": 1, "start_pos": START_POS_RTS_1, "ip": "10.1.100.206", "mavlink_port": 5656}
        ]


# Функция для обработки QR-кода
def process_qr_code(drone_id, data):
    if data == "Груз БПЛА":
        print(f"Device {drone_id}: Обнаружен QR-код для погрузки БПЛА")
    else:
        print(f"Device {drone_id}: Обнаружен QR-код: {data}")
    save_qr_data_to_csv(drone_id, data)


# Функция для записи данных QR-кода в CSV файл
def save_qr_data_to_csv(drone_id, data, filename="qr_data.csv"):
    try:
        with open(filename, mode="a", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow([f"Device {drone_id}", data, datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
    except IOError as e:
        print(f"Ошибка записи в CSV: {e}")


# Класс для разведывательных БВС
class ScoutDrone:
    def __init__(self, drone_info):
        self.id = drone_info["id"]
        self.drone = Pion(ip=drone_info["ip"], mavlink_port=drone_info["mavlink_port"])
        self.start_pos = drone_info["start_pos"]
        self.camera = SocketCamera(ip=drone_info["ip"], port=drone_info["camera_port"])
        self.frame_center = (320, 240)
        self.qr_found = set()
        self.running = True
        self.height = 1.5 if self.id == 0 else 2.0  # Scout 0: 1.5 м, Scout 1: 2.0 м
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
                    print(
                        f"Scout {self.id}: Камера подключена успешно, кадр получен (попытка {attempt + 1}/{max_attempts})")
                    return True
                else:
                    print(
                        f"Scout {self.id}: Камера подключена, но кадр не получен (попытка {attempt + 1}/{max_attempts})")
            except Exception as e:
                print(f"Scout {self.id}: Ошибка при подключении к камере: {e} (попытка {attempt + 1}/{max_attempts})")
            time.sleep(2)
        print(f"Scout {self.id}: Не удалось подключиться к камере после {max_attempts} попыток")
        return False

    def smart_takeoff(self):
        print(f"Scout {self.id}: Взлет")
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                self.drone.arm()
                print(f"Scout {self.id}: Двигатели включены")
                time.sleep(0.5)
                self.drone.takeoff()
                print(f"Scout {self.id}: Команда на взлет отправлена, ожидаю набор высоты до {self.height} м")
                time.sleep(8)
                position = self.drone.position
                print(f"Scout {self.id}: Позиция после взлёта: {position}")
                if position is not None and len(position) >= 3:
                    current_height = float(position[2])
                    print(f"Scout {self.id}: Текущая высота: {current_height} м")
                    if current_height > 0.1:
                        return True
                else:
                    print(f"Scout {self.id}: Позиция недоступна или некорректна: {position}")
                print(f"Scout {self.id}: Взлёт не удался, повторная попытка {attempt + 1}/{max_attempts}")
            except Exception as e:
                print(f"Scout {self.id}: Ошибка при взлете: {e}")
                time.sleep(2)
        print(f"Scout {self.id}: Не удалось взлететь после {max_attempts} попыток")
        self.drone.land()
        self.drone.disarm()
        self.running = False
        return False

    def detect_qr(self):
        try:
            frame = self.camera.get_cv_frame()
            if frame is None:
                print(f"Scout {self.id}: Не удалось получить кадр")
                return None

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            qr_codes = pyzbar.decode(gray)
            if qr_codes:
                qr_code = qr_codes[0]
                data = qr_code.data.decode("utf-8")
                process_qr_code(self.id, data)
                points = np.array(qr_code.polygon)
                coords = self.calculate_coords(points)
                if data in ["Box 2 1", "Box 2 2", "Box 1 1", "Box 1 2", "Stone_1", "Wood_1", "Stone_2", "Wood_2"]:
                    detect_object(self.drone, data)
                    print(f"Scout {self.id}: QR-код {data} распознан, координаты: {coords}")
                    return {"key": data, "coords": coords}
                else:
                    print(f"Scout {self.id}: QR-код {data} не соответствует заданию, игнорируется")
            else:
                print(f"Scout {self.id}: QR-код не обнаружен в кадре")
            return None
        except Exception as e:
            print(f"Scout {self.id}: Ошибка с камерой: {e}")
            return None

    def calculate_coords(self, points):
        pos = self.drone.position[:3] if self.drone.position is not None else [0, 0, 0]
        qr_center = np.mean(points, axis=0)
        shift_x_px = qr_center[0] - self.frame_center[0]
        shift_y_px = qr_center[1] - self.frame_center[1]
        shift_x_m = (shift_x_px * self.height) / 700
        shift_y_m = (shift_y_px * self.height) / 700
        return [pos[0] + shift_x_m, pos[1] + shift_y_m, self.height]

    def show_video_stream(self):
        while self.running:
            try:
                frame = self.camera.get_cv_frame()
                if frame is not None:
                    display_frame = frame
                    gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
                    qr_codes = pyzbar.decode(gray)
                    for qr_code in qr_codes:
                        data = qr_code.data.decode("utf-8")
                        (x, y, w, h) = qr_code.rect
                        cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(display_frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.imshow(f"Scout {self.id} Stream", display_frame)
                    if cv2.waitKey(1) == 27:
                        break
                else:
                    print(f"Scout {self.id}: Кадр не получен")
            except Exception as e:
                print(f"Scout {self.id}: Ошибка с видеопотоком: {e}")
            time.sleep(0.1)

    def scout_mission(self):
        if not self.running:
            print(f"Scout {self.id}: Миссия невозможна из-за проблем с камерой")
            return
        if not self.smart_takeoff():
            print(f"Scout {self.id}: Миссия прервана из-за неудачного взлёта")
            return

        scan_points = []
        if self.id == 0:  # Scout 0: заданные координаты в треугольнике (-5,5), (5,5), (5,-5)
            fixed_points = [
                (3.5, 3.15, self.height),
                (0.5, 0.5, self.height),
                (-1.5, 2.5, self.height),
                (-1, 3, self.height),
                (3, -1, self.height),
                (2.5, -1.5, self.height),
                (1, 0, self.height),
                (3.5, 3.15, self.height)
            ]
            triangle_vertices = [(-5, 5), (5, 5), (5, -5)]
            for x, y, z in fixed_points:
                if is_point_in_triangle(x, y, *triangle_vertices):
                    scan_points.append((x, y, z))
            print(
                f"Scout {self.id}: Установлен маршрут по заданным координатам в треугольнике (-5,5), (5,5), (5,-5): {scan_points}")
        else:  # Scout 1: новый фиксированный маршрут
            scan_points = [
                (-3.92, -2.14, self.height),
                (-0.5, -0.5, self.height),
                (1.5, -2.5, self.height),
                (1, -3, self.height),
                (-3, -1, self.height),
                (2.5, 1.5, self.height),
                (-1, 0, self.height),
                (-3.92, -2.14, self.height)
            ]
            print(f"Scout {self.id}: Установлен маршрут по заданным координатам: {scan_points}")

        print(f"Scout {self.id}: Всего точек для траектории: {len(scan_points)}")
        found_qr_codes = set()
        for i, (x, y, z) in enumerate(scan_points):
            try:
                print(f"Scout {self.id}: Отправляю команду на движение к точке ({x}, {y}, {z})")
                self.drone.goto_from_outside(x, y, z, 0)
                print(f"Scout {self.id}: Лечу к точке ({x}, {y}, {z})")
                start_time = time.time()
                max_wait_time = 15.0
                while not self.drone.point_reached:
                    if time.time() - start_time > max_wait_time:
                        print(f"Scout {self.id}: Превышено время ожидания в точке ({x}, {y}, {z}), продолжаю движение")
                        break
                    qr_info = self.detect_qr()
                    if qr_info:
                        qr_key = qr_info["key"]
                        if qr_key and qr_key not in found_qr_codes:
                            found_qr_codes.add(qr_key)
                            qr_locations.put(qr_info)
                            print(
                                f"Scout {self.id}: QR-код {qr_key} найден, координаты: {qr_info['coords']}, найдено уникальных QR-кодов: {len(found_qr_codes)}")
                    time.sleep(0.05)
                    print(
                        f"Scout {self.id}: Текущая позиция: {self.drone.position[:3] if self.drone.position else 'неизвестно'}")
            except Exception as e:
                print(f"Scout {self.id}: Ошибка при полете: {e}")
                break

        if len(found_qr_codes) < 4:
            print(f"Scout {self.id}: Не найдены все 4 QR-кода, возвращаюсь на старт")
        else:
            print(f"Scout {self.id}: Найдены все 4 QR-кода, возвращаюсь на старт")
        self.return_to_start()

    def return_to_start(self):
        try:
            print(f"Scout {self.id}: Отправляю команду на возвращение к стартовой точке: {self.start_pos}")
            self.drone.goto_from_outside(self.start_pos[0], self.start_pos[1], self.start_pos[2], 0)
            print(f"Scout {self.id}: Возвращаюсь на старт")
            start_time = time.time()
            max_wait_time = 20.0
            while not self.drone.point_reached:
                if time.time() - start_time > max_wait_time:
                    print(f"Scout {self.id}: Превышено время ожидания возврата, аварийная посадка")
                    self.drone.land()
                    break
                print(
                    f"Scout {self.id}: Текущая позиция: {self.drone.position[:3] if self.drone.position else 'неизвестно'}, цель: {self.start_pos}")
                time.sleep(0.5)
            self.drone.land()
            self.drone.disarm()
            print(f"Scout {self.id}: Посадка и выключение двигателей")
            self.running = False
        except Exception as e:
            print(f"Scout {self.id}: Ошибка при возврате: {e}")
            self.drone.land()
            self.drone.disarm()
            self.running = False


# Класс для транспортных БВС
class TransportDrone:
    def __init__(self, drone_info):
        self.id = drone_info["id"]
        self.drone = Pion(ip=drone_info["ip"], mavlink_port=drone_info["mavlink_port"])
        self.start_pos = drone_info["start_pos"]
        self.camera = SocketCamera(ip=drone_info["ip"], port=drone_info["camera_port"])
        self.frame_center = (320, 240)
        self.running = True
        self.group = 1 if self.id == 0 else 2  # Группа 1 для id=0, группа 2 для id=1
        print(
            f"Transport {self.id}: Инициализация камеры на {drone_info['ip']}:{drone_info['camera_port']}, группа РТК: {self.group}")
        if not self.check_camera_connection():
            print(f"Transport {self.id}: Не удалось подключиться к камере, завершаю инициализацию")
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
                    print(
                        f"Transport {self.id}: Камера подключена успешно, кадр получен (попытка {attempt + 1}/{max_attempts})")
                    return True
                else:
                    print(
                        f"Transport {self.id}: Камера подключена, но кадр не получен (попытка {attempt + 1}/{max_attempts})")
            except Exception as e:
                print(
                    f"Transport {self.id}: Ошибка при подключении к камере: {e} (попытка {attempt + 1}/{max_attempts})")
            time.sleep(2)
        print(f"Transport {self.id}: Не удалось подключиться к камере после {max_attempts} попыток")
        return False

    def smart_takeoff(self):
        print(f"Transport {self.id}: Взлет")
        try:
            self.drone.arm()
            print(f"Transport {self.id}: Двигатели включены")
            time.sleep(0.5)
            self.drone.takeoff()
            print(f"Transport {self.id}: Команда на взлет отправлена")
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
            qr_codes = pyzbar.decode(gray)
            if qr_codes:
                qr_code = qr_codes[0]
                data = qr_code.data.decode("utf-8")
                process_qr_code(self.id, data)
                points = np.array(qr_code.polygon)
                coords = self.calculate_coords(points)
                detect_object(self.drone, data)
                print(f"Transport {self.id}: QR-код распознан: {data}, координаты: {coords}")
                return {"key": data, "coords": coords}
            else:
                print(f"Transport {self.id}: QR-код не обнаружен в кадре")
            return None
        except Exception as e:
            print(f"Transport {self.id}: Ошибка с камерой: {e}")
            return None

    def calculate_coords(self, points):
        pos = self.drone.position[:3] if self.drone.position is not None else [0, 0, 0]
        qr_center = np.mean(points, axis=0)
        shift_x_px = qr_center[0] - self.frame_center[0]
        shift_y_px = qr_center[1] - self.frame_center[1]
        shift_x_m = (shift_x_px * pos[2]) / 700
        shift_y_m = (shift_y_px * pos[2]) / 700
        return [pos[0] + shift_x_m, pos[1] + shift_y_m, pos[2]]

    def show_video_stream(self):
        while self.running:
            try:
                frame = self.camera.get_cv_frame()
                if frame is not None:
                    display_frame = frame
                    gray = cv2.cvtColor(display_frame, cv2.COLOR_BGR2GRAY)
                    qr_codes = pyzbar.decode(gray)
                    for qr_code in qr_codes:
                        data = qr_code.data.decode("utf-8")
                        (x, y, w, h) = qr_code.rect
                        cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(display_frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.imshow(f"Transport {self.id} Stream", display_frame)
                    if cv2.waitKey(1) == 27:
                        break
                else:
                    print(f"Transport {self.id}: Кадр не получен")
            except Exception as e:
                print(f"Transport {self.id}: Ошибка с видеопотоком: {e}")
            time.sleep(0.1)

    def transport_mission(self):
        if not self.running:
            print(f"Transport {self.id}: Миссия невозможна из-за проблем с камерой")
            return
        print(f"Transport {self.id}: Ожидаю координаты QR-кода (текущий размер очереди: {qr_locations.qsize()})")
        while qr_locations.qsize() == 0:
            print(f"Transport {self.id}: Очередь пуста, жду...")
            time.sleep(0.5)

        self.smart_takeoff()
        qr_info = qr_locations.get()
        x, y, z = qr_info["coords"]
        try:
            print(f"Transport {self.id}: Лечу к QR-коду на ({x}, {y}, {z})")
            self.drone.goto_from_outside(x, y, z, 0)
            while not self.drone.point_reached:
                time.sleep(0.1)
            print(f"Transport {self.id}: Приземляюсь для погрузки")
            self.drone.land()
            get_box(self.drone)
            time.sleep(3)

            qr_data = qr_info["key"]
            if self.group == 1:  # Группа РТК 1
                if "Box 1 1" in qr_data:
                    dest_x, dest_y, _ = DEST_POS_1
                elif "Box 1 2" in qr_data:
                    dest_x, dest_y, _ = DEST_POS_3
                else:
                    dest_x, dest_y, _ = DEST_POS_1  # По умолчанию
            else:  # Группа РТК 2
                if "Box 2 1" in qr_data:
                    dest_x, dest_y, _ = DEST_POS_2
                elif "Box 2 2" in qr_data:
                    dest_x, dest_y, _ = DEST_POS_3
                else:
                    dest_x, dest_y, _ = DEST_POS_2  # По умолчанию

            self.smart_takeoff()
            print(f"Transport {self.id}: Транспортирую груз в пункт назначения ({dest_x}, {dest_y}, 0)")
            self.drone.goto_from_outside(dest_x, dest_y, 0, 0)
            while not self.drone.point_reached:
                time.sleep(0.1)
            self.drone.land()
            drop_box(self.drone)
            print(f"Transport {self.id}: Посадка у пункта назначения")
            time.sleep(3)

            print(f"Transport {self.id}: Возвращаюсь на старт")
            self.drone.goto_from_outside(self.start_pos[0], self.start_pos[1], self.start_pos[2], 0)
            while not self.drone.point_reached:
                time.sleep(0.1)
            self.drone.land()
            self.drone.disarm()
            print(f"Transport {self.id}: Посадка и выключение двигателей")
            self.running = False
        except Exception as e:
            print(f"Transport {self.id}: Ошибка при полете: {e}")
            self.running = False


# Класс для транспортных РТС
class TransportRTS:
    def __init__(self, rts_info):
        self.id = rts_info["id"]
        self.robot = Robot(ip=rts_info["ip"], port=rts_info["mavlink_port"])
        self.start_pos = rts_info["start_pos"]
        self.running = True
        self.is_obstacle_removal = (self.id == 1)  # Группа 2 для расчистки завала
        print(f"TransportRTS {self.id}: Инициализация, is_obstacle_removal: {self.is_obstacle_removal}")

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
        if self.is_obstacle_removal:  # Группа 2: расчистка завала
            if "Stone_2" in qr_data:
                dest_x, dest_y, _ = STONE_DEST
            elif "Wood_2" in qr_data:
                dest_x, dest_y, _ = WOOD_DEST
            else:
                dest_x, dest_y, _ = STONE_DEST  # По умолчанию
            print(f"TransportRTS {self.id}: Перемещаю препятствие на ({dest_x}, {dest_y}, 0)")
        else:  # Группа 1: крупные грузы
            if "Wood_1" in qr_data:
                dest_x, dest_y, _ = DEST_POS_1
            elif "Stone_1" in qr_data:
                dest_x, dest_y, _ = DEST_POS_2
            else:
                dest_x, dest_y, _ = DEST_POS_1  # По умолчанию
            print(f"TransportRTS {self.id}: Перемещаю крупный груз на ({dest_x}, {dest_y}, 0)")
        self.move_to(dest_x, dest_y, 0)
        if not self.is_obstacle_removal:
            drop_box(self.robot)
        self.move_to(self.start_pos[0], self.start_pos[1], self.start_pos[2])
        print(f"TransportRTS {self.id}: Возвращаюсь на старт")
        self.running = False


# Главная функция
def main():
    config = CompetitionConfig()

    scouts = []
    for i in range(2):
        try:
            scouts.append(ScoutDrone(config.scout_drones[i]))
        except Exception as e:
            print(f"Не удалось инициализировать Scout {i}: {e}")

    transports = []
    for i in range(2):
        try:
            transports.append(TransportDrone(config.trans_drones[i]))
        except Exception as e:
            print(f"Не удалось инициализировать Transport {i}: {e}")

    rts_units = [TransportRTS(config.rts_units[i]) for i in range(2)]

    scout_threads = [threading.Thread(target=s.scout_mission) for s in scouts if s.running]
    transport_threads = [threading.Thread(target=t.transport_mission) for t in transports if t.running]
    rts_threads = [
        threading.Thread(target=rts_units[0].rts_mission),
        threading.Thread(target=rts_units[1].rts_mission)
    ]

    # Запускаем развед-дроны
    for t in scout_threads:
        t.start()

    # Ждем завершения миссий развед-дронов
    for t in scout_threads:
        t.join()

    print(f"Разведка завершена, размер очереди QR-кодов: {qr_locations.qsize()}")

    # Запускаем дроны-доставщики и РТС, если есть хотя бы 1 QR-код
    if qr_locations.qsize() > 0 or not scout_threads:
        for t in transport_threads + rts_threads:
            t.start()

        for t in transport_threads + rts_threads:
            t.join()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()