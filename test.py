import time
import threading
import queue
import numpy as np
import csv
import cv2
from datetime import datetime
from pyzbar import pyzbar
from pion import Pion  # Используем Pion из библиотеки pion
from rzd import *  # Для видеопотока дронов


# Глобальная очередь для передачи координат QR-кодов
qr_locations = queue.Queue()

# Координаты объектов
START_POS_SCOUT_0 = (0, 0, 0)  # Нижний левый угол
START_POS_SCOUT_1 = (0, 4, 0)  # Верхний левый угол
START_POS_TRANS_0 = (4, 0, 0)  # Нижний правый угол
START_POS_TRANS_1 = (4, 4, 0)  # Верхний правый угол
START_POS_RTS_0 = (2, 0, 0)   # Пример для РТС 0 (крупные грузы)
START_POS_RTS_1 = (2, 4, 0)   # Пример для РТС 1 (расчистка завала)
DEST_POS_1 = (-0.88, -3.1)    # Населённый пункт 1
DEST_POS_2 = (-3.82, 0.84)    # Населённый пункт 2
DEST_POS_3 = (5, 5)           # Населённый пункт 3 (условно)
STONE_DEST = (1.8, 3.15)      # Площадка для камней
WOOD_DEST = (3.5, -0.24)      # Площадка для деревьев
RAILWAY_START = (-5, 5, 0.03) # Начало железнодорожного полотна
RAILWAY_END = (5, -5, 0.03)   # Конец железнодорожного полотна
WAGON_POS = (0, 0, 0.2)       # Условно центр вагонов



class WeatherDetector:
    def __init__(self, url="http://127.0.0.1:8003/get_weather"):
        self.url = url

    def get_weather(self):
        try:
            response = requests.get(self.url)
            if response.status_code == 200:
                image = Image.open(BytesIO(response.content))
                image_np = np.array(image)
                height, width, _ = image_np.shape
                bottom_right_pixel = image_np[height - 1, width - 1]
                r, g, b = bottom_right_pixel
                print(f"Цвет в нижнем правом углу (BGR): ({b}, {g}, {r})")
                return {"r": int(r), "g": int(g), "b": int(b)}
            else:
                print(f"Ошибка: {response.status_code}")
                return None
        except Exception as e:
            print(f"Ошибка при получении погоды: {e}")
            return None


# Класс для хранения параметров полигона и информации о дронах
class CompetitionConfig:
    def __init__(self):
        self.field_size = (11, 11, 4)  # Полигон 11x11x4 м
        self.scout_drones = [
            {"id": 0, "start_pos": START_POS_SCOUT_0, "ip": "127.0.0.1", "mavlink_port": 8001, "camera_port": 18004},
            {"id": 1, "start_pos": START_POS_SCOUT_1, "ip": "127.0.0.1", "mavlink_port": 8002, "camera_port": 18005}
        ]
        self.trans_drones = [
            {"id": 0, "start_pos": START_POS_TRANS_0, "ip": "127.0.0.1", "mavlink_port": 8005, "camera_port": 18006},
            {"id": 1, "start_pos": START_POS_TRANS_1, "ip": "127.0.0.1", "mavlink_port": 8006, "camera_port": 18007}
        ]
        self.rts_units = [
            {"id": 0, "start_pos": START_POS_RTS_0, "ip": "127.0.0.1", "mavlink_port": 8007, "camera_port": 18008},  # Для крупных грузов
            {"id": 1, "start_pos": START_POS_RTS_1, "ip": "127.0.0.1", "mavlink_port": 8008, "camera_port": 18009}   # Для расчистки завала
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
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([f"Device {drone_id}", data, datetime.now().strftime("%Y-%m-%d %H:%M:%S")])

# Класс для разведывательных БВС
class ScoutDrone:
    def __init__(self, drone_info):
        self.id = drone_info["id"]
        self.drone = Pion(ip=drone_info["ip"], mavlink_port=drone_info["mavlink_port"])
        self.start_pos = drone_info["start_pos"]
        self.camera = SocketCamera(ip=drone_info["ip"], port=drone_info["camera_port"])
        self.frame_center = (320, 240)  # Для 640x480
        self.qr_found = False
        self.running = True
        self.weather_detector = WeatherDetector()  # Добавляем детектор погоды
        print(f"Scout {self.id}: Инициализация камеры на {drone_info['ip']}:{drone_info['camera_port']}")
        self.check_camera_connection()
        self.video_thread = threading.Thread(target=self.show_video_stream)
        self.video_thread.start()

    def check_weather(self):
        weather_data = self.weather_detector.get_weather()
        if weather_data:
            r, g, b = weather_data["r"], weather_data["g"], weather_data["b"]
            # Пример логики: если цвет слишком темный (плохая погода), откладываем миссию
            if r < 50 and g < 50 and b < 50:
                print(f"Scout {self.id}: Плохая погода, откладываем миссию")
                return False
            else:
                print(f"Scout {self.id}: Погода нормальная, продолжаем миссию")
                return True
        else:
            print(f"Scout {self.id}: Не удалось получить данные о погоде, продолжаем миссию")
            return True

    def scout_mission(self, config):
        if not self.check_weather():  # Проверяем погоду перед началом миссии
            return

        self.smart_takeoff()
        scan_points = []
        if self.id == 0:
            for x in np.arange(0, 2.6, 0.5):
                for y in np.arange(0, 5.1, 0.5):
                    scan_points.append((x, y, 2.0))
        else:
            for x in np.arange(3, 5.1, 0.5):
                for y in np.arange(0, 5.1, 0.5):
                    scan_points.append((x, y, 2.0))

        for x, y, z in scan_points:
            try:
                self.drone.goto_from_outside(x, y, z, 0)
                print(f"Scout {self.id}: Лечу к точке ({x}, {y}, {z})")
                while not self.drone.point_reached:
                    qr_info = self.detect_qr()
                    if qr_info:
                        self.qr_found = True
                        qr_locations.put(qr_info)
                        print(f"Scout {self.id}: QR-код найден, завершаю сканирование")
                        break
                    time.sleep(0.5)
                if self.qr_found:
                    break
            except Exception as e:
                print(f"Scout {self.id}: Ошибка при полете: {e}")
                break

        # Возвращение на стартовую точку