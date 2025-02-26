from typing import List, Tuple, Dict, Optional
from pion import Pion
import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import math
# Импорт необходимых функций из модуля pion.functions
from pion.functions import vector_reached, update_array

# ------------------ Вспомогательные функции ------------------

def calculate_shift_global(points: np.ndarray, 
                           frame_center: Tuple[int, int],
                           yaw: float,
                           altitude: float) -> List[float]:
    """
    Вычисляет смещение QR-кода относительно центра кадра.

    :param points: Координаты вершин найденного QR-кода.
    :type points: np.ndarray
    :param frame_center: Центр кадра (ширина, высота).
    :type frame_center: Tuple[int, int]
    :param yaw: Текущий угол поворота дрона (в радианах).
    :type yaw: float
    :param altitude: Текущая высота дрона.
    :type altitude: float
    :return: Список с корректированными смещениями [x, y].
    :rtype: List[float]
    """
    qr_center = np.mean(points, axis=0)
    shift_x_px = qr_center[0] - frame_center[0]
    shift_y_px = qr_center[1] - frame_center[1]
    shift_x_m = (shift_x_px * altitude) / 700  # 700 – фокальная длина
    shift_y_m = (shift_y_px * altitude) / 700
    # Преобразование с учетом угла yaw:
    corrected_x = shift_x_m * math.cos(yaw) - shift_y_m * math.sin(yaw)
    corrected_y = shift_x_m * math.sin(yaw) + shift_y_m * math.cos(yaw)
    return [corrected_x, corrected_y]

def detect_qr_global(drone: Pion,
                     cap: cv2.VideoCapture, 
                     finished_targets: List[str],
                     frame_center: Tuple[int, int], 
                     coordinates_or_error: bool = True
                    ) -> Tuple[Dict[str, np.ndarray], np.ndarray]:
    """
    Считывает кадр из видеопотока, ищет QR-коды и вычисляет error-вектор. Если coordinates_or_error=True,
    возвращается вектор смещения относительно дрона; иначе вектор суммируется с координатами дрона для получения
    глобальных координат.

    :param drone: Объект дрона.
    :type drone: Pion
    :param cap: Объект VideoCapture.
    :type cap: cv2.VideoCapture
    :param finished_targets: Список уже обработанных QR-кодов.
    :type finished_targets: List[str]
    :param frame_center: Центр кадра (ширина, высота).
    :type frame_center: Tuple[int, int]
    :param coordinates_or_error: Флаг выбора типа возвращаемых координат.
    :type coordinates_or_error: bool
    :return: Кортеж (словарь обнаруженных QR, считанный кадр).
    :rtype: Tuple[Dict[str, np.ndarray], np.ndarray]
    """
    with drone._handler_lock:
        ret, frame = cap.read()
        if not ret:
            print("Не удалось получить кадр")
            return {}, frame

    # Уменьшение разрешения кадра для повышения производительности
    frame = cv2.resize(frame, (640, 480))

    key_errors: Dict[str, np.ndarray] = {}
    try:
        data = decode(frame)
        if data:
            print(f"Обработка кадра: найдено {len(data)} QR-кодов")
            for item in data:
                decoded_key = item[0].decode()
                if decoded_key not in finished_targets:
                    drone.led_control(255, 0, 255, 0)  # Мигание светодиодами
                    points = np.array(item[3])
                    shift = calculate_shift_global(points, frame_center, drone.yaw, drone.position[2])
                    if shift:
                        if coordinates_or_error:
                            error = np.array([-shift[0], shift[1], 0, 0])
                        else:
                            error = np.array([-shift[0] + drone.xyz[0],
                                              shift[1] + drone.xyz[1], 0, 0])
                        key_errors[decoded_key] = error
                        print(f"Обнаружен: {decoded_key} = {error}")
                    drone.led_control(255, 0, 0, 0)  # Выключение светодиодов
    except Exception as e:
        print(f"Ошибка при декодировании QR-кода: {e}")
        return {}, frame

    return key_errors, frame

def move_to_target(drone: Pion,
                   cap: cv2.VideoCapture,
                   frame_center: Tuple[int, int],
                   finished_targets: List[str],
                   show: bool = False,
                   key: str = '4',
                   scaling_factor: float = 0.5, 
                   threshold: float = 0.05,
                   time_break: float = float('inf')
                   ) -> Tuple[List[str], np.ndarray]:
    """
    Корректирует позицию дрона с помощью видеопотока до достижения заданной точности для указанного QR-кода.

    :param drone: Объект дрона.
    :type drone: Pion
    :param cap: Объект VideoCapture.
    :type cap: cv2.VideoCapture
    :param frame_center: Центр кадра (ширина, высота).
    :type frame_center: Tuple[int, int]
    :param finished_targets: Список уже обработанных QR-кодов.
    :type finished_targets: List[str]
    :param show: Флаг отображения видеопотока.
    :type show: bool
    :param key: Ключ (название) QR-кода, по которому корректируется позиция.
    :type key: str
    :param scaling_factor: Коэффициент масштабирования корректирующей скорости.
    :type scaling_factor: float
    :param threshold: Порог точности для завершения корректировки (в метрах).
    :type threshold: float
    :param time_break: Максимальное время работы корректировки.
    :type time_break: float
    :return: Кортеж (обновлённый список finished_targets, конечные координаты дрона).
    :rtype: Tuple[List[str], np.ndarray]
    """
    errors = np.zeros((10, 4))
    flag_reach_zero_error = False
    drone.speed_flag = False
    t_0 = time.time()
    while not flag_reach_zero_error:
        if time.time() - t_0 > time_break:
            break
        key_errors, frame = detect_qr_global(drone, cap, finished_targets, frame_center)
        print("key_errors =", key_errors)
        if key in key_errors:
            if show:
                cv2.imshow('Delivery stream', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            errors = update_array(errors, key_errors[key])
            flag_reach_zero_error = vector_reached(np.zeros(4), errors, accuracy=threshold)
            adjusted_speed = key_errors[key] * scaling_factor
            print("Отправляем скорость:", *adjusted_speed, f"\nxyz: {drone.xyz}")
            drone.send_speed(*adjusted_speed)
    drone.t_speed = np.zeros(4)
    finished_targets.append(key)
    final_coordinate = drone.xyz.copy()
    return finished_targets, final_coordinate

# ------------------ Класс DroneScanner ------------------

class DroneScanner:
    """
    Класс для сканирования окружающей среды дроном с целью обнаружения QR-кодов.
    После выполнения миссии сканирования результатом является словарь обнаруженных QR-кодов,
    где для каждого кода усреднены координаты (на основе накопленных error-векторов).
    """
    FOCAL_LENGTH: int = 700

    def __init__(self, drone: Pion, base_coords: np.ndarray, scan_points: np.ndarray, show: bool = False) -> None:
        """
        Инициализирует дрона-сканер.

        :param drone: Объект дрона.
        :type drone: Pion
        :param base_coords: Координаты базы (точка возврата).
        :type base_coords: np.ndarray
        :param scan_points: Массив точек для обхода (каждая точка – кортеж (x, y, z, yaw)).
        :type scan_points: np.ndarray
        :param show: Флаг отображения видеопотока.
        :type show: bool
        :return: None
        """
        self.show = show
        self.drone: Pion = drone
        self.base_coords: np.ndarray = base_coords
        self.scan_points: np.ndarray = scan_points
        self.unique_points: Dict[str, List[np.ndarray]] = {}  # Накопление QR-кодов
        self.rtsp_url: str = f'rtsp://{self.drone.ip}:8554/front'
        self.cap: cv2.VideoCapture = cv2.VideoCapture(self.rtsp_url)
        self.initialize_drone()

    def initialize_drone(self) -> None:
        """
        Инициализирует дрона: выполняет арминг, взлёт и устанавливает режим скорости.
        
        :return: None
        """
        time.sleep(1)
        self.smart_take_off()
        time.sleep(7)
        self.drone.set_v()

    def smart_take_off(self) -> None:
        """
        Функция для принудительного взлета.
        Использовать с осторожностью!!!
        """
        print("Smart take off is beginning")
        while self.drone.xyz[2] < 0.3:
            time.sleep(1)
            self.drone.arm()
            time.sleep(0.5)
            self.drone.takeoff()
        time.sleep(7)
        print("Smart take off is ending")

    def detect_qr(self,
                  cap: cv2.VideoCapture, 
                  frame_center: Tuple[int, int],
                  finished_targets: Optional[List[str]] = None,
                  coordinates_or_error: bool = True
                 ) -> Tuple[Dict[str, np.ndarray], np.ndarray]:
        """
        Обёртка для функции detect_qr_global.

        :param cap: Объект VideoCapture.
        :type cap: cv2.VideoCapture
        :param frame_center: Центр кадра.
        :type frame_center: Tuple[int, int]
        :param finished_targets: Список уже обработанных QR-кодов.
        :type finished_targets: Optional[List[str]]
        :param coordinates_or_error: Флаг выбора типа возвращаемых координат.
        :type coordinates_or_error: bool
        :return: Кортеж (словарь обнаруженных QR, считанный кадр).
        :rtype: Tuple[Dict[str, np.ndarray], np.ndarray]
        """
        return detect_qr_global(self.drone, cap, finished_targets or [], frame_center, coordinates_or_error)

    def process_mission_point(self,
                              target_point: Tuple[float, float],
                              show: bool = False
                              ) -> None:
        """
        Перемещает дрона-сканер к заданной точке и собирает обнаруженные QR-коды.

        :param target_point: Целевая точка (x, y) для обхода.
        :type target_point: Tuple[float, float]
        :param show: Флаг отображения видеопотока.
        :type show: bool
        :return: None
        """
        finished_targets = list(self.unique_points.keys())
        frame_center = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2,
                        int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) // 2)
        self.drone.speed_flag = False
        while True:
            if self.drone.xyz[1] > target_point[1]:
                break
            vector_speed = (np.array(target_point) - self.drone.xyz[:2])
            vector_length = np.linalg.norm(vector_speed)
            if vector_length > 0:
                vector_speed = vector_speed / vector_length * 0.1
                self.drone.send_speed(vector_speed[0], vector_speed[1], 0, 0)

            # Вызов функции detect_qr для обнаружения QR-кодов
            key_errors, frame = self.detect_qr(self.cap, frame_center, finished_targets, coordinates_or_error=False)

            for key, error in key_errors.items():
                self.unique_points.setdefault(key, []).append(error)
                finished_targets.append(key)

            if self.show:
                cv2.imshow(f'Drone Scanner {self.drone.ip}', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.drone.speed_flag = False

    def execute_scan(self) -> Dict[str, np.ndarray]:
        """
        Выполняет миссию сканирования: дрон перемещается по заданным точкам,
        собирает QR-коды и возвращается на базу. Итоговые координаты для каждого QR усредняются.

        :return: Словарь обнаруженных QR-кодов с усредненными координатами.
        :rtype: Dict[str, np.ndarray]
        """
        for point in self.scan_points:
            print(f"Перемещение к точке: {point}")
            self.drone.goto_from_outside(*point)
            self.process_mission_point(point[:2], show=self.show)

        # Возврат на базу
        self.return_to_base()

        # Усреднение координат для каждого QR-кода
        averaged_coords = {}
        for key, errors in self.unique_points.items():
            averaged_coords[key] = np.mean(errors, axis=0)

        return averaged_coords
    def return_to_base(self) -> None:
        """
        Возвращает дрона-сканер на базу.
        
        :return: None
        """
        print(f"\n\nReturn to base: {self.base_coords}\n\n")
        self.drone.set_v()
        self.drone.goto_from_outside(*self.base_coords[0:2], 0.7, 0)
        self.drone.speed_flag = False
        self.drone.land()
        time.sleep(20)

    @property
    def scanned_qr(self) -> Dict[str, np.ndarray]:
        """
        Возвращает усреднённые координаты всех обнаруженных QR-кодов.

        :return: Словарь, где ключ – код, значение – усреднённая координата.
        :rtype: Dict[str, np.ndarray]
        """
        averaged_coords = {}
        for key, errors in self.unique_points.items():
            averaged_coords[key] = np.mean(errors, axis=0)
        return averaged_coords


# ------------------ Класс DroneDeliverer ------------------

class DroneDeliverer:
    """
    Класс для доставки. Дрон-доставщик получает словарь QR-кодов с координатами,
    после чего для каждой найденной цели выполняется корректировка позиции с помощью камеры,
    приземление с уточнением и возврат на базу.
    """
    def __init__(self,
                 drone: Pion, 
                 base_coords: np.ndarray,
                 delivery_points: List[Tuple[float, float, float, float]],
                 mission_keys: Optional[List[str]] = None,
                 show: bool = False) -> None:
        """
        Инициализирует дрона-доставщика.

        :param drone: Объект дрона для доставки.
        :type drone: Pion
        :param base_coords: Координаты базы (точка возврата).
        :type base_coords: np.ndarray
        :param delivery_points: Список точек для кандидатов (x, y, z, yaw).
        :type delivery_points: List[Tuple[float, float, float, float]]
        :param mission_keys: Список QR-кодов, для которых необходимо выполнить доставку.
        :type mission_keys: Optional[List[str]]
        :return: None
        """
        self.drone: Pion = drone
        self.base_coords: np.ndarray = base_coords
        self.show = show
        self.delivery_points: List[Tuple[float, float, float, float]] = delivery_points
        self.mission_keys: List[str] = mission_keys if mission_keys is not None else []
        self.initialize_drone()

    def initialize_drone(self) -> None:
        """
        Инициализирует дрона-доставщика: арминг, взлёт и установка скорости.
        
        :return: None
        """
        time.sleep(1)
        self.smart_take_off()
        self.drone.set_v()

    def smart_take_off(self) -> None:
        """
        Функция для принудительного взлета.
        Использовать с осторожностью!!!
        """
        print("Smart take off is beginning")
        while self.drone.xyz[2] < 0.3:
            time.sleep(1)
            self.drone.arm()
            time.sleep(0.5)
            self.drone.takeoff()
        time.sleep(7)
        print("Smart take off is ending")

    def deliver_to_target(self, target_key: str,
                          target_coord: np.ndarray,
                          scaling_factor: float = 0.5,
                          threshold: float = 0.05,
                          dict_of_points_to_return: Optional[Dict[str, np.ndarray]] = None
                          ) -> np.ndarray:
        """
        Выполняет доставку для одного QR-кода. В качестве основной candidate точки используется
        найденная координата (target_coord) из сканирования. Если корректировка не удается,
        дополнительно перебираются точки из delivery_points. После корректировки дрон возвращается на базу.

        :param target_key: Имя QR-кода (цель).
        :type target_key: str
        :param target_coord: Исходная (усреднённая) координата цели, полученная сканером.
        :type target_coord: np.ndarray
        :param scaling_factor: Коэффициент масштабирования скорости корректировки.
        :type scaling_factor: float
        :param threshold: Порог точности (в метрах) для завершения корректировки.
        :type threshold: float
        :param dict_of_points_to_return: Опциональный словарь с точками возврата для цели.
        :type dict_of_points_to_return: Optional[Dict[str, np.ndarray]]
        :return: Финальная координата дрона после корректировки.
        :rtype: np.ndarray
        """
        final_coord: Optional[np.ndarray] = None
        # Формируем список candidate точек: сначала используем найденную координату (с принудительной высотой 1.7),
        # затем перебираем дополнительные точки из delivery_points.
        candidate_points: List[Tuple[float, float, float, float]] = [(target_coord[0], target_coord[1], 1.7, 0)]
        candidate_points.extend(self.delivery_points)
        for point in candidate_points:
            if self.drone.xyz[2] < 0.3:
                print("Взлет")
                self.smart_take_off()
            print(f"Попытка доставки для '{target_key}' с candidate точкой {point}")
            self.drone.speed_flag = True
            self.drone.goto_from_outside(*point)
            self.drone.speed_flag = False
            time.sleep(2)
            rtsp_url = f'rtsp://{self.drone.ip}:8554/front'
            cap = cv2.VideoCapture(rtsp_url)
            if not cap.isOpened():
                print(f"Не удалось открыть видеопоток для '{target_key}' на точке {point}")
                continue
            frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print("Размеры видеопотока:", frame_height, frame_width)
            frame_center = (frame_width // 2, frame_height // 2)
            finished_targets: List[str] = []

            # Вызов функции move_to_target для корректировки позиции
            finished_targets, final_coord = move_to_target(self.drone, cap, frame_center, finished_targets,
                                                           show=self.show, key=target_key,
                                                           scaling_factor=scaling_factor, threshold=threshold)

            cap.release()
            cv2.destroyAllWindows()
            print(f"Для '{target_key}' получены координаты: {final_coord}")
            self.drone.land()
            time.sleep(30)
            self.smart_take_off()
            if final_coord is not None:
                break
        if final_coord is None:
            print(f"Не удалось уточнить позицию для '{target_key}'")
            final_coord = target_coord
        if dict_of_points_to_return:
            if target_key in dict_of_points_to_return:
                self.return_to_point(dict_of_points_to_return[target_key])
        else:
            self.return_to_base()
        return final_coord

    def deliver_all(self,
                    targets: Dict[str, np.ndarray],
                    coordinates_of_bases: Dict[str, np.ndarray]
                    ) -> Dict[str, np.ndarray]:
        """
        Для каждого QR-кода из словаря целей выполняется доставка:
        дрон уточняет позицию, приземляется с корректировкой и возвращается на базу.
        
        :param targets: Словарь QR-кодов с исходными координатами, полученными сканером.
        :type targets: Dict[str, np.ndarray]
        :param coordinates_of_bases: Словарь с точками возврата для каждой цели.
        :type coordinates_of_bases: Dict[str, np.ndarray]
        :return: Словарь доставленных координат для каждого QR-кода.
        :rtype: Dict[str, np.ndarray]
        """
        delivered: Dict[str, np.ndarray] = {}
        for key, coord in targets.items():
            print(f"Запуск доставки для цели '{key}' с координатами {coord}")
            final_coord = self.deliver_to_target(key, coord, dict_of_points_to_return=coordinates_of_bases)
            delivered[key] = final_coord
        print("Доставка завершена. Доставленные координаты:", delivered)
        return delivered

    def return_to_base(self) -> None:
        """
        Возвращает дрона-доставщика на базу.
        
        :return: None
        """
        print(f"\n\nReturn to base: {self.base_coords}\n\n")
        self.smart_take_off()
        self.drone.goto_from_outside(*self.base_coords)
        self.drone.speed_flag = False
        self.drone.land()
        time.sleep(20)

    def return_to_point(self, point: np.ndarray) -> None:
        """
        Возвращает дрона-доставщика на заданную точку.
        
        :param point: Таргетная точка возврата (x, y, z, yaw).
        :type point: np.ndarray
        :return: None
        """
        print(f"\n\nReturn to point {point}\n\n")
        self.drone.goto_from_outside(*point)
        self.drone.speed_flag = False
        self.drone.land()
        time.sleep(20)


