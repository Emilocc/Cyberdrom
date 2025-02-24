from typing import List, Tuple, Dict, Optional
from pion import Pion  # Предполагается, что это импортируется из библиотеки pion
import numpy as np
import cv2
from rzd import *  # Предполагается, что это содержит SocketCamera или аналог
import sys
import time

# Класс для сканирования QR-кодов
class DroneScanner:
    def __init__(self, drone: Pion, base_coords: np.ndarray, scan_points: np.ndarray, show: bool = True, camera_port: int = 554):
        self.drone = drone
        self.base_coords = base_coords
        self.scan_points = scan_points
        self.show = show
        self.unique_points = {}  # Словарь для хранения координат QR-кодов
        self.cap = cv2.VideoCapture(f"rtsp://{drone.ip}:{camera_port}/stream")  # RTSP-поток для камеры

    def smart_take_off(self) -> None:
        print("Smart take off is beginning")
        retries = 3
        while retries > 0:
            try:
                while self.drone.xyz[2] < 0.3:
                    time.sleep(1)
                    self.drone.arm()
                    time.sleep(0.5)
                    self.drone.takeoff()
                print("Smart take off is ending")
                break
            except ConnectionResetError:
                print(f"Connection reset. Retrying... ({retries} attempts left)")
                retries -= 1
                time.sleep(2)
        if retries == 0:
            raise Exception("Failed to complete takeoff due to connection issues.")

    def detect_qr(self, cap, frame_center, finished_targets, coordinates_or_error=False):
        ret, frame = cap.read()
        if not ret:
            print("Не удалось получить кадр с камеры")
            return {}, np.array([])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qr_codes = pyzbar.decode(gray)
        key_errors = {}
        for qr_code in qr_codes:
            data = qr_code.data.decode("utf-8")
            if data in finished_targets:
                continue
            points = np.array(qr_code.polygon)
            qr_center = np.mean(points, axis=0)
            shift_x_px = qr_center[0] - frame_center[0]
            shift_y_px = qr_center[1] - frame_center[1]
            shift_x_m = (shift_x_px * self.drone.xyz[2]) / 700
            shift_y_m = (shift_y_px * self.drone.xyz[2]) / 700
            coords = np.array([self.drone.xyz[0] + shift_x_m, self.drone.xyz[1] + shift_y_m, self.drone.xyz[2]])
            key_errors[data] = coords
            print(f"Обнаружен QR-код: {data}, координаты: {coords}")

        return key_errors, frame

    def execute_scan(self) -> Dict[str, np.ndarray]:
        self.unique_points = {}
        frame_center = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2,
                        int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) // 2)

        self.smart_take_off()  # Выполняем взлет перед сканированием

        for point in self.scan_points:
            print(f"Перемещение к точке сканирования: {point}")
            self.drone.goto_from_outside(*point)
            time.sleep(5)

            if not self.cap.isOpened():
                print("RTSP stream is not available. Skipping scan at this point.")
                continue

            finished_targets = list(self.unique_points.keys())
            key_errors, frame = self.detect_qr(self.cap, frame_center, finished_targets, coordinates_or_error=False)

            # Skip if the frame is invalid
            if frame.size == 0:
                print("Invalid frame received. Skipping display.")
                continue

            for key, error in key_errors.items():
                self.unique_points.setdefault(key, []).append(error)

            if self.show:
                try:
                    cv2.imshow(f'Drone Scanner {self.drone.ip}', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                except cv2.error as e:
                    print(f"OpenCV error during imshow: {e}")

        self.return_to_base()

        averaged_coords = {}
        for key, errors in self.unique_points.items():
            averaged_coords[key] = np.mean(errors, axis=0)

        return averaged_coords

    def return_to_base(self):
        print("Возвращение на базу")
        self.drone.goto_from_outside(*self.base_coords)
        time.sleep(5)

# Класс для доставки грузов
class DroneDeliverer:
    def __init__(self, drone: Pion, base_coords: np.ndarray, delivery_points: List[Tuple[float, float, float, float]],
                 mission_keys: List[str], show: bool = True):
        self.drone = drone
        self.base_coords = base_coords
        self.delivery_points = delivery_points
        self.mission_keys = mission_keys
        self.show = show

    def smart_take_off(self) -> None:
        print("Smart take off is beginning")
        retries = 3
        while retries > 0:
            try:
                while self.drone.xyz[2] < 0.3:
                    time.sleep(1)
                    self.drone.arm()
                    time.sleep(0.5)
                    self.drone.takeoff()
                print("Smart take off is ending")
                break
            except ConnectionResetError:
                print(f"Connection reset. Retrying... ({retries} attempts left)")
                retries -= 1
                time.sleep(2)
        if retries == 0:
            raise Exception("Failed to complete takeoff due to connection issues.")

    def deliver_all(self, matched_targets: Dict[str, np.ndarray], coordinates_of_bases: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        delivered_results = {}
        self.smart_take_off()  # Выполняем взлет перед доставкой
        for key, coords in matched_targets.items():
            print(f"Доставка к цели: {key}, координаты: {coords}")
            self.drone.goto_from_outside(*coords)
            time.sleep(5)
            self.drone.land()
            time.sleep(3)
            self.drone.takeoff()
            delivered_results[key] = coords
        return delivered_results

    def return_to_base(self):
        print("Возвращение на базу")
        self.drone.goto_from_outside(*self.base_coords)
        time.sleep(5)

def main() -> None:
    """
    Основная функция. Сначала создается объект DroneScanner для сканирования QR-кодов.
    После сканирования результат (словарь QR-кодов с координатами) передается объекту DroneDeliverer,
    который выполняет доставку для каждой найденной цели.

    :return: None
    """
    # Параметры для дрона-сканера
    scout_drone = Pion(
        ip="127.0.0.1",
        mavlink_port=8001,
        logger=False,
        dt=0.0,
        accuracy=0.08,  # Точность позиционирования при goto_from_outside()
        count_of_checking_points=5  # Количество точек, которым проверяется отклонение accuracy
    )
    base_coords = np.array([2.78, -0.56, 0, 0])
    scan_points = np.array([
        [2.74, -3.04, 2, 0],
        [4.72, 2.40, 2, 0],
        [-4.07,-0.03, 2, 0]
    ])

    # Список целевых QR-кодов, которые необходимо найти
    targets: List[str] = ["Box 2 2", "Box 2 1"]
    # Точки возврата для каждой цели (например, где должен приземлиться дрон после доставки)
    coordinates_of_bases = {targets[0]: np.array([0, 0, 0, 0]), targets[1]: np.array([0, 0, 0, 0])}

    # Создаем сканирующий дрон и выполняем сканирование
    scanner = DroneScanner(drone=scout_drone, base_coords=base_coords, scan_points=scan_points, show=True)
    scanned_results = scanner.execute_scan()
    print("Результаты сканирования (усредненные координаты):", scanned_results)

    # Отбираем только те QR-коды, которые входят в список targets
    matched_targets = {k: v for k, v in scanned_results.items() if k in targets}
    if not matched_targets:
        print("Целевые QR-коды не обнаружены. Завершаем работу.")
        return

    # Параметры для дрона-доставщика
    delivery_drone = Pion(
        ip="10.1.100.211",
        mavlink_port=5656,
        logger=False,
        dt=0.0,
        accuracy=0.08,
        count_of_checking_points=5
    )
    delivery_points: List[Tuple[float, float, float, float]] = [
        (0, 0, 0, 0),
        (0, 0, 0, 0),
        (0, 0, 0, 0)
    ]

    # Создаем дрона-доставщика и выполняем доставку для найденных QR-кодов
    deliverer = DroneDeliverer(drone=delivery_drone,
                               base_coords=np.array([0, 0, 0, 0]),
                               delivery_points=delivery_points,
                               mission_keys=targets,
                               show=True)
    delivered_results = deliverer.deliver_all(matched_targets, coordinates_of_bases=coordinates_of_bases)
    print("Итоговые доставленные координаты:", delivered_results)
    deliverer.return_to_base()

try:
    main()
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    print("Cleaning up resources...")
    cv2.destroyAllWindows()
    sys.exit(0)

if __name__ == "__main__":
    main()