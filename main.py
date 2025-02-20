from typing import List, Tuple, Dict, Optional
from pion import Pion
import numpy as np
from rzd import *
import sys



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
        accuracy=0.08, # Точность позиционирования при goto_from_outside()
        count_of_checking_points=5 # Количество точек, которым проверяется отклонение accuracy
    )
    base_coords = np.array([2.78,-0.56,0, 0])
    scan_points = np.array([
        [2.74,-3.04, 2, 0],
        [4.72,2.40 ,2 , 0],
        [0, 0, 0, 0]
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

def execute_scan(self) -> Dict[str, np.ndarray]:
    self.unique_points = {}
    frame_center = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) // 2,
                    int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) // 2)

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


