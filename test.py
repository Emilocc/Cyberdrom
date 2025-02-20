from typing import List, Tuple, Dict, Optional
import sys
from typing import List, Dict
from pion import Pion
import numpy as np
from rzd import *
import threading

# Общий массив для хранения результатов сканирования
scanned_results: Dict[str, np.ndarray] = {}
scanned_results_lock = threading.Lock()  # Для синхронизации доступа к общему массиву


def scan_with_drone(drone: Pion, base_coords: np.ndarray, scan_points: np.ndarray, drone_name: str) -> None:
    """
    Функция для сканирования QR-кодов с использованием дрона.
    Результаты сохраняются в общий массив scanned_results.
    """
    scanner = DroneScanner(drone=drone, base_coords=base_coords, scan_points=scan_points, show=True)
    results = scanner.execute_scan()

    # Блокируем доступ к общему массиву для записи результатов
    with scanned_results_lock:
        scanned_results.update(results)

    print(f"{drone_name} завершил сканирование. Результаты добавлены в общий массив.")


def main() -> None:
    """
    Основная функция. Создает два дрона-сканера, которые параллельно сканируют QR-коды.
    После завершения сканирования результаты передаются дрону-доставщику.
    """
    # Параметры для первого дрона-сканера
    scout_drone_1 = Pion(
        ip="127.0.0.1",
        mavlink_port=8001,
        logger=False,
        dt=0.0,
        accuracy=0.08,
        count_of_checking_points=5
    )
    base_coords_1 = np.array([0, 0, 0, 0])
    scan_points_1 = np.array([
        [2.74, -3.04, 2, 0],
        [4.72, 2.40, 2, 0],
        [0, 0, 0, 0]
    ])

    # Параметры для второго дрона-сканера
    scout_drone_2 = Pion(
        ip="127.0.0.1",
        mavlink_port=8002,
        logger=False,
        dt=0.0,
        accuracy=0.08,
        count_of_checking_points=5
    )
    base_coords_2 = np.array([0, 0, 0, 0])
    scan_points_2 = np.array([
        [4.72, 2.40, 2, 0],
        [-5, -2, 2, 0],
        [0, 0, 0, 0]
    ])

    # Создаем и запускаем потоки для сканирования
    thread_1 = threading.Thread(target=scan_with_drone, args=(scout_drone_1, base_coords_1, scan_points_1, "Дрон 1"))
    thread_2 = threading.Thread(target=scan_with_drone, args=(scout_drone_2, base_coords_2, scan_points_2, "Дрон 2"))

    thread_1.start()
    thread_2.start()

    # Ожидаем завершения работы обоих дронов
    thread_1.join()
    thread_2.join()

    print("Результаты сканирования (общий массив):", scanned_results)

    # Список целевых QR-кодов, которые необходимо найти
    targets: List[str] = ["Box 2 2", "Box 2 1"]

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
    delivered_results = deliverer.deliver_all(matched_targets, coordinates_of_bases={})
    print("Итоговые доставленные координаты:", delivered_results)
    deliverer.return_to_base()


if __name__ == "__main__":
    main()