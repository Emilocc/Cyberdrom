
# для Cubesat ( погода)
import os
import numpy as np
import cv2
import requests


def check_weather_square(image_path):
    # Считываем изображение
    image_data = np.fromfile(image_path, dtype=np.uint8)
    image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

    if image is None:
        raise ValueError("Изображение не загружено. Проверьте путь к файлу.")

    height, width, _ = image.shape
    square_size = 10
    x_start = width - square_size
    y_start = height - square_size
    center_x = x_start + square_size // 2
    center_y = y_start + square_size // 2
    pixel_color = image[center_y, center_x]

    # BGR, а не RGB
    b, g, r = pixel_color

    # Определяем состояние погоды на основе цвета
    if (r, g, b) == (0, 255, 0):  # Зеленый
        return "go"
    elif (r, g, b) == (0, 255, 255):  # Желтый
        return "warning"
    elif (r, g, b) == (255, 0, 0):  # Красный
        return "stop"
    else:
        return "unknown"


def get_weather_from_server(ip):
    url = f"http://{ip}:8003/get_weather"
    response = requests.get(url)

    if response.status_code == 200:
        return response.text
    else:
        raise ValueError("Ошибка при получении данных погоды с сервера.")


def main():
    image_path = r"Красный #ff0000.png"  # Укажите путь к вашему изображению
    ip = "192.168.1.1"

    # Проверяем состояние погоды по цвету квадрата
    square_weather = check_weather_square(image_path)
    print(f"Состояние погоды квадрат: {square_weather}")

    # Получаем состояние погоды с сервера
    try:
        server_weather = get_weather_from_server(ip)
        print(f"Состояние погоды с сервера: {server_weather}")

        # Логика на основе состояния
        if server_weather == "go":
            print("Разрешен полет дронов.")
        elif server_weather == "warning":
            print("Необходимо вернуть все БВС на взлетные площадки.")
        elif server_weather == "stop":
            print("Все БВС должны находиться на взлетных площадках.")
        else:
            print("Неизвестное состояние погоды.")

    except ValueError as e:
        print(e)


if __name__ == "__main__":
    main()

# QR-коды
import cv2
from pyzbar import pyzbar


def process_qr_code(data):
    if data == "Груз БПЛА":
        print("Обнаружен QR-код для погрузки БПЛА")
        # Здесь можно вызвать функцию
        # load_cargo()
    else:
        print(f"Обнаружен QR-код: {data}")

# Функция для распознавания QR-кодов в реальном времени
def detect_qr_code():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        qr_codes = pyzbar.decode(gray)


        for qr_code in qr_codes:
            data = qr_code.data.decode("utf-8")
            process_qr_code(data)


            (x, y, w, h) = qr_code.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)


            cv2.putText(frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        cv2.imshow("QR Code Scanner", frame)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()


detect_qr_code()

#Aruco метки
import cv2
import cv2.aruco as aruco

# Функция для обработки ArUco-метки
def process_aruco_marker(marker_id):
    if marker_id == 0:  # Предположим, что ID метки "Груз РТС" равен 0
        print("Обнаружена ArUco-метка для погрузки РТС")
        # Здесь можно вызвать функцию для погрузки
        # load_cargo()
    else:
        print(f"Обнаружена ArUco-метка с ID: {marker_id}")

# Функция для распознавания ArUco-меток в реальном времени
def detect_aruco_markers():
    # Захват видеопотока с камеры
    cap = cv2.VideoCapture(0)

    # Загрузка словаря ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    # Создаем детектор ArUco
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    while True:
        # Чтение кадра с камеры
        ret, frame = cap.read()

        # Поиск ArUco-меток
        corners, ids, _ = detector.detectMarkers(frame)

        # Если метки найдены
        if ids is not None:
            # Перебираем все найденные метки
            for i in range(len(ids)):
                # Получаем углы текущей метки
                marker_corners = corners[i][0]

                # Отрисовка квадрата вокруг метки
                for j in range(4):
                    # Соединяем углы линиями
                    start_point = tuple(map(int, marker_corners[j]))
                    end_point = tuple(map(int, marker_corners[(j + 1) % 4]))
                    cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

                # Отрисовка ID метки
                marker_id = ids[i][0]
                cv2.putText(frame, str(marker_id), tuple(map(int, marker_corners[0])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Обработка метки
                process_aruco_marker(marker_id)

        # Отображение кадра с распознанными метками
        cv2.imshow("ArUco Marker Detection", frame)

        # Выход из цикла по нажатию клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Освобождение ресурсов
    cap.release()
    cv2.destroyAllWindows()

# Запуск функции распознавания ArUco-меток
detect_aruco_markers()