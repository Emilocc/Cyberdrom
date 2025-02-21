import cv2
from pyzbar import pyzbar
import csv
from datetime import datetime


# Функция для обработки QR-кода
def process_qr_code(data):
    if data == "Груз БПЛА":
        print("Обнаружен QR-код для погрузки БПЛА")
        # Здесь можно вызвать функцию для погрузки
        # load_cargo()
    else:
        print(f"Обнаружен QR-код: {data}")


# Функция для сохранения данных в CSV
def save_qr_data_to_csv(data, filename="qr_data.csv"):
    # Открываем файл в режиме добавления
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        # Записываем данные и текущее время
        writer.writerow([data, datetime.now().strftime("%Y-%m-%d %H:%M:%S")])


# Функция для распознавания QR-кодов в реальном времени
def detect_qr_code():
    cap = cv2.VideoCapture(0)  # Открываем камеру

    while True:
        ret, frame = cap.read()  # Получаем кадр с камеры
        if not ret:
            print("Не удалось получить кадр с камеры.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Преобразуем в оттенки серого

        qr_codes = pyzbar.decode(gray)  # Распознаем QR-коды

        # Обрабатываем каждый обнаруженный QR-код

        for qr_code in qr_codes:
            data = qr_code.data.decode("utf-8")  # Декодируем данные
            process_qr_code(data)  # Обрабатываем данные
            save_qr_data_to_csv(data)  # Сохраняем данные в CSV

            # Рисуем прямоугольник вокруг QR-кода
            (x, y, w, h) = qr_code.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Добавляем текст с данными QR-кода
            cv2.putText(frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Отображаем кадр с выделенными QR-кодами
        cv2.imshow("QR Code Scanner", frame)

        # Выход по нажатию 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Освобождаем ресурсы
    cap.release()
    cv2.destroyAllWindows()


# Запуск функции распознавания QR-кодов
detect_qr_code()
