import cv2
import csv
from pyzbar import pyzbar
from datetime import datetime
from rzd import *  # Ensure you have the appropriate import for the SocketCamera


# Функция для обработки QR-кода
def process_qr_code(data):
    if data == "Груз БПЛА":
        print("Обнаружен QR-код для погрузки БПЛА")
        # Здесь можно вызвать функцию для погрузки
        # load_cargo()
    else:
        print(f"Обнаружен QR-код: {data}")
    # Сохраняем данные QR-кода в CSV
    save_qr_data_to_csv(data)


# Функция для записи данных QR-кода в CSV файл
def save_qr_data_to_csv(data, filename="qr_data.csv"):
    # Открываем файл в режиме добавления
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        # Записываем данные и текущее время
        writer.writerow([data, datetime.now().strftime("%Y-%m-%d %H:%M:%S")])


# Основная функция
def main():
    # Используем камеру с IP и портом, соответствующими симулятору
    camera = SocketCamera(ip="127.0.0.1", port=18001)

    while True:
        # Получаем кадр с камеры
        frame = camera.get_cv_frame()
        if frame is None:
            print("Не удалось получить кадр с камеры.")
            continue

        # Преобразуем кадр в оттенки серого для распознавания QR-кодов
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Распознаем QR-коды
        qr_codes = pyzbar.decode(gray)

        # Обрабатываем каждый обнаруженный QR-код
        for qr_code in qr_codes:
            data = qr_code.data.decode("utf-8")
            process_qr_code(data)

            # Рисуем прямоугольник вокруг QR-кода
            (x, y, w, h) = qr_code.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Добавляем текст с данными QR-кода
            cv2.putText(frame, data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Отображаем кадр с выделенными QR-кодами
        cv2.imshow("Socket Camera with QR Code Detection", frame)

        # Выход по нажатию ESC
        if cv2.waitKey(1) == 27:
            break

    # Закрываем окна и освобождаем ресурсы
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


