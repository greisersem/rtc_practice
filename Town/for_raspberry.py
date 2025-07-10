import time
import serial
import RPi.GPIO as GPIO
from tkinter import Tk, Label, Button, StringVar


class TrafficLight:
    def __init__(self, id: int) -> None:                                                                              
        self.id = id
    
    def change_light(self, light_id: int) -> bytes:
        return f"{self.id}{light_id}".encode()


class Barrier:
    open_id = 1
    close_id = 0

    def __init__(self, id: int) -> None:
        self.id = id

    def open(self) -> bytes:
        return f"{self.id}{Barrier.open_id}".encode()

    def close(self) -> bytes:
        return f"{self.id}{Barrier.close_id}".encode()
       

class RoadSign:
    def __init__(self, id: int) -> None:
        self.id = id
    
    def rotate(self) -> bytes:
        return str(self.id).encode()


class RS485Controller:
    def __init__(self, serial_obj, tx_rx_pin):
        self.ser = serial_obj
        self.tx_rx_pin = tx_rx_pin
        self.last_response = ""
        self.buffer = bytearray()  # Буфер для накопления данных

    def set_send(self):
        GPIO.output(self.tx_rx_pin, GPIO.HIGH)
        time.sleep(0.05)

    def set_receive(self):
        GPIO.output(self.tx_rx_pin, GPIO.LOW)
        time.sleep(0.05)

    def send_data(self, data: bytes):
        self.set_send()
        self.ser.write(data)
        self.ser.flush()
        self.set_receive()

    def check_response(self):
        """Проверяет наличие входящих данных и обрабатывает построчно"""
        # Читаем все доступные данные
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)
        
        # Ищем завершающий символ новой строки
        while b'\n' in self.buffer:
            line_end = self.buffer.index(b'\n')
            line = self.buffer[:line_end]
            del self.buffer[:line_end + 1]  # Удаляем обработанную строку
            
            try:
                decoded = line.decode('utf-8').strip()
                if decoded:  # Игнорируем пустые строки
                    self.last_response = decoded
                    print(f"[RECV] {decoded}")
            except UnicodeDecodeError:
                print(f"[RECV] Raw data: {line.hex()}")
        
        return self.last_response


def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7, GPIO.OUT, initial=GPIO.LOW)

    ser = serial.Serial(
        port='/dev/serial0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
    )

    rs485 = RS485Controller(ser, 7)
    
    root = Tk()
    root.title("Управление городом")
    root.geometry("400x400")
    
    status_var = StringVar()
    status_var.set("Ожидание команд...")
    status_label = Label(root, textvariable=status_var, font=("Arial", 12))
    status_label.pack(pady=10)
    
    def update_status():
        rs485.check_response()
        if rs485.last_response:
            status_var.set(f"Последний ответ: {rs485.last_response}")
        root.after(100, update_status)
    
    barrier1 = Barrier(2)
    light1 = TrafficLight(1)
    sign1 = RoadSign(3)
    
    Label(root, text="Шлагбаум").pack()
    Button(root, text="Открыть", command=lambda: send_command(barrier1.open())).pack()
    Button(root, text="Закрыть", command=lambda: send_command(barrier1.close())).pack()

    Label(root, text="\nСветофор").pack()
    Button(root, text="Зелёный", command=lambda: send_command(light1.change_light(0))).pack()
    Button(root, text="Жёлтый", command=lambda: send_command(light1.change_light(1))).pack()
    Button(root, text="Красный", command=lambda: send_command(light1.change_light(2))).pack()

    Label(root, text="\nДорожный знак").pack()
    Button(root, text="Повернуть", command=lambda: send_command(sign1.rotate())).pack()
    
    def send_command(data: bytes):
        rs485.send_data(data)
        status_var.set("Команда отправлена...")
        # Очищаем предыдущий ответ при отправке новой команды
        rs485.last_response = ""
    
    def on_closing():
        ser.close()
        GPIO.cleanup()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.after(100, update_status)
    root.mainloop()


if __name__ == "__main__":
    main()
