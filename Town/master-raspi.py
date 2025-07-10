import time
import serial
import RPi.GPIO as GPIO
from tkinter import Tk, Label, Button
import threading


class TrafficLight:
    """Класс, реализующий логику светофора
   
    Attributes
    ----------
    id : int 
        ID светофора в системе города

    Methods
    ----------
    change_light(light_id: int)
        Меняет свет светофора на указанный в id:
        0 - зелёный
        1 - жёлтый
        2 - красный 
    ----------
    """

    def __init__(self, id: int) -> None:                                                                              
        self.id = id
    
    def change_light(self, light_id: int) -> bytes:
        data = f"{self.id}{light_id}".encode()
        return data


class Barrier:
    """Класс, реализующий логику шлагбаума
   
    Attributes
    ----------
    id : int 
        ID шлагбаума в системе города

    Methods
    ----------
    open()
        Открыть шлагбаум — выставить 0 градусов на сервоприводе
    
    close()
        Закрыть шлагбаум — выставить 90 градусов на сервоприводе
    ----------
    """

    open_id = 1
    close_id = 0

    def __init__(self, id: int) -> None:
        self.id = id

    def open(self) -> bytes:
        data = f"{self.id}{Barrier.open_id}".encode()
        return data

    def close(self) -> bytes:
        data = f"{self.id}{Barrier.close_id}".encode()
        return data
       

class RoadSign:
    """Класс, реализующий логику дорожного знака
    
    Attributes
    ----------
    id : int 
        ID знака в системе города
    ----------

    Methods
    ----------
    rotate()
        Повернуть знак на 180 градусов
    ----------
    """

    def __init__(self, id: int) -> None:
        self.id = id
    
    def rotate(self) -> bytes:
        data = str(self.id).encode()
        return data


class RS485Controller:
    """Класс для управления платой MAX485
    
    Attributes
    ----------
    ser : serial_object 
        Настройки порта serial
    
    tx_rx_pin : int
        Порт переключения платы в режимы приема и отправки
    ----------

    Methods
    ----------
    set_send()
        Установка в режим отправки

    set_receive()
        Установка в режим получения

    send_data(data: bytes)
        Функция для отправки данных

    receive_loop()
        Цикл для обработки обратной связи
    ----------
    """



    def __init__(self, serial_obj, tx_rx_pin):
        self.ser = serial_obj
        self.tx_rx_pin = tx_rx_pin 

    def set_send(self):
        GPIO.output(self.tx_rx_pin, GPIO.HIGH)
        time.sleep(0.2)

    def set_receive(self):
        GPIO.output(self.tx_rx_pin, GPIO.LOW)
        time.sleep(0.2) 

    def send_data(self, data: bytes):
        self.set_send()  
        self.ser.write(data)
        self.ser.flush() 
        self.set_receive()

    def receive_loop(self):
        while True:
            try:
                if self.ser.in_waiting >= 2:
                    incoming = self.ser.readline().decode(errors="replace").strip()
                    if incoming:
                        print(f"[RECV] {incoming}")
            except Exception as e:
                print(f"[ERROR]" {e})
            time.sleep(0.1)
       

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7, GPIO.OUT, initial=GPIO.HIGH)

    ser = serial.Serial(
        port='/dev/serial0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )

    rs485 = RS485Controller(ser, 7)

    recv_thread = threading.Thread(target=rs485.receive_loop, daemon=True)
    recv_thread.start()
    
    root = Tk()
    root.title("Управление городом")
    root.geometry("400x400")

    barrier1 = Barrier(2)
    light1 = TrafficLight(1)
    sign1 = RoadSign(3)

    Label(root, text="Шлагбаум").pack()
    Button(root, text="Открыть", command=lambda: rs485.send_data(barrier1.open())).pack()
    Button(root, text="Закрыть", command=lambda: rs485.send_data(barrier1.close())).pack()

    Label(root, text="\nСветофор").pack()
    Button(root, text="Зелёный", command=lambda: rs485.send_data(light1.change_light(0))).pack()
    Button(root, text="Жёлтый", command=lambda: rs485.send_data(light1.change_light(1))).pack()
    Button(root, text="Красный", command=lambda: rs485.send_data(light1.change_light(2))).pack()

    Label(root, text="\nДорожный знак").pack()
    Button(root, text="Повернуть", command=lambda: rs485.send_data(sign1.rotate())).pack()

    root.mainloop()

if __name__ == "__main__":
    main()