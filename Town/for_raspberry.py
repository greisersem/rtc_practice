import time
import serial
import minimalmodbus
import RPi.GPIO as GPIO
from tkinter import Tk, Label, Button, StringVar


class TrafficLight:
    """Класс логики светофора через Modbus

    Attributes
    ----------
    id : int
        Modbus ID устройства
    register : int
        Адрес holding register для команды
    """
    def __init__(self, id: int, register: int = 1) -> None:
        self.id = id
        self.register = register

    def change_light(self, light_id: int):
        # возвращаем кортеж: (slave_id, register, value)
        return (self.id, self.register, light_id)


class Barrier:
    """Класс логики шлагбаума через Modbus

    Attributes
    ----------
    id : int
        Modbus ID устройства
    coil : int
        Адрес coil для управления
    """
    open_id = 1
    close_id = 0

    def __init__(self, id: int, coil: int = 0) -> None:
        self.id = id
        self.coil = coil

    def open(self):
        return (self.id, self.coil, True)

    def close(self):
        return (self.id, self.coil, False)


class RoadSign:
    """Класс логики дорожного знака через Modbus

    Attributes
    ----------
    id : int
        Modbus ID устройства
    coil : int
        Адрес coil для управления
    """
    def __init__(self, id: int, coil: int = 2) -> None:
        self.id = id
        self.coil = coil

    def rotate(self):
        return (self.id, self.coil, True)


class RS485Controller:
    """Modbus RTU Master через minimalmodbus и MAX485"""
    def __init__(self, port: str, tx_rx_pin: int, baudrate: int = 9600):
        # настроим GPIO для DE/RE
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(tx_rx_pin, GPIO.OUT, initial=GPIO.LOW)
        self.tx_rx_pin = tx_rx_pin
        # настроим minimalmodbus.Instrument при каждом запросе создаётся с нужным slave
        self.port = port
        self.baudrate = baudrate

    def _get_instrument(self, slave_id: int):
        inst = minimalmodbus.Instrument(self.port, slave_id)
        inst.serial.baudrate = self.baudrate
        inst.serial.parity = serial.PARITY_NONE
        inst.serial.stopbits = 1
        inst.serial.bytesize = 8
        inst.serial.timeout = 0.5
        # DE/RE handlers
        inst.serial.before_write = self._pre_send
        inst.serial.after_write = self._post_send
        return inst

    def _pre_send(self):
        GPIO.output(self.tx_rx_pin, GPIO.HIGH)
        time.sleep(0.01)

    def _post_send(self):
        time.sleep(0.01)
        GPIO.output(self.tx_rx_pin, GPIO.LOW)

    def send_data(self, args):
        """
        Универсальный send:
        для coil args = (slave_id, coil_addr, bool)
        для register args = (slave_id, reg_addr, value)
        Возвращает строку статуса или ошибку
        """
        slave, addr, val = args
        inst = self._get_instrument(slave)
        try:
            if isinstance(val, bool):
                inst.write_bit(addr, val)
            else:
                inst.write_register(addr, val)
            return f"OK"
        except Exception as e:
            return f"ERROR: {e}"


def main():
    # инициализация контроллера
    controller = RS485Controller('/dev/serial0', tx_rx_pin=7)

    # создаём объекты
    barrier1 = Barrier(id=1, coil=0)
    light1 = TrafficLight(id=1, register=1)
    sign1  = RoadSign(id=1, coil=2)

    # GUI
    root = Tk()
    root.title("Управление городом")
    root.geometry("400x300")

    status_var = StringVar(value="Ожидание команд...")
    Label(root, textvariable=status_var, wraplength=380).pack(pady=10)

    def on_action(action_args, label: str):
        status_var.set(f"Отправка {label}...")
        resp = controller.send_data(action_args)
        status_var.set(f"{label} → {resp}")

    # Шлагбаум
    Label(root, text="Шлагбаум").pack()
    Button(root, text="Открыть",  command=lambda: on_action(barrier1.open(), "Barrier OPEN")).pack()
    Button(root, text="Закрыть", command=lambda: on_action(barrier1.close(),"Barrier CLOSE")).pack()

    # Светофор
    Label(root, text="Светофор").pack(pady=(10,0))
    Button(root, text="Зелёный", command=lambda: on_action(light1.change_light(0),"Light GREEN")).pack()
    Button(root, text="Жёлтый", command=lambda: on_action(light1.change_light(1),"Light YELLOW")).pack()
    Button(root, text="Красный", command=lambda: on_action(light1.change_light(2),"Light RED")).pack()

    # Знак
    Label(root, text="Дорожный знак").pack(pady=(10,0))
    Button(root, text="Повернуть", command=lambda: on_action(sign1.rotate(),"Sign ROTATE")).pack()

    root.mainloop()


if __name__ == '__main__':
    main()
