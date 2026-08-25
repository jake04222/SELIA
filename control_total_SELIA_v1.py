import os
os.environ['KIVY_WINDOW'] = 'sdl2'
os.environ['KIVY_GL_BACKEND'] = 'gl'

import threading
import time
import math
import serial

from kivy.config import Config
Config.set('graphics', 'width', '800')
Config.set('graphics', 'height', '480')
Config.set('graphics', 'resizable', '0')
Config.set('graphics', 'fullscreen', '0') 

from kivy.app import App
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.properties import NumericProperty, BooleanProperty, StringProperty
from kivy.clock import Clock
from kivy.lang import Builder
import RPi.GPIO as GPIO

GPIO.setwarnings(False)

PIN_SENSOR = 24    # Entrada Sensor Inductivo
PIN_ACTUADOR = 23  # Salida Actuador Perforador

DIAMETRO_RUEDA_M = 0.254 
PPR = 16384  
CIRCUNFERENCIA = math.pi * DIAMETRO_RUEDA_M
PULSOS_POR_METRO = PPR / CIRCUNFERENCIA

PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
SLAVE_ID = 1
RAMPA_MS = 400  
VEL_RPM = 150     
VEL_LENTA_RPM = 40  # Velocidad suave para traslado en modo constante

class PantallaPrincipal(Screen):
    conteo_pasos = NumericProperty(0)
    ciclo_activo = BooleanProperty(False)
    sensor_on = BooleanProperty(False)
    actuador_on = BooleanProperty(False)
    estado_texto = StringProperty("ESTADO: LISTO PARA INICIAR")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PIN_SENSOR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PIN_ACTUADOR, GPIO.OUT)
        GPIO.output(PIN_ACTUADOR, GPIO.HIGH)
        
        self.ser = None
        Clock.schedule_interval(self.check_sensor_ui, 0.05)

    def check_sensor_ui(self, dt):
        self.sensor_on = GPIO.input(PIN_SENSOR) == GPIO.LOW

    def calcular_crc(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, 'little')

    def enviar_trama(self, trama_base):
        if self.ser and self.ser.is_open:
            trama_completa = trama_base + self.calcular_crc(trama_base)
            self.ser.write(trama_completa)
            time.sleep(0.04)

    def int_a_4bytes(self, n):
        b = n.to_bytes(4, 'big', signed=True)
        return b[0:2], b[2:4]

    def iniciar_ciclo(self):
        if not self.ciclo_activo:
            try:
                self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
                self.ciclo_activo = True
                threading.Thread(target=self.logica_maquina, daemon=True).start()
            except Exception as e:
                self.estado_texto = f"ERROR SERIAL: No se pudo abrir {PORT}. {str(e)}"

    def detener_ciclo(self):
        self.ciclo_activo = False
        self.estado_texto = "ESTADO: DETENIENDO SISTEMA..."
        GPIO.output(PIN_ACTUADOR, GPIO.HIGH)
        self.actuador_on = False
        
        if self.ser and self.ser.is_open:
            self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x07]))
            self.ser.close()
        self.estado_texto = "ESTADO: DETENIDO Y DESHABILITADO"

    def resetear_contador(self):
        self.conteo_pasos = 0
        self.estado_texto = "ESTADO: CONTADOR RESETEADO"

    def logica_maquina(self):
        try:
            self.estado_texto = "ESTADO: ESPERANDO INICIO (3s)..."
            time.sleep(3)
            
            while self.ciclo_activo:
                if not self.ciclo_activo: break

                self.estado_texto = "ESTADO: PERFORANDO..."
                self.actuador_on = True
                GPIO.output(PIN_ACTUADOR, GPIO.LOW)
                
                self.estado_texto = "ESTADO: ESPERANDO SENSOR INDUCTIVO..."
                while GPIO.input(PIN_SENSOR) == GPIO.HIGH:
                    if not self.ciclo_activo: break
                    time.sleep(0.02)
                
                if not self.ciclo_activo: break
                
                self.conteo_pasos += 1
                self.actuador_on = False
                GPIO.output(PIN_ACTUADOR, GPIO.HIGH) 
                
                self.estado_texto = "ESTADO: PAUSA PRE-AVANCE (2s)..."
                time.sleep(2) 
                if not self.ciclo_activo: break
                
                app_global = App.get_running_app()
                distancia_cm = app_global.distancia_avance_cm
                avance_metros = distancia_cm / 100.0 
                
                self.estado_texto = f"ESTADO: AVANZANDO {distancia_cm} CM..."
                self.mover_motor_zltech(avance_metros) 
                
                if not self.ciclo_activo: break
                
                self.estado_texto = "ESTADO: PAUSA POST-AVANCE (4s)..."
                time.sleep(20) # tiempo de pausa a configurar

        except Exception as e:
            self.estado_texto = f"ERROR EN CICLO: {str(e)}"
            self.detener_ciclo()

    def mover_motor_zltech(self, metros):
        pulsos_base = int(metros * PULSOS_POR_METRO)
        pulsos_izq = pulsos_base
        pulsos_der = -pulsos_base 

        self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x0F, 0x00, 0x01]))
        self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x0D, 0x00, 0x01]))
        
        r_bytes = RAMPA_MS.to_bytes(2, 'big')
        for reg in [0x80, 0x81, 0x82, 0x83]:
            self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, reg]) + r_bytes)

        v_bytes = VEL_RPM.to_bytes(2, 'big')
        self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x8E]) + v_bytes)
        self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x8F]) + v_bytes)

        self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x08]))
        time.sleep(0.05)

        h_izq, l_izq = self.int_a_4bytes(pulsos_izq)
        h_der, l_der = self.int_a_4bytes(pulsos_der)
        trama_pos = bytearray([SLAVE_ID, 0x10, 0x20, 0x8A, 0x00, 0x04, 0x08])
        trama_pos += h_izq + l_izq + h_der + l_der
        self.enviar_trama(trama_pos)

        self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x10]))

        tiempo_viaje = (abs(metros) / (CIRCUNFERENCIA * (VEL_RPM/60))) + (RAMPA_MS/1000) + 0.5
        time.sleep(tiempo_viaje)

class PantallaConfiguracion(Screen):
    pass

class PantallaVelocidadConstante(Screen):
    marcha_activa = BooleanProperty(False)
    estado_texto = StringProperty("ESTADO: DETENIDO")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.ser = None

    def calcular_crc(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, 'little')

    def enviar_trama(self, trama_base):
        if self.ser and self.ser.is_open:
            trama_completa = trama_base + self.calcular_crc(trama_base)
            self.ser.write(trama_completa)
            time.sleep(0.04)

    def iniciar_marcha_lenta(self):
        if not self.marcha_activa:
            try:
                self.ser = serial.Serial(PORT, BAUDRATE, timeout=1)
                self.marcha_activa = True
                self.estado_texto = "ESTADO: INICIANDO MODO VELOCIDAD..."
                
                # 1. Configurar ZLTech en Profile Velocity Mode (0x200D = 0x0003)
                self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x0D, 0x00, 0x03]))
                
                # Opcional: Mantener rampas para aceleración suave
                r_bytes = RAMPA_MS.to_bytes(2, 'big')
                for reg in [0x80, 0x81, 0x82, 0x83]:
                    self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, reg]) + r_bytes)

                # 2. Habilitar drivers/motores (Motor enable 0x200E = 0x0008)
                self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x08]))

                # 3. Mando síncrono de velocidad según manual (01 10 20 88 00 02 04 [VI_H VI_L] [VD_H VD_L])
                v_izq_bytes = VEL_LENTA_RPM.to_bytes(2, 'big', signed=True)
                v_der_bytes = (-VEL_LENTA_RPM).to_bytes(2, 'big', signed=True)
                
                trama_velocidad = bytearray([SLAVE_ID, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04])
                trama_velocidad += v_izq_bytes + v_der_bytes
                
                self.enviar_trama(trama_velocidad)
                
                self.estado_texto = f"ESTADO: MARCHA CONSTANTE ACTIVA ({VEL_LENTA_RPM} RPM)"

            except Exception as e:
                self.estado_texto = f"ERROR EN MARCHA: {str(e)}"
                self.detener_marcha_lenta()

    def detener_marcha_lenta(self):
        self.marcha_activa = False
        if self.ser and self.ser.is_open:
            # Mandar velocidad a 0 síncronamente
            v_zero = (0).to_bytes(2, 'big', signed=True)
            trama_velocidad = bytearray([SLAVE_ID, 0x10, 0x20, 0x88, 0x00, 0x02, 0x04])
            trama_velocidad += v_zero + v_zero
            self.enviar_trama(trama_velocidad)
            
            # Deshabilitar motores
            self.enviar_trama(bytearray([SLAVE_ID, 0x06, 0x20, 0x0E, 0x00, 0x07]))
            self.ser.close()
        self.estado_texto = "ESTADO: MARCHA DETENIDA Y DESHABILITADA"

    def volver_menu(self):
        if self.marcha_activa:
            self.detener_marcha_lenta()
        App.get_running_app().root.current = 'principal'

    def on_leave(self):
        if self.marcha_activa:
            self.detener_marcha_lenta()

class VentanaManager(ScreenManager):
    pass

class SeaApp(App):
    distancia_avance_cm = NumericProperty(10)

    def build(self):
        Builder.load_file('sea_v3.kv')
        return VentanaManager()

if __name__ == '__main__':
    try:
        SeaApp().run()
    except KeyboardInterrupt:
        print("\nPrograma detenido manualmente.")
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass

