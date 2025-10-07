import time
import board
import digitalio
import analogio
import pwmio
import wifi
import socketpool
import json
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import supervisor
import sys

# =====================================================
#                 CONFIGURACION GENERAL
# =====================================================
DO_ACTIVE_LOW   = True   # KY-026: True si DO=0 cuando detecta temperatura alta
INVERT_RGB      = False  # KY-009: invierte brillo si te queda al reves
TILT_ACTIVE_LOW = True   # KY-027: True si S=0 cuando esta inclinado

# =====================================================
#                 UMBRALES Y PARAMETROS DEL SISTEMA
# =====================================================
TEMP_HIGH_V = 0.80          # Voltios - umbral de temperatura alta (INVERTIDO: valores bajos = temp alta)
PARACHUTE_MS = 2000          # Duración del paracaídas desplegado (ms)
RPM_BASE = 30                # RPM base del motor (aumentado)
RPM_TURB = 20              # RPM durante turbulencia (inclinación)
RPM_OVERHEAT = 10            # RPM durante sobrecalentamiento (fuego)
RPM_DEPLOY = 5               # RPM durante despliegue de paracaídas

# =====================================================
#                 CONFIGURACION DE RED Y MQTT
# =====================================================
SSID = "wfrre-Docentes"
PASSWORD = "20$tscFrre.24"
BROKER = "10.13.100.84"
NOMBRE_EQUIPO = "Transferitos"
DESCOVERY_TOPIC = "descubrir"
TOPIC = "sensores/" + NOMBRE_EQUIPO

# Variables para control de publicación MQTT
last_pub = 0
PUB_INTERVAL = 5  # Publicar cada 5 segundos

# =====================================================
#                  KY-026 (sensor temperatura)
#   Cableado: +->3V3, G->GND, DO->GP16, AO->A0(GP26)
# =====================================================
do_fire = digitalio.DigitalInOut(board.GP16)  # DO
do_fire.direction = digitalio.Direction.INPUT
do_fire.pull = digitalio.Pull.UP

ao_fire = analogio.AnalogIn(board.A0)         # AO (GP26/ADC0)

def ao_volts(adc):
    return (adc.value * 3.3) / 65535.0

def temperatura_detectada(d_val_bool):
    """Detecta temperatura alta basada en el pin digital DO del KY-026
    DO_ACTIVE_LOW=True significa que DO=0 cuando detecta temperatura alta"""
    if DO_ACTIVE_LOW:
        return (not d_val_bool)
    else:
        return d_val_bool

# =====================================================
#                     LED 5 mm (paracaídas)
#          GP17 -> R=680 ohm -> anodo, catodo->GND
# =====================================================
led5 = digitalio.DigitalInOut(board.GP17)
led5.direction = digitalio.Direction.OUTPUT
led5.value = False

# LED onboard (opcional)
try:
    led_on = digitalio.DigitalInOut(board.LED)
    led_on.direction = digitalio.Direction.OUTPUT
    HAVE_LED_ON = True
except Exception:
    HAVE_LED_ON = False

# =====================================================
#                      KY-009 RGB (estados seguridad)
#         Catodo comun a GND; R->R=680->GP18, G->R=680->GP19, B->R=680->GP20
# =====================================================
pwm_r = pwmio.PWMOut(board.GP18, frequency=1000, duty_cycle=0)
pwm_g = pwmio.PWMOut(board.GP19, frequency=1000, duty_cycle=0)
pwm_b = pwmio.PWMOut(board.GP20, frequency=1000, duty_cycle=0)

def u16(x8):
    return x8 * 257

def set_rgb(r8, g8, b8):
    R = u16(r8); G = u16(g8); B = u16(b8)
    if INVERT_RGB:
        R = 65535 - R; G = 65535 - G; B = 65535 - B
    pwm_r.duty_cycle = R
    pwm_g.duty_cycle = G
    pwm_b.duty_cycle = B

# Estados de seguridad del sistema
ESTADO_SEGURO = (255, 0, 0)     # Verde: condiciones seguras
ESTADO_TURBULENCIA = (0, 0, 255)  # Azul: turbulencia detectada
ESTADO_SOBRECALENTAMIENTO = (0, 255, 0)  # Rojo: sobrecalentamiento
ESTADO_APAGADO = (0, 0, 0)       # Negro: sistema apagado

# =====================================================
#                 KY-027 Magic Light Cup (turbulencia/apogeo)
#          G->GND, +->3V3, S->GP21 (input), L->GP22 (LED)
# =====================================================
tilt = digitalio.DigitalInOut(board.GP21)   # S
tilt.direction = digitalio.Direction.INPUT
tilt.pull = digitalio.Pull.UP

tilt_led = digitalio.DigitalInOut(board.GP22)  # L
tilt_led.direction = digitalio.Direction.OUTPUT
tilt_led.value = False

# =====================================================
#        Motor 28BYJ-48 + ULN2003 (propulsión) - ADAPTADO
#       IN1..IN4 = GP10..GP13 ; VCC ULN2003 = +5V ; GND comun
# =====================================================
PINS_MOTOR = [board.GP10, board.GP11, board.GP12, board.GP13]

# Half-step (8 estados) — secuencia completa con IN2
HALF_SEQ = [
    (1,0,0,0),  # IN1
    (1,1,0,0),  # IN1+IN2
    (0,1,0,0),  # IN2
    (0,1,1,0),  # IN2+IN3
    (0,0,1,0),  # IN3
    (0,0,1,1),  # IN3+IN4
    (0,0,0,1),  # IN4
    (1,0,0,1),  # IN1+IN4
]
STEPS_PER_REV = 4096  # típico 28BYJ-48 en half-step

# GPIO del motor
coils = [digitalio.DigitalInOut(p) for p in PINS_MOTOR]
for c in coils:
    c.direction = digitalio.Direction.OUTPUT
    c.value = 0

def write_state(s):
    for pin, val in zip(coils, s):
        pin.value = bool(val)

# Control de velocidad mejorado
_current_rpm = 0.0
_last_t = 0

def rpm_to_delay(rpm):
    # segundos por paso = 60 / (rpm * pasos_por_vuelta)
    d = 60.0 / (max(rpm, 0.1) * STEPS_PER_REV)
    # Límite más permisivo para velocidades más altas
    if d < 0.001:  # 1ms por paso mínimo para velocidades más altas
        d = 0.001
    return d

def set_rpm(target_rpm, slew_rpm_per_s=20.0):
    """Rampa la rpm actual hacia la objetivo para evitar saltos bruscos."""
    global _current_rpm, _last_t
    now = time.monotonic()
    dt = now - _last_t if '_last_t' in globals() else 0.02
    _last_t = now
    max_delta = slew_rpm_per_s * dt
    if target_rpm > _current_rpm:
        _current_rpm = min(target_rpm, _current_rpm + max_delta)
    else:
        _current_rpm = max(target_rpm, _current_rpm - max_delta)
    return _current_rpm

# Movimiento continuo mejorado
seq_idx = 0
def motor_step_once(direction=1):
    global seq_idx
    seq_idx = (seq_idx + direction) % len(HALF_SEQ)
    write_state(HALF_SEQ[seq_idx])
    # Debug: mostrar cada paso (comentado para no saturar)
    # print("Paso " + str(seq_idx) + ": " + str(HALF_SEQ[seq_idx]))

def motor_spin(rpm, direction=1, chunk_steps=64):
    """Gira en chunks muy grandes con delays más largos"""
    rpm_now = set_rpm(rpm)
    d = rpm_to_delay(rpm_now)
    # Delay mínimo más permisivo para velocidades más altas
    d = max(d, 0.002)  # Mínimo 2ms por paso
    for _ in range(chunk_steps):
        motor_step_once(direction)
        time.sleep(d)
    return rpm_now

def motor_off():
    write_state((0,0,0,0))

# =====================================================
#                 VARIABLES DE CONTROL CONTINUO
# =====================================================
# Variables para filtrado de señales
tilt_history = []
last_tilt_change = 0

# Variables para control del motor
motor_active = False
rpm_actual = 0

# Variables para control del paracaídas
parachute_start = 0
parachute_active = False
parachute_deploying = False
parachute_closing = False
parachute_ejected = False  # Estado cuando el paracaídas fue expulsado en emergencia

# Variables para simulación de altura
altura_actual = 0
altura_incremento = 50  # metros por ciclo cuando el motor está activo
ALTURA_MAXIMA = 40000   # metros - altura máxima simulada

# Variables para sistema de descenso
descenso_activo = False
tiempo_paracaidas = 0
PARACAIDAS_DURACION = 5  # segundos antes de empezar descenso
aterrizaje_completado = False

# Variables para control del parpadeo LED
last_led_change = 0
led_state = False

# Variables para control del parpadeo LED del paracaídas
last_parachute_led_change = 0
parachute_led_state = False

# Variables para telemetría
last_console = 0

# =====================================================
#                 CONTROL POR PUERTO SERIAL
# =====================================================
# Variables para control manual
manual_rpm_override = False
manual_rpm_value = 0
context_rpm_base = RPM_BASE
context_rpm_turb = RPM_TURB
context_rpm_overheat = RPM_OVERHEAT
context_rpm_deploy = RPM_DEPLOY
emergency_mode = False
emergency_ejection = False

# =====================================================
#                 FUNCIONES DE FILTRADO Y DETECCION
# =====================================================
def is_turbulent(tilted):
    """Detecta turbulencia cuando hay inclinación"""
    return tilted

def calculate_rpm_target(overheated, turbulent, descenso=False, parachute_deployed=False):
    """Calcula RPM objetivo basado en condiciones"""
    global manual_rpm_override, manual_rpm_value, emergency_mode, context_rpm_base, context_rpm_turb, context_rpm_overheat, context_rpm_deploy
    
    # Si hay override manual, usar ese valor
    if manual_rpm_override:
        return manual_rpm_value
    
    # Si el paracaídas está desplegado, usar RPM de despliegue
    if parachute_deployed:
        return context_rpm_deploy
    
    if descenso:
        # Durante el descenso: RPM inversas (más problemas = más RPM)
        if overheated and turbulent:
            return 60  # Fuego + turbulencia = 60 RPM
        elif overheated:
            return 60  # Solo fuego = 60 RPM
        elif turbulent:
            return 40  # Solo turbulencia = 40 RPM
        else:
            return 20  # Condiciones seguras = 20 RPM
    else:
        # Durante el ascenso: RPM normales (más problemas = menos RPM)
        if overheated and turbulent:
            return min(context_rpm_overheat, context_rpm_turb)
        elif overheated:
            return context_rpm_overheat
        elif turbulent:
            return context_rpm_turb
        else:
            return context_rpm_base

def update_motor_control(rpm_target_new):
    """Controla el motor usando la implementación adaptada"""
    global motor_active, rpm_actual
    
    # Control del motor - usando nueva implementación
    if rpm_target_new <= 0:
        motor_off()
        motor_active = False
        rpm_actual = 0
        return
    
    motor_active = True
    rpm_actual = rpm_target_new  # Usar directamente el RPM objetivo
    
    # Usar chunks más pequeños pero más frecuentes
    motor_spin(rpm_target_new, direction=1, chunk_steps=16)

def control_led_rgb(overheated, turbulent):
    """Controla el LED RGB con parpadeo según condiciones"""
    global last_led_change, led_state
    
    now = time.monotonic()
    blink_interval = 0.5  # 500ms para parpadeo

    if overheated:
        # Parpadeo rojo para fuego (temperatura alta)
        if now - last_led_change >= blink_interval:
            led_state = not led_state
            last_led_change = now
        if led_state:
            set_rgb(*ESTADO_SOBRECALENTAMIENTO)  # Rojo
        else:
            set_rgb(*ESTADO_APAGADO)  # Apagado
    elif turbulent:
        # Parpadeo azul para turbulencia (inclinación)
        if now - last_led_change >= blink_interval:
            led_state = not led_state
            last_led_change = now
        if led_state:
            set_rgb(*ESTADO_TURBULENCIA)  # Azul
        else:
            set_rgb(*ESTADO_APAGADO)  # Apagado
    else:
        # Verde fijo para condiciones seguras (sin fuego ni turbulencia)
        set_rgb(*ESTADO_SEGURO)  # Verde fijo, sin parpadeo
        led_state = False

def control_parachute_led(should_blink):
    """Controla el LED del paracaídas con parpadeo"""
    global last_parachute_led_change, parachute_led_state
    
    now = time.monotonic()
    blink_interval = 0.3  # 300ms para parpadeo más rápido
    
    if should_blink:
        if now - last_parachute_led_change >= blink_interval:
            parachute_led_state = not parachute_led_state
            last_parachute_led_change = now
        led5.value = parachute_led_state
        if HAVE_LED_ON:
            led_on.value = parachute_led_state
    else:
        # LED fijo cuando no debe parpadear
        led5.value = False
        if HAVE_LED_ON:
            led_on.value = False
        parachute_led_state = False

def update_altitude(overheated, turbulent):
    """Simula el aumento/descenso de altura basado en el motor y RPM"""
    global altura_actual, descenso_activo, parachute_active, parachute_closing
    
    # Calcular velocidad basada en RPM actual (RPM * 3)
    velocidad_base = rpm_actual * 3
    
    if descenso_activo:
        # Durante el descenso, velocidad proporcional a RPM (100 - RPM)
        velocidad_descenso = 100 - rpm_actual
        altura_actual -= velocidad_descenso
        altura_actual = max(altura_actual, 0)  # No puede bajar de 0
        
        # Cuando llega a 0, activar secuencia de aterrizaje con delay
        if altura_actual <= 0 and not parachute_closing:
            altura_actual = 0
            parachute_closing = True
            parachute_start = time.monotonic()
            print("🛬 ATERRIZAJE COMPLETADO - Iniciando secuencia de cierre del paracaídas")
    elif motor_active:
        # Durante el ascenso, velocidad basada en RPM
        altura_actual += velocidad_base
        
        # Si llegamos a la altura máxima pero no podemos desplegar paracaídas, seguimos subiendo
        if altura_actual >= ALTURA_MAXIMA and not parachute_active and not parachute_deploying:
            condiciones_seguras = not overheated and not turbulent
            if not condiciones_seguras:
                # Continuar subiendo si las condiciones no son seguras
                altura_actual += velocidad_base
                if overheated:
                    print("⚠️ ALTURA MÁXIMA ALCANZADA PERO FUEGO DETECTADO - Continuando ascenso...")
                if turbulent:
                    print("⚠️ ALTURA MÁXIMA ALCANZADA PERO TURBULENCIA DETECTADA - Continuando ascenso...")
    
    return altura_actual

def update_parachute_control(overheated, turbulent, altura):
    """Controla el paracaídas y el sistema de descenso con delays y parpadeo"""
    global parachute_start, parachute_active, descenso_activo, tiempo_paracaidas, parachute_deploying, parachute_closing, aterrizaje_completado, emergency_mode, emergency_ejection, parachute_ejected
    
    now = time.monotonic()
    
    # Si hay expulsión de emergencia, desplegar paracaídas inmediatamente
    if emergency_ejection and not parachute_active and not parachute_deploying:
        parachute_active = True
        tiempo_paracaidas = now
        led5.value = True
        if HAVE_LED_ON:
            led_on.value = True
        print("🚨 EXPULSIÓN DE EMERGENCIA - PARACAÍDAS DESPLEGADO")
        emergency_ejection = False  # Reset flag
    
    # Condiciones para desplegar paracaídas: altura mínima alcanzada + condiciones seguras + SOLO DURANTE ASCENSO
    altura_suficiente = altura >= ALTURA_MAXIMA  # 40000 metros
    condiciones_seguras = not overheated and not turbulent
    durante_ascenso = not descenso_activo  # Solo desplegar durante ascenso, NO durante descenso
    
    should_deploy = altura_suficiente and condiciones_seguras and durante_ascenso
    
    # Despliegue inmediato del paracaídas cuando las condiciones sean seguras (SOLO EN ASCENSO)
    if should_deploy and not parachute_active and not parachute_deploying:
        parachute_active = True
        tiempo_paracaidas = now
        # Encender LED inmediatamente al desplegar
        led5.value = True
        if HAVE_LED_ON:
            led_on.value = True
        print("🪂 PARACAÍDAS DESPLEGADO COMPLETAMENTE")
    
    # LED fijo encendido cuando el paracaídas está desplegado
    if parachute_active:
        led5.value = True
        if HAVE_LED_ON:
            led_on.value = True
    
    # Control del descenso después del paracaídas (esperar 5 segundos)
    # FIXED: Ahora funciona tanto en ascenso como en descenso para emergencias
    if parachute_active and (now - tiempo_paracaidas >= PARACAIDAS_DURACION):
        if not descenso_activo:
            descenso_activo = True
            parachute_active = False
            parachute_ejected = True  # Marcar como expulsado después de 5 segundos
            # Apagar LED del paracaídas al iniciar descenso
            control_parachute_led(False)
            print("📉 INICIANDO DESCENSO - Paracaídas expulsado")
        else:
            # Si ya estamos en descenso (emergencia), cambiar estado a "EXPULSADO" después de 5 segundos
            parachute_active = False
            parachute_ejected = True
            control_parachute_led(False)
            print("🚨 PARACAÍDAS EXPULSADO - Estado cambiado a EXPULSADO")
    
    # Control de secuencia de cierre después del aterrizaje (5 segundos con LED parpadeando)
    # Solo ejecutar si el paracaídas no fue expulsado en emergencia
    if parachute_closing and (now - parachute_start >= 5.0) and not parachute_ejected:
        parachute_closing = False
        descenso_activo = False
        parachute_active = False
        parachute_ejected = False
        control_parachute_led(False)  # Apagar LED después del delay
        aterrizaje_completado = True
        emergency_mode = False  # Reset emergency mode
        print("🛬 PARACAÍDAS CERRADO COMPLETAMENTE")
    
    # LED parpadeando durante el cierre
    if parachute_closing:
        control_parachute_led(True)  # Parpadeo durante cierre
    
    # Si el paracaídas fue expulsado y llegamos a altura 0, activar aterrizaje
    if parachute_ejected and altura <= 0 and not aterrizaje_completado:
        aterrizaje_completado = True
        emergency_mode = False  # Reset emergency mode
        print("🛬 ATERRIZAJE COMPLETADO - Paracaídas expulsado")

# =====================================================
#                 FUNCIONES DE CONTROL SERIAL
# =====================================================
def process_serial_command(command):
    """Procesa comandos recibidos por puerto serial"""
    global manual_rpm_override, manual_rpm_value, context_rpm_base, context_rpm_turb, context_rpm_overheat, context_rpm_deploy
    global emergency_mode, emergency_ejection, descenso_activo, parachute_active
    
    command = command.strip().upper()
    
    if command == "HELP":
        print("\n=== COMANDOS DISPONIBLES ===")
        print("SET_RPM <valor>     - Cambiar RPM actual (ignora cambios automáticos)")
        print("RESET_RPM           - Volver al control automático de RPM")
        print("SET_CONTEXT_RPM <contexto> <valor> - Cambiar RPM específico (F/T/N/D)")
        print("EMERGENCY_LANDING   - Aterrizaje de emergencia (sin expulsión)")
        print("EMERGENCY_EJECTION  - Expulsión de emergencia + aterrizaje")
        print("STATUS              - Mostrar estado actual del sistema")
        print("HELP                - Mostrar esta ayuda")
        print("===========================\n")
        return True
    
    elif command.startswith("SET_RPM "):
        try:
            rpm = float(command.split()[1])
            manual_rpm_override = True
            manual_rpm_value = rpm
            print(f"🎮 ✅ RPM MANUAL: {rpm} RPM establecido")
            return True
        except (IndexError, ValueError):
            print("❌ Error: SET_RPM requiere un valor numérico")
            return False
    
    elif command == "RESET_RPM":
        manual_rpm_override = False
        print("🎮 ✅ CONTROL AUTOMÁTICO restaurado")
        return True
    
    elif command.startswith("SET_CONTEXT_RPM "):
        try:
            parts = command.split()
            if len(parts) != 3:
                print("❌ Error: SET_CONTEXT_RPM <contexto> <valor>")
                print("   Contextos: F (Fuego), T (Turbulencia), N (Normal), D (Despliegue)")
                return False
            
            context = parts[1].upper()
            rpm_value = float(parts[2])
            
            if context == "F":
                context_rpm_overheat = rpm_value
                print(f"🎮 ✅ RPM FUEGO: {rpm_value}")
            elif context == "T":
                context_rpm_turb = rpm_value
                print(f"🎮 ✅ RPM TURBULENCIA: {rpm_value}")
            elif context == "N":
                context_rpm_base = rpm_value
                print(f"🎮 ✅ RPM NORMAL: {rpm_value}")
            elif context == "D":
                context_rpm_deploy = rpm_value
                print(f"🎮 ✅ RPM DESPLIEGUE: {rpm_value}")
            else:
                print("❌ Error: Contexto inválido. Use F, T, N o D")
                return False
            
            return True
        except (IndexError, ValueError):
            print("❌ Error: SET_CONTEXT_RPM <contexto> <valor>")
            print("   Contextos: F (Fuego), T (Turbulencia), N (Normal), D (Despliegue)")
            return False
    
    elif command == "EMERGENCY_LANDING":
        emergency_mode = True
        descenso_activo = True
        parachute_active = False
        manual_rpm_override = True
        manual_rpm_value = 20  # RPM para descenso controlado
        print("🚨 ⚠️ ATERRIZAJE DE EMERGENCIA ACTIVADO")
        return True
    
    elif command == "EMERGENCY_EJECTION":
        emergency_ejection = True
        emergency_mode = True
        print("🚨 ⚠️ EXPULSIÓN DE EMERGENCIA ACTIVADA")
        return True
    
    elif command == "STATUS":
        print("\n=== ESTADO DEL SISTEMA ===")
        print(f"RPM Manual: {'SÍ' if manual_rpm_override else 'NO'} ({manual_rpm_value if manual_rpm_override else 'AUTO'})")
        print(f"RPM Contexto - Normal: {context_rpm_base}, Turbulencia: {context_rpm_turb}")
        print(f"RPM Contexto - Fuego: {context_rpm_overheat}, Despliegue: {context_rpm_deploy}")
        print(f"Modo Emergencia: {'SÍ' if emergency_mode else 'NO'}")
        print(f"Descenso Activo: {'SÍ' if descenso_activo else 'NO'}")
        print(f"Paracaídas Activo: {'SÍ' if parachute_active else 'NO'}")
        print("========================\n")
        return True
    
    else:
        print(f"❌ Comando desconocido: {command}")
        print("Escribe 'HELP' para ver comandos disponibles")
        return False

def check_serial_input():
    """Verifica si hay comandos disponibles en el puerto serial de forma no bloqueante"""
    if supervisor.runtime.serial_bytes_available:
        try:
            # Leer datos disponibles de forma no bloqueante
            command = ""
            while supervisor.runtime.serial_bytes_available:
                char = sys.stdin.read(1)
                if char == '\n' or char == '\r':
                    break
                command += char
            
            if command.strip():
                print(f"\n📡 " + "="*60)
                print(f"📡 COMANDO RECIBIDO: {command.strip()}")
                print(f"📡 " + "="*60)
                process_serial_command(command.strip())
                print(f"📡 " + "="*60)
                print("📡 COMANDO PROCESADO - CONTINUANDO OPERACIÓN NORMAL")
                print(f"📡 " + "="*60 + "\n")
        except Exception as e:
            print(f"Error procesando comando serial: {e}")

# =====================================================
#                 CONEXION WiFi Y MQTT
# =====================================================
print("Intentando conectar a " + SSID + "...")
try:
    wifi.radio.connect(SSID, PASSWORD)
    print("Conectado a " + SSID)
    print("Dirección IP: " + str(wifi.radio.ipv4_address))
except Exception as e:
    print("Error al conectar a WiFi: " + str(e))
    print("Continuando sin WiFi...")
    wifi_connected = False
else:
    wifi_connected = True

# Configuración MQTT solo si hay WiFi
mqtt_client = None
if wifi_connected:
    try:
        pool = socketpool.SocketPool(wifi.radio)

        def connect(client, userdata, flags, rc):
            print("Conectado al broker MQTT")
            client.publish(DESCOVERY_TOPIC, json.dumps({
                "equipo": NOMBRE_EQUIPO,
                "magnitudes": ["temperatura_digital", "temperatura_analogica", "inclinacion"]
            }))

        mqtt_client = MQTT.MQTT(
            broker=BROKER,
            port=1883,
            socket_pool=pool
        )
        mqtt_client.on_connect = connect
        mqtt_client.connect()
        print("Cliente MQTT configurado")
    except Exception as e:
        print("Error configurando MQTT: " + str(e))
        mqtt_client = None

# =====================================================
#                 FUNCION PUBLICACION MQTT
# =====================================================
def publish_mqtt_data(temp_digital, temp_analog, tilted):
    global last_pub
    now = time.monotonic()
    
    if mqtt_client is None:
        return
   
    if now - last_pub >= PUB_INTERVAL:
        try:
            # Publicar datos de temperatura KY-026
            temp_digital_topic = TOPIC + "/temperatura_digital"
            mqtt_client.publish(temp_digital_topic, str(int(temp_digital)))
            
            temp_analog_topic = TOPIC + "/temperatura_analogica"
            mqtt_client.publish(temp_analog_topic, str(temp_analog).lower())
            
            # Publicar datos de inclinación KY-027
            inclinacion_topic = TOPIC + "/inclinacion"
            mqtt_client.publish(inclinacion_topic, str(tilted).lower())
            
            last_pub = now
            print("Datos MQTT publicados: temp_digital=" + str(int(temp_digital)) + ", temp_analog=" + str(round(temp_analog, 2)) + ", tilted=" + str(int(tilted)))

        except Exception as e:
            print("Error publicando MQTT: " + str(e))

# =====================================================
#                      LOOP PRINCIPAL
# =====================================================
print("\n" + "🚀" + "="*78 + "🚀")
print("🚀                    SISTEMA TRANSFERITOS - CONTROL EN TIEMPO REAL                    🚀")
print("🚀" + "="*78 + "🚀")
print("📡 Sistema de control por puerto serial ACTIVADO")
print("🎮 Escribe comandos en cualquier momento - el sistema continúa funcionando")
print("📊 Telemetría actualizada cada 300ms")
print("❓ Escribe 'HELP' para ver todos los comandos disponibles")
print("🚀" + "="*78 + "🚀\n")

while True:
    # ---- Verificar comandos por puerto serial ----
    check_serial_input()
    
    # ---- Lecturas y acondicionamiento de señales ----
    now = time.monotonic()  # Definir now al inicio del loop
    
    # KY-026: Sensor de temperatura
    d_raw = do_fire.value
    ao_raw = ao_fire.value
    temp_volts = ao_volts(ao_fire)  # Valores bajos = temperatura alta (lógica invertida)
    temp_digital = temperatura_detectada(d_raw)
    
    # KY-027: Sensor de inclinación/turbulencia
    raw_tilt = tilt.value
    tilted = (not raw_tilt) if TILT_ACTIVE_LOW else raw_tilt
    tilt_led.value = tilted
    
    # ---- Derivar banderas de condición ----
    overheated = temp_volts <= TEMP_HIGH_V  # INVERTIDO: valores bajos = temperatura alta
    turbulent = is_turbulent(tilted)  # Cualquier inclinación = turbulencia
    
    # ---- Control de propulsión (motor) ----
    rpm_target_new = calculate_rpm_target(overheated, turbulent, descenso_activo, parachute_active)
    update_motor_control(rpm_target_new)
    
    # ---- Actualizar altura simulada ----
    altura_actual = update_altitude(overheated, turbulent)
    
    # ---- Indicadores luminosos (LED RGB) ----
    # ---- Control del LED RGB ----
    control_led_rgb(overheated, turbulent)
    
    # ---- Reiniciar sistema después del aterrizaje ----
    if aterrizaje_completado:
        print("🚀 SUBIENDO AL SIGUIENTE PASAJERO...")
        
        # Parpadeo durante 5 segundos antes de reiniciar
        inicio_espera = time.monotonic()
        while (time.monotonic() - inicio_espera) < 5.0:
            control_parachute_led(True)  # Parpadeo durante la espera
            time.sleep(0.1)  # Pequeña pausa para no bloquear el sistema
        
        # Reiniciar todas las variables
        aterrizaje_completado = False
        altura_actual = 0
        parachute_deploying = False
        parachute_closing = False
        parachute_active = False
        parachute_ejected = False
        descenso_activo = False
        emergency_mode = False
        emergency_ejection = False
        manual_rpm_override = False
        control_parachute_led(False)  # Asegurar que el LED esté apagado
        print("🚀 SISTEMA REINICIADO - Nuevo vuelo iniciado")
    
    # ---- Control del paracaídas (LED 5mm) ----
    update_parachute_control(overheated, turbulent, altura_actual)
    
    # ---- Publicación MQTT ----
    publish_mqtt_data(temp_digital, temp_volts, tilted)
    
    # ---- Telemetría/Debug cada ~300ms ----
    if now - last_console >= 0.3:
        motor_status = "RPM: " + str(round(rpm_actual, 1)) if motor_active else "Motor: OFF"
        parachute_status = "EXPULSADO" if parachute_ejected else ("DESPLEGADO" if parachute_active else "RETRAÍDO")
        control_status = "MANUAL" if manual_rpm_override else "AUTO"
        emergency_status = "EMERGENCIA" if emergency_mode else "NORMAL"
        
        # Debug del motor
        if rpm_target_new == 0:
            print("🔧 Motor OFF - Temp: " + str(round(temp_volts, 2)) + "V, Turbulencia: " + str(turbulent))
        
        estado_vuelo = "DESCENSO" if descenso_activo else ("ASCENSO" if motor_active else "ESTACIONARIO")
        
        # Formato mejorado para seguimiento en tiempo real
        print("🚀 " + "="*80)
        print(f"📊 ALTURA: {altura_actual:>6}m | 🌡️ TEMP: {temp_volts:>5.2f}V ({'ALTA' if overheated else 'OK':>3}) | 🌪️ TURB: {'SÍ' if turbulent else 'NO':>2}")
        print(f"⚙️  MOTOR: {motor_status:>12} | 🪂 PARACAÍDAS: {parachute_status:>10} | 🎯 ESTADO: {estado_vuelo:>8}")
        print("🚀 " + "="*80)
        
        last_console = now
    
    # ---- Pausa del ciclo ----
    time.sleep(0.1)  # 100ms de ciclo para mantener responsividad
