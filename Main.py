# main.py – Continuous Drive + Avoid Wall (Back + Turn) – START ON BUTTON PRESS
import time
import machine
from machine import Pin, PWM
from time import sleep_ms, sleep_us, ticks_us, ticks_ms, ticks_diff

# ================================
# Pin Assignments
# ================================
PWMA = PWM(Pin(9))
AIN2 = Pin(10, Pin.OUT)
AIN1 = Pin(11, Pin.OUT)
STBY = Pin(12, Pin.OUT)          
BIN1 = Pin(13, Pin.OUT)
BIN2 = Pin(14, Pin.OUT)
PWMB = PWM(Pin(15))
LED_R = Pin(18, Pin.OUT)
LED_G = Pin(17, Pin.OUT)
LED_B = Pin(16, Pin.OUT)
BUTTON = Pin(1, Pin.IN, Pin.PULL_DOWN)   
trig = Pin(3, Pin.OUT)
echo = Pin(2, Pin.IN, Pin.PULL_DOWN)

PWMA.freq(1000)
PWMB.freq(1000)
STBY.value(1)                     

# ================================
# State & Constants
# ================================
MODE_PAUSE = 0
MODE_WORK  = 1
current_mode = MODE_PAUSE

work_duration = 0
low_battery = False
critical_battery = False
red_blink_start_time = None

FULL_DUTY = 32768
HALF_DUTY = FULL_DUTY // 2
OBSTACLE_DISTANCE = 0.50          # meters

# Motor direction states
DIR_STOP       = 0
DIR_FORWARD    = 1
DIR_BACKWARD   = 2
DIR_TURN_RIGHT = 3

current_dir   = DIR_STOP
current_duty  = 0                 # 0-65535

# ================================
# Low-level helpers 
# ================================
def _set_direction(d):
    if d == DIR_FORWARD:
        AIN1.value(1); AIN2.value(0)
        BIN1.value(0); BIN2.value(1)
    elif d == DIR_BACKWARD:
        AIN1.value(0); AIN2.value(1)
        BIN1.value(1); BIN2.value(0)
    elif d == DIR_TURN_RIGHT:
        AIN1.value(1); AIN2.value(0)   
        BIN1.value(1); BIN2.value(0)   
    else:                               
        AIN1.value(0); AIN2.value(0)
        BIN1.value(0); BIN2.value(0)

def _set_pwm(duty):
    global current_duty
    if duty == current_duty: return
    PWMA.duty_u16(duty)
    PWMB.duty_u16(duty)
    current_duty = duty

def motor_stop():
    _set_pwm(0)

def motor_go(direction, duty):
    global current_dir
    if direction != current_dir:
        _set_direction(direction)
        current_dir = direction
    _set_pwm(duty)

# ================================
# LED Functions
# ================================
def led_all_on():  LED_R.value(1); LED_G.value(1); LED_B.value(1)
def led_all_off(): LED_R.value(0); LED_G.value(0); LED_B.value(0)
def led_red():   LED_R.value(1); LED_G.value(0); LED_B.value(0)
def led_green(): LED_R.value(0); LED_G.value(1); LED_B.value(0)
def led_blue():  LED_R.value(0); LED_G.value(0); LED_B.value(1)

# ================================
# Ultrasonic
# ================================
distance = None
tic = None
def compute_distance(pin):
    global distance, tic
    if pin.value() == 1:
        tic = ticks_us()
    else:
        if tic is None: return
        dur = ticks_diff(ticks_us(), tic)
        if dur < 100:
            distance = 0
        elif dur < 38000:
            distance = dur / 58 / 100
        else:
            distance = None
echo.irq(handler=compute_distance, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)

def measure_distance():
    global distance
    distance = None
    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10)
    trig.value(0)
    timeout = time.ticks_add(ticks_ms(), 100)
    while distance is None and time.ticks_diff(timeout, ticks_ms()) > 0:
        time.sleep_ms(1)
    return distance if distance is not None else 999

# ================================
# Button
# ================================
last_button_time = 0
debounce_delay = 50
def button_pressed():
    global last_button_time
    now = ticks_ms()
    if ticks_diff(now, last_button_time) < debounce_delay: return False
    if BUTTON.value() == 1:
        last_button_time = now
        return True
    return False

# ================================
# System check
# ================================
def system_check():
    global current_mode
    trig.value(0); time.sleep_ms(100)
    button_ok = (BUTTON.value() == 0)
    dist = measure_distance()
    sensor_ok = (dist is not None and 0 < dist < 10)
    if button_ok and sensor_ok:
        for _ in range(10):
            led_all_on(); time.sleep(0.1)
            led_all_off(); time.sleep(0.1)
    current_mode = MODE_PAUSE
    motor_stop()                     

# ================================
# Main Loop
# ================================
system_check()
work_start_time = None

while True:
    current_time = ticks_ms()

    # ---- work time / battery ------------------------------------------------
    if current_mode == MODE_WORK and work_start_time is not None:
        work_duration += ticks_diff(current_time, work_start_time) / 1000.0
        work_start_time = current_time
    low_battery      = work_duration > 45
    critical_battery = work_duration > 55

    if critical_battery and red_blink_start_time is None:
        red_blink_start_time = current_time
    if red_blink_start_time and ticks_diff(current_time, red_blink_start_time) >= 5000:
        motor_stop()
        led_all_off()
        print("SHUTDOWN - CRITICAL BATTERY")
        break

    # --------------------------------------------------------------------- PAUSE
    if current_mode == MODE_PAUSE:
        
        if not critical_battery:
            if low_battery:
                pwm = PWM(LED_B); pwm.freq(1000)
                for i in range(51):
                    pwm.duty_u16(int((i/50)*65535)); time.sleep(0.01)
                for i in range(50, -1, -1):
                    pwm.duty_u16(int((i/50)*65535)); time.sleep(0.01)
                LED_B.value(0)
            else:
                pwm = PWM(LED_G); pwm.freq(1000)
                for i in range(51):
                    pwm.duty_u16(int((i/50)*65535)); time.sleep(0.01)
                for i in range(50, -1, -1):
                    pwm.duty_u16(int((i/50)*65535)); time.sleep(0.01)
                LED_G.value(0)
        else:
            LED_R.value((current_time // 50) % 2)
            LED_G.value(0); LED_B.value(0)

        # **BUTTON PRESS → START IMMEDIATELY**
        if button_pressed():
            current_mode = MODE_WORK
            work_start_time = current_time
            led_all_off()
            duty = HALF_DUTY if low_battery else FULL_DUTY
            motor_go(DIR_FORWARD, duty)          # ← starts moving right now

    # --------------------------------------------------------------------- WORK
    else:   # MODE_WORK
        duty = HALF_DUTY if low_battery else FULL_DUTY

        # LED status
        if not critical_battery:
            led_blue() if low_battery else led_green()
        else:
            LED_R.value((current_time // 50) % 2)
            LED_G.value(0); LED_B.value(0)

        dist = measure_distance()
        if dist < OBSTACLE_DISTANCE:            # ---- OBSTACLE ----
            motor_stop();                time.sleep(0.1)
            motor_go(DIR_BACKWARD, duty); time.sleep(0.6)
            motor_stop();                time.sleep(0.1)
            motor_go(DIR_TURN_RIGHT, duty); time.sleep(0.2)
            motor_stop();                time.sleep(0.1)
            motor_go(DIR_FORWARD, duty)          # resume
        else:
            # keep forward 
            motor_go(DIR_FORWARD, duty)

     

# Final cleanup
motor_stop()
led_all_off()
