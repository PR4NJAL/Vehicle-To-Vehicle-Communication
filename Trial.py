import cv2
import numpy as np
import serial
import threading
from threading import Timer, Thread, Event
import RPi.GPIO as GPIO
import time
import sys
import Adafruit_CharLCD as LCD
import image_dehazer
import pyrebase
import os
import datetime
from PIL import Image

os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/local/lib/python3.9/dist-packages/cv2/qt/plugins'

ser = serial.Serial('/dev/ttyUSB0', 9600)
lcd = LCD.Adafruit_CharLCD(rs=18, en=23, d4=24, d5=25, d6=8, d7=7, cols=16, lines=2)

# Load YOLO model (keeping your existing YOLOv3)
Loc = "/home/pi/Desktop/Vehicle To Vehicle Communication/"
net = cv2.dnn.readNet(Loc + "yolov3.weights", Loc + "yolov3.cfg")
layer_names = net.getLayerNames()
unconnected_layers = net.getUnconnectedOutLayers()
unconnected_layers = unconnected_layers.reshape(-1)
output_layers = [layer_names[i - 1] for i in unconnected_layers]

# Load class names for YOLO
with open(Loc + "coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Animal class IDs in COCO dataset (same as YOLOv3)
animal_classes = [16, 17, 18, 19, 20, 21, 22, 23]  # bird, cat, dog, horse, sheep, cow, elephant, bear

# Initialize global variables
frame = None
detection_result = None
animal_detected = False

# class which creates a resettable timer as a thread
class ResetTimer(object):
    def __init__(self, time, function, daemon=None):
        self.__time = time
        self.__function = function
        self.__set()
        self.__running = False
        self.__killed = False
        self.__daemon = daemon
        self.__thread = None

    def __set(self):
        self.__timer = Timer(self.__time, self.__function)

    def stop(self):
        self.__daemon = True

    def run(self):
        self.__running = True
        self.__timer = Timer(self.__time, self.__function)
        self.__thread = Thread(target=self.__timer.start)
        self.__thread.start()
        if self.__daemon == True:
            sys.exit(0)

    def cancel(self):
        self.__running = False
        self.__timer.cancel()

    def reset(self, start=False):
        if self.__running:
            self.cancel()
        self.__set()
        if self.__running or start:
            self.run()

# method that counts how often the light barrier is triggered
def count(self):
    global counter
    global car_stop
    counter = counter + 1

# method for calculating / displaying of rotations
def output():
    global counter
    global ldr_value
    global car_stop
    global accident_detected
    global animal_detected
    
    timer.cancel()  # stopping the timer
    speed = int(((counter / 2) * calc) / wheel)  # calculating rotations per minute
    lcd.clear()
    lcd.message("RPM: " + str(speed) + "\nLDR: " + str(ldr_value))  # Display on LCD

    # Check if RPM sensor is stable for 10 seconds
    if speed == 0:
        car_stop += 1
    else:
        car_stop = 0
    
    print("Accident Status : ", accident_detected)
    print("Animal Status : ", animal_detected)
    
    # Priority: Animal detection first (highest priority)
    if animal_detected:
        animal_detected = False  # Reset flag
        lcd.clear()
        lcd.message("Status:\nAnimal Detected")
        mes_send("I")  # New code for animal detection
    elif car_stop >= 3 and ldr_value > 0 and accident_detected == False:
        lcd.clear()
        lcd.message("Status:\nCar is Still")
        mes_send("C")
    elif car_stop < 3 and ldr_value == 0 and accident_detected == False:
        lcd.clear()
        lcd.message("Status:\nDim Light")
        mes_send("B")
    elif car_stop < 3 and ldr_value == 1 and accident_detected == True:
        accident_detected = False
        lcd.clear()
        lcd.message("Status:\nAccident Occur")
        mes_send("A")
    elif car_stop >= 3 and ldr_value == 0 and accident_detected == False:
        lcd.clear()
        lcd.message("Car is Still\nDim Light")
        mes_send("D")
    elif car_stop < 3 and ldr_value == 0 and accident_detected == True:
        accident_detected = False
        lcd.clear()
        lcd.message("Dim Light\nAccident Occur")
        mes_send("E")
    elif car_stop >= 3 and ldr_value > 0 and accident_detected == True:
        accident_detected = False
        lcd.clear()
        lcd.message("Car is Still\nAccident Occur")
        mes_send("F")
    elif car_stop >= 3 and ldr_value == 1 and accident_detected == True:
        accident_detected = False
        lcd.clear()
        lcd.message("Car Status:Still \nAccident,Dim Lgt")
        mes_send("G")

    counter = 0  # resetting the counter
    timer.reset()  # resetting the timer
    timer.run()  # restart timer

# method to read LDR value and display messages based on LDR value
def read_ldr():
    global ldr_value

    timer_reset_ldr.cancel()  # Cancel reset timer
    ldr_value = GPIO.input(ldr_pin)
    timer_reset_ldr.reset()  # Reset reset timer

    timer_reset_ldr.run()  # Restart reset timer
    timer_read_ldr.run()  # Restart timer for LDR reading

# Optimized detection function for Raspberry Pi
def detect_accident_and_animals():
    global frame, detection_result, accident_detected, animal_detected
    detection_counter = 0
    
    while True:
        if frame is not None:
            # Skip frames to reduce processing load (process every 3rd frame)
            detection_counter += 1
            if detection_counter % 3 != 0:
                time.sleep(0.1)
                continue

            # Detect fog first (lightweight operation)
            fogDetected = detectFog(frame)

            # Resize frame for faster processing
            small_frame = cv2.resize(frame, (320, 320))  # Smaller than original 416x416
            
            blob = cv2.dnn.blobFromImage(
                small_frame, 0.00392, (320, 320), (0, 0, 0), True, crop=False
            )
            net.setInput(blob)
            outs = net.forward(output_layers)

            class_ids = []
            confidences = []
            boxes = []
            cars = []
            animals = []

            # Scale factor for coordinates (since we resized the frame)
            scale_x = frame.shape[1] / 320
            scale_y = frame.shape[0] / 320

            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]

                    if confidence > 0.5:
                        # Object detected
                        center_x = int(detection[0] * small_frame.shape[1])
                        center_y = int(detection[1] * small_frame.shape[0])
                        w = int(detection[2] * small_frame.shape[1])
                        h = int(detection[3] * small_frame.shape[0])

                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        # Scale back to original frame size
                        x = int(x * scale_x)
                        y = int(y * scale_y)
                        w = int(w * scale_x)
                        h = int(h * scale_y)

                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
                        
                        # Separate cars and animals
                        if class_id == 2:  # car
                            cars.append([x, y, w, h])
                        elif class_id in animal_classes:  # animals
                            animals.append([x, y, w, h])
                            animal_detected = True
                            print(f"Animal detected: {classes[class_id]} (confidence: {confidence:.2f})")

            # Car accident detection (only if we have multiple cars)
            if len(cars) >= 2:
                for i in range(len(cars)):
                    x1, y1, w1, h1 = cars[i]
                    center_x1 = x1 + w1 // 2
                    center_y1 = y1 + h1 // 2
                    for j in range(i + 1, len(cars)):
                        x2, y2, w2, h2 = cars[j]
                        center_x2 = x2 + w2 // 2
                        center_y2 = y2 + h2 // 2

                        distance = np.sqrt(
                            (center_x1 - center_x2) ** 2
                            + (center_y1 - center_y2) ** 2
                        )
                        if distance < 100:
                            detection_result = "Accident Detected"
                            accident_detected = True
                        else:
                            detection_result = "No Accident"
                            if not animal_detected:  # Don't override animal detection
                                accident_detected = False
            else:
                detection_result = "No Accident"
                if not animal_detected:  # Don't override animal detection
                    accident_detected = False

            # Handle fog detection
            if fogDetected:
                lcd.clear()
                lcd.message("Status:\nFog Detected")
                mes_send("H")
                os.makedirs(Loc+"Foggy Images", exist_ok=True)
                os.makedirs(Loc+"Dehazed Images", exist_ok=True)

                timecr = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                img_path = Loc+f"Foggy Images/foggyimage_{timecr}.jpg"
                cv2.imwrite(img_path, frame)
                saving_result_path = Loc+"Dehazed Images/result_{timecr}.jpg"
                result_path = dehazeImage(input_image_path=img_path, saved_path=saving_result_path)
                sendToDatabase(file_path=result_path, firebaseClient=firebase)
                
        time.sleep(0.1)  # Small delay to prevent excessive CPU usage

def find_available_camera():
    for i in range(11):
        cap = cv2.VideoCapture(10-i)
        if cap.isOpened():
            print(f"Camera found at index: {10-i}")
            return cap, 10-i
    return None, -1

# Function for video capture and preview
def capture_video():
    global frame, detection_result
        
    cap, cam_index = find_available_camera()
    if cap is None:
        print("No camera found!")
        return
        
    # Set lower resolution for better performance on Raspberry Pi
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 15)  # Lower FPS for better performance
    
    while True:
        ret, frame = cap.read()
        if ret:
            # Display current detections on frame
            if detection_result:
                cv2.putText(
                    frame,
                    detection_result,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
            
            if animal_detected:
                cv2.putText(
                    frame,
                    "ANIMAL DETECTED",
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                
            cv2.imshow("Vehicle Monitoring System", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()

def mes_send(mes):
    try:
        ser.write(str.encode(mes))
        print("Serial Message Sent ", mes)
    except Exception as e:
        print(e)

def slow_horizontal_variance(im):
    """Return average variance of horizontal lines of a grayscale image"""
    width, height = im.size
    if not width or not height:
        return 0
    vars = []
    pix = im.load()
    for y in range(height):
        row = [pix[x, y] for x in range(width)]
        mean = sum(row) / width
        variance = sum([(x - mean) ** 2 for x in row]) / width
        vars.append(variance)
    return sum(vars) / height

def detectFog(frame):
    fogThres = 950
    im = Image.fromarray(frame).convert("L")
    var = slow_horizontal_variance(im)
    fog = var < fogThres  # FOG THRESHOLD
    print("%5.0f - %5s" % (var, fog and "FOGGY" or "SHARP"))
    return fog

def dehazeImage(input_image_path, saved_path):
    HazeImg = cv2.imread(input_image_path)
    HazeCorrectedImg, HazeMap = image_dehazer.remove_haze(
        HazeImg, showHazeTransmissionMap=False
    )
    cv2.imwrite(saved_path, HazeCorrectedImg)
    return saved_path

def sendToDatabase(file_path, firebaseClient):
    storage = firebaseClient.storage()
    now = datetime.datetime.now()
    formatted_time = now.strftime("%I:%M:%S %p")
    print("Current time in 12-hour format:", formatted_time)
    file_name = f"{formatted_time} dehaze-image"
    storage.child(file_name).put(file_path)
    image_url = storage.child(file_name).get_url(None)
    print("URL of the uploaded image:", image_url)

# Firebase configuration
config = {
    "apiKey": "AIzaSyCiIyexhUK7NxLR4MOmFcCVvuNm-jcphxY",
    "authDomain": "dehazeimage.firebaseapp.com",
    "projectId": "dehazeimage",
    "storageBucket": "dehazeimage.appspot.com",
    "messagingSenderId": "660004190976",
    "appId": "1:660004190976:web:2888503f98ac0d902411e5",
    "measurementId": "G-3GERC4P50Q",
    "serviceAccount": Loc+"serviceAccount.json",
    "databaseURL": "https://dehazeimage-default-rtdb.firebaseio.com/",
}

firebase = pyrebase.initialize_app(config=config)

# Create and start threads for camera and detection
capture_thread = threading.Thread(target=capture_video)
detect_thread = threading.Thread(target=detect_accident_and_animals)

capture_thread.start()
detect_thread.start()

# Rest of your existing code
counter = 0
car_stop = 0
ldr_pin = 21
speed_sensor_pin = 20
interval = 10.0
calc = 60 / int(interval)
wheel = 20
accident_detected = False

GPIO.setmode(GPIO.BCM)
GPIO.setup(speed_sensor_pin, GPIO.IN)
GPIO.setup(ldr_pin, GPIO.IN)

# Create timer objects for LDR reading and messages
timer_read_ldr = ResetTimer(1.0, read_ldr)
timer_reset_ldr = ResetTimer(10.0, lambda: None)
timer = ResetTimer(interval, output)

# Start the threads
ldr_thread = threading.Thread(target=timer_read_ldr.run)
ldr_thread.start()

try:
    GPIO.add_event_detect(speed_sensor_pin, GPIO.FALLING, count)
    timer.run()
    while True:
        pass
except KeyboardInterrupt:
    timer.stop()
    ldr_thread.join()
    timer.join()
    GPIO.cleanup()
    lcd.clear()
    ser.close()