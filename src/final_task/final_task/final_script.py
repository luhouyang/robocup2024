import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import final_task.process_image as proc

from openai import OpenAI
from dotenv import load_dotenv
import whisper
import json
import re
import os
import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
from gtts import gTTS
import time

import cv2
import cvzone
import math
from ultralytics import YOLO

# declare variables and functions
model = whisper.load_model("small")

# Load environment variables from the .env file
load_dotenv()

# Get the OpenAI API key from the .env file
openai_api_key = os.getenv("OPENAI_API_KEY")

# Set the API key for the OpenAI client
client = OpenAI(api_key=openai_api_key)

name_filter = ["my", "name", "is"]

medical_filter = ["my", "condition", "is"]


def record_audio(duration=5, sample_rate=44100):
    print("Recording... Please speak.")
    audio_data = sd.rec(int(duration * sample_rate),
                        samplerate=sample_rate,
                        channels=1,
                        dtype='float32')
    sd.wait()  # Wait until the recording is finished
    audio_data = np.squeeze(audio_data)  # Remove single-dimensional entries
    write("recorded_audio.wav", sample_rate, audio_data)  # Save as WAV file
    print("Recording complete. Saved as 'recorded_audio.wav'")
    return "recorded_audio.wav"


def transcribe_audio(audio_file_path):
    result = model.transcribe(audio_file_path, language='en')
    # print(result['text'])
    return result['text']


def confirm_choice(confirm_msg, deny_msg, action, confirmation_text, vartype):
    msgs = [confirm_msg, deny_msg]

    while True:

        announce(
            f'Say {confirm_msg} if your {vartype} is {confirmation_text}, Say {deny_msg} to reset your {vartype}'
        )
        audio_file = record_audio(duration=3)  # Record 5 seconds of audio
        transcribed_text = transcribe_audio(audio_file)

        # Match any name from the list
        res = [
            w for w in msgs if re.search(r'\b' + w + r'\b',
                                         transcribed_text,
                                         re.IGNORECASE)
        ]

        if res:
            if (res[0].lower() == confirm_msg.lower()):
                return
            elif (res[0].lower() == deny_msg.lower()):
                resvalue = action()
                confirmation_text = resvalue

        else:
            print("No valid response detected. Please try again.")


# Function to detect names and keep asking until a correct name is matched
def detect_name():
    global PERSON_NAME
    PERSON_NAME = ''
    while True:
        announce("Please say My Name is, followed by your name")
        audio_file = record_audio(duration=5)  # Record 5 seconds of audio
        transcribed_text = transcribe_audio(audio_file)

        # Match any name from the list
        words_list = [word.lower() for word in transcribed_text.split(sep=' ')]
        print(f'Names Text: {words_list}')

        for i in range(len(words_list) - 3):
            idx = 0
            for word in name_filter:
                if (words_list[i + idx] == word):
                    if (idx == len(name_filter) - 1):
                        for w in name_filter:
                            words_list.remove(w)
                        PERSON_NAME = str.join(' ', words_list)
                        print(f"Detected name: {PERSON_NAME}")
                        announce(f"Hello {PERSON_NAME}, glad to know you")
                        return PERSON_NAME
                    idx += 1

        if (PERSON_NAME == ''):
            print("No valid name detected. Please try again.")


def detect_medical():
    global MEDICAL_CONDITION
    MEDICAL_CONDITION = ''
    while True:
        announce(
            f'Please say my condition is, followed by your medical condition')
        audio_file = record_audio(duration=5)
        transcribed_text = transcribe_audio(audio_file)

        # Match any name from the list
        words_list = [word.lower() for word in transcribed_text.split(sep=' ')]
        print(f'Medical Conditions Text: {words_list}')

        for i in range(len(words_list) - 3):
            idx = 0
            for word in medical_filter:
                if (words_list[i + idx] == word):
                    if (idx == len(medical_filter) - 1):
                        for w in medical_filter:
                            words_list.remove(w)
                        MEDICAL_CONDITION = str.join(' ', words_list)
                        print(
                            f"Detected Medical Condition: {MEDICAL_CONDITION}")
                        announce(
                            f"Your medical condition is {MEDICAL_CONDITION}")
                        return MEDICAL_CONDITION
                    idx += 1

        if (MEDICAL_CONDITION == ''):
            print('No valid medical condition')


def announce(announcement_text):
    tts = gTTS(text=announcement_text, lang='en', slow=False)
    tts.save("final_audio.mp3")
    os.system("mpg321 final_audio.mp3")


# Function to detect names and keep asking until a correct name is matched
def detect_doctor():
    docter = ['yes']
    while True:

        announce('Say yes if you are a doctor')
        audio_file = record_audio(duration=3)  # Record 5 seconds of audio
        transcribed_text = transcribe_audio(audio_file)

        # Match any name from the list
        res = [
            w for w in docter if re.search(r'\b' + w + r'\b',
                                           transcribed_text,
                                           re.IGNORECASE)
        ]

        if res:
            announce(
                f"Hello there, my owner is in trouble. He has {MEDICAL_CONDITION} and is at risk of {str.join(' ', RISK_LIST)}. Help him I will pay you 1 billion ringit"
            )
            return
        else:
            print("No valid doctor detected. Please try again.")


class FinalScript(Node):

    def __init__(self):
        super().__init__('detect_person')

        self.get_logger().info('Starting final task')

        # Initialize YOLO model and class names
        self.model = YOLO('yolov8s.pt')
        self.classnames = []
        with open('classes.txt', 'r') as f:
            self.classnames = f.read().splitlines()

        # Publishers
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image,
                                                      "/image_tuning",
                                                      1)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)

        # fall detection
        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.fall_detected = True

        # movement
        self.move_img_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",  # Make sure this is the correct topic for your RealSense camera
            self.move_image_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.follow = True

        # Declare parameters
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.5)
        self.declare_parameter("forward_chase_speed", 0.5)
        self.declare_parameter("search_angular_speed", 0.25)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)

        # Retrieve parameters
        self.rcv_timeout_secs = self.get_parameter(
            "rcv_timeout_secs").get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter(
            "angular_chase_multiplier").get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter(
            "forward_chase_speed").get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter(
            "search_angular_speed").get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter(
            "max_size_thresh").get_parameter_value().double_value
        self.filter_value = self.get_parameter(
            "filter_value").get_parameter_value().double_value

        self.lastrcvtime = time.time() - 10000
        self.bridge = CvBridge()
        self.cmd_vel = Twist()
        self.previous_boxes = []

        self.robot_main_program()

    def robot_main_program(self):
        # Main program flow
        print('Starting Main Program')
        # time.sleep(5)
        announce('Your personal robot is starting')

        # Ask for name
        detect_name()
        confirm_choice(confirm_msg='yes',
                       deny_msg='no',
                       action=detect_name,
                       confirmation_text=PERSON_NAME,
                       vartype='name')

        # Ask for medical condition
        detect_medical()
        confirm_choice(confirm_msg='yes',
                       deny_msg='no',
                       action=detect_medical,
                       confirmation_text=MEDICAL_CONDITION,
                       vartype='medical condition')

        global RISK_LIST
        RISK_LIST = []
        while RISK_LIST == []:
            # Search for possible risk of the medical condition (no need format, limit to 10 words, as many condition without other text)
            try:
                medical_risk_res = client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=
                    [{
                        "role": "system",
                        "content": "You are a helpful assistant."
                    },
                     {
                         "role": "user",
                         "content": f"I have medical condition {MEDICAL_CONDITION}, please tell me what medical emergencies risk I might have, only give 5 most relevant risks and give no descriptions. FORMAT the risks in JSON format, the key is 'risks'"
                     }])
                response_content = medical_risk_res.choices[0].message.content
                response_dict = json.loads(response_content)
                print(response_content)
                RISK_LIST = response_dict.get("risks", [])
                print(RISK_LIST)

            except:
                print("FAILED TO PARSE RISKS RESPONSE")
            if RISK_LIST == []:
                detected_medical_condition = detect_medical()

        # # Take pictures of the person
        # announce('Face the camera for owner registration')

        # # Process picture, ask person to wait
        # announce('Please wait while I process the data')

        # announce(announcement_text=f'Processing is done, glad to know you {PERSON_NAME}')

        # Follow the person, detect if they fall down
        announce('You can do you things, I will assist in emergencies')
        self.fall_detected = False

        # # Call ambulance if they fall
        # announce('Oh no, owner is down, calling ambulance now')
        # os.system("mpg321 siren.mp3")
        # announce(
        #     f"Hello there, my owner is in trouble. Please come to the location U M Exam Hall now. He has {MEDICAL_CONDITION} and is at risk of {str.join(' ', RISK_LIST)}"
        # )

        # # Ask are you the doctor (IF get 'YES'), explain condition
        # detect_doctor()

        # # END

    def move_image_callback(self, data):
        if not self.follow:
            return

        try:
            # Convert the ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.get_logger().info(
                f"Image received with resolution: {cv_image.shape[1]}x{cv_image.shape[0]}"
            )
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        try:
            # Call your processing function to get bounding box (cx, cy, w, h)
            cx, cy, w, h = proc.find_bounding_boxes(cv_image)

            # Calculate control commands
            image_center_x = cv_image.shape[1] / 2
            image_center_y = cv_image.shape[0] / 2
            error_x = cx - image_center_x
            error_y = cy - image_center_y

            # linear_x = self.forward_chase_speed * (error_y / image_center_y)
            linear_x = self.forward_chase_speed * (cv_image.shape[0] - h -
                                                   20) / cv_image.shape[0]
            angular_z = -self.angular_chase_multiplier * (error_x /
                                                          image_center_x)

            # Publish velocity commands
            self.cmd_vel.linear.x = linear_x
            self.cmd_vel.angular.z = angular_z
            self.cmd_vel_pub.publish(self.cmd_vel)

        except Exception as e:
            self.get_logger().error(f"Failed during image processing: {e}")

    def image_callback(self, data):
        if self.fall_detected:
            # Publish velocity commands
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            return

        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        try:
            # Resize the image for processing
            frame = cv2.resize(cv_image, (980, 740))

            # YOLO object detection
            results = self.model(frame)

            detected_persons = []
            fall_confirmed = False
            for info in results:
                parameters = info.boxes
                for box in parameters:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    confidence = box.conf[0]
                    class_detect = int(box.cls[0])
                    class_detect = self.classnames[class_detect]
                    conf = math.ceil(confidence * 100)

                    # Calculate height and width of the bounding box
                    height = y2 - y1
                    width = x2 - x1

                    # Detect person and fall condition
                    if conf > 80 and class_detect == 'person':
                        detected_persons.append((x1,
                                                 y1,
                                                 x2,
                                                 y2,
                                                 height,
                                                 width))

                        # Check if the bounding box is likely a fall with a refined threshold
                        if width > height and (width / height > 1.5
                                               ):  # Adjust threshold as needed
                            cvzone.putTextRect(frame,
                                               'Fall Detected',
                                               [x1 + 8,
                                                y1 - 50],
                                               thickness=2,
                                               scale=2)

                        # Draw bounding box and label
                        cvzone.cornerRect(frame,
                                          [x1,
                                           y1,
                                           width,
                                           height],
                                          l=30,
                                          rt=6)
                        cvzone.putTextRect(frame,
                                           f'{class_detect}',
                                           [x1 + 8,
                                            y1 - 12],
                                           thickness=2,
                                           scale=2)

            # Validate detected fall by checking overlap with previous detections
            if self.previous_boxes:
                for (x1_prev,
                     y1_prev,
                     x2_prev,
                     y2_prev,
                     _,
                     _) in self.previous_boxes:
                    for (x1, y1, x2, y2, height, width) in detected_persons:
                        if self.is_overlapping(x1_prev,
                                               y1_prev,
                                               x2_prev,
                                               y2_prev,
                                               x1,
                                               y1,
                                               x2,
                                               y2):
                            if width > height and (
                                    width / height
                                    > 1.5):  # Adjust threshold as needed
                                cvzone.putTextRect(frame,
                                                   'Confirmed Fall',
                                                   [x1 + 8,
                                                    y1 - 100],
                                                   thickness=2,
                                                   scale=2)
                                fall_confirmed = True

            self.previous_boxes = detected_persons

            # Handle fall detection and unsubscribe if necessary
            if fall_confirmed:
                self.fall_detected = True
                self.follow = False
                self.get_logger().info('Fall detected')

                # Call ambulance if they fall
                announce('Oh no, owner is down, calling ambulance now')
                os.system("mpg321 siren.mp3")
                announce(
                    f"Hello there, my owner is in trouble. Please come to the location U M Exam Hall now. He has {MEDICAL_CONDITION} and is at risk of {str.join(' ', RISK_LIST)}"
                )

                # Ask are you the doctor (IF get 'YES'), explain condition
                detect_doctor()

                # END

            # Display the frame (optional for debugging)
            cv2.imshow("Live Fall Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed during image processing: {e}")

    def is_overlapping(self,
                       x1_prev,
                       y1_prev,
                       x2_prev,
                       y2_prev,
                       x1,
                       y1,
                       x2,
                       y2):
        # Check if bounding boxes overlap
        return not (x2 < x1_prev or x1 > x2_prev or y2 < y1_prev
                    or y1 > y2_prev)


def main(args=None):
    rclpy.init(args=args)
    final_script = FinalScript()

    try:
        rclpy.spin(final_script)
    except KeyboardInterrupt:
        pass

    # Destroy node when finished
    final_script.destroy_node()
    rclpy.shutdown()
