import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import Jetson.GPIO as GPIO
from time import sleep

# Define GPIO Pins
f_pin = 13
b_pin = 16

dir_pin = 18
step_pin = 32

high = GPIO.HIGH
low = GPIO.LOW

# Motor Direction Constants
cw = 0
ccw = 1
pulse_t = 0.001
ppr = 800
gear_ratio = 4

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(f_pin, GPIO.OUT)
GPIO.setup(b_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)
GPIO.setup(step_pin, GPIO.OUT)


def forward():
    GPIO.output(f_pin, high)
    GPIO.output(b_pin, low)

def backward():
    GPIO.output(f_pin, low)
    GPIO.output(b_pin, high)

def stop():
    GPIO.output(f_pin, low)
    GPIO.output(b_pin, low)

def cw():
    GPIO.output(dir_pin, cw)
    GPIO.output(step_pin, high)
    sleep(pulse_t)
    GPIO.output(step_pin, low)
    sleep(pulse_t)

def ccw():
    GPIO.output(dir_pin, ccw)
    GPIO.output(step_pin, high)
    sleep(pulse_t)
    GPIO.output(step_pin, low)
    sleep(pulse_t)



class TractorMotorController(Node):
    def __init__(self):
        super().__init__('tractor_motor_controller_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.controller,
            10)
        self.subscription # Prevent unused variable warning

    def controller(self, msg):
        x = msg.linear.x
        z = msg.angular.z

        match x:
            case 1:
                forward()
                print("X: Forward")
            case -1:
                backward()
                print("X: Backwards")
            case _:
                stop()
                print("X: Stop")
        
        match z:
            case 1:
                cw()
                print("Z: Clockwise (Right)")
            case -1:
                ccw()
                print("Z: Counter-Clockwise (Left)")
            case _:
                print("Z: No command")

def main(args=None):
    rclpy.init(args=args)
    tractor_motor_controller_node = TractorMotorController()

    try:
        rclpy.spin(tractor_motor_controller_node)
    except:
        "Error: tractor_motor_controller_node has failed to spin."
    finally:
        tractor_motor_controller_node.destroy_node()
        rclpy.shutdown()

        #TODO: Clean up GPIO
        stop()
        GPIO.cleanup()
        

if __name__ == "__main__":
    main()

