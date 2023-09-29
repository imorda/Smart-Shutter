//======================================================
//Choose your device type
//======================================================
//#define useServo //Uncomment this if you use Servo motor
#define useDC //Uncomment this if you use DC motor
#define Three-Pos //Uncomment this if you want to use 3 shutter positions (default: 2)

#ifdef useDC
#define ENAdc_pin 4 //DC motor driver power control (must be PWM-capable)
#define IN1dc_pin 43 //DC motor driver direction forward mode pin
#define IN2dc_pin 44 //DC motor driver direction backward mode pin
#define EncoderA 2 //DC motor encoder pin A (Must be Interrupt-capable)
#define InterruptNumber 0 //Interrupt number of EncoderA pin
#define EncoderB 3 //DC motor encoder pin B
#endif
#ifdef useServo
#define servo_pin 7  //Servo motor signal pin number (must be PWM-capable)
#endif
#define rightbtn_pin 51 //Right-arrow button pin (uses builtin PULLUP). Connect your button to GND
#define leftbtn_pin 49 //Left-arrow button pin (uses builtin PULLUP). Connect your button to GND
#define setupbtn_pin 50 //Setup button pin (uses builtin PULLUP). Connect your button to GND

#ifdef useServo
#define ServoRange 225 //Type here your servo rotational range (degrees)
#endif
#ifdef useDC
#define PPR 748.44 //Type here your DC motor PPR (Encoder Pulse Per Rotation)
#endif


//======================================================
//Adjust some kinematic settings
//======================================================
#ifdef useServo
const int maxusedspeed = 180; //Max speed that is used while rotating (deg/sec)
const int acceleration = 35; //Acceleration that is used while accelerating and decelerating while rotating (deg/sec^2)
#endif
const short tolerance = 1; //Maximum motor deviation (degrees)

//======================================================
//Debug mode
//======================================================
//#define Debug //Uncomment this if you want your device to send its rotation speed and position to UART
