#include <Servo.h>

Servo servo;

// #define RUN_TEST
#define RUN_PID

// Pin locations
const int servoPin = 5;
const int trigPin = 11;
const int echoPin = 10;

// SET 1
const float Kp = 5;
const float Ki = 3;
const float Kd = -3.3;

// SET 2
// const float Kp = 4.9;
// const float Ki = 2.9;
// const float Kd = -2.15;

// Constants for PID algorithm
// const float Kp = 6;
// const float Ki = 4.75;
// const float Kd = -3;

// Global variables
float setPoint = 15;
float intError = 0;
const unsigned long pollingTime = 10;
float pollingSecs = pollingTime/1000;
unsigned long lastTimeCalculated = 0;
uint32_t lastOutput = 0;
float lastError = 0;
unsigned long last_read_time = 0;
float last_distance_cm = 0;
uint32_t dummyData = 0;

// UART Variables
uint32_t Error = 0;
uint32_t Sensor = 0;
uint32_t PWM = 0;
bool End = 0;

// Global Test Variables 
int counter = 0;
bool CountDown = false;

float calculate_ball_speed(void);
int getBallLocation(void);
int calculate(double input);
uint32_t sound_travel_time(void);

void setup(){
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    Serial.begin(9600);
    servo.attach(servoPin);
    lastTimeCalculated = millis();
    End = 0;
}

void loop(){
#ifdef RUN_TEST
    dummyData = sound_travel_time();
    Serial.println(dummyData);
    delay(100);

#endif

#ifdef RUN_PID
    uint32_t now = millis();
    if (now - lastTimeCalculated >= pollingTime){
        int ballLocation = getBallLocation();
        Sensor = ballLocation;
        int output = calculate(ballLocation);
        PWM = output;
        lastTimeCalculated = now;
        servo.write(output);
        // Serial.println(ballLocation);
    }

    if(lastTimeCalculated > 10000)
        End = 1;
    
    Serial.print(Error);
    Serial.print(",");
    Serial.print(Sensor);
    Serial.print(",");
    Serial.print(PWM);
    Serial.print(",");
    Serial.print(lastTimeCalculated);
    Serial.print(",");
    Serial.print(End);
    Serial.print(",");
    Serial.println("E");

#endif

}

int getBallLocation(){
    int duration = getSonarValue();
    // Calculating the distance
    int distance = duration*0.034/2;
    if (distance < 0) distance = 51;
    if (distance > 51) distance = 51;
    return distance;
}

int getSonarValue(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    return pulseIn(echoPin, HIGH);
}

int calculate(int input){
    float error = setPoint - input;
    Error = error;
//    Serial.println(error);
    intError += error * pollingSecs;
    float derError = (error - lastError);

    float output = error * Kp + intError * Ki + derError * Kd;
    if (output > 33) output = 33;
    else if (output < 0) output = 0;

    if (abs(output - lastOutput) < 5) output = lastOutput;
    lastOutput = output;
    
    return output;
}

float calculate_ball_speed()
{
    unsigned long time_now = sound_travel_time();
    float distance_cm = time_now/29/2;
    float speed = (distance_cm - last_distance_cm) / (time_now - last_read_time);
    last_distance_cm = distance_cm;
    last_read_time = time_now;
    return speed;
}

uint32_t sound_travel_time()
{
    uint32_t duration = getSonarValue();
    last_read_time = millis();
    if( duration == 0)
    {
        duration = 3000;
    }
    return duration;
}

void inc_dec_counter(void)
{
    if(CountDown)
        counter--;
    else
        counter++;
}

void check_counter_state(void)
{
    if(counter == 25)
        CountDown = true;
    else if(counter ==0)
        CountDown = false;
}
