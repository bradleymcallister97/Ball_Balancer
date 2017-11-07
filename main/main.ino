#include <Servo.h>

Servo servo;

#define RUN_TEST
#define RUN_PID

// Pin locations
const int servoPin = 9;
const int trigPin = 11;
const int echoPin = 10;

// Constants for PID algorithm
const float Kp = 8;
const float Ki = 6;
const float Kd = -5;

// Global variables
double middlePoint = 15;
double middlePoint_Upper = 1;
double middlePoint_Lower = 1;
double intError = 0;
const unsigned long pollingTime = 10;
float pollingSecs = pollingTime/1000;
unsigned long lastTimeCalculated = 0;
uint32_t lastOutput = 0;
double lastError = 0;
unsigned long last_read_time = 0;
float last_distance_cm = 0;

int counter = 0;
bool CountDown = false;

float calculate_ball_speed(void);
int getBallLocation(void);
int calculate(double input);
long sound_travel_time(void);

void loop(){
#ifdef RUN_TEST


#endif

#ifndef RUN_PID
    long now = millis();
    if (now - lastTimeCalculated >= pollingTime){
        int ballLocation = getBallLocation();
        int output = calculate(ballLocation);
        lastTimeCalculated = now;
        servo.write(output);
        Serial.println(ballLocation);
    }
#endif
   
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

void setup(){
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    Serial.begin(9600);
    servo.attach(servoPin);
    lastTimeCalculated = millis();
}

int getBallLocation(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    int duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    int distance = duration*0.034/2;
    if (distance < 0) distance = 51;
    if (distance > 51) distance = 51;
    return distance;
}

int calculate(double input){
    double error = middlePoint - input;
//    Serial.println(error);
    intError += error * pollingSecs;
    double derError = (error - lastError);

    double output = error * Kp + intError * Ki + derError * Kd;
    if (output > 25) output = 25;
    else if (output < 0) output = 0;

    if (abs(output - lastOutput) < 5) output = lastOutput;
    lastOutput = output;
    
    return output;
}

float calculate_ball_speed(void)
{
    unsigned long time_now = sound_travel_time();
    float distance_cm = time/29/2;
    float speed = (distance_cm - last_distance_cm) / (time_now - last_read_time);
    last_distance_cm = distance_cm;
    last_read_time = time_now;
    return speed;
}

long sound_travel_time(void)
{
    unsigned long now = millis();
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    uint32_t duration = pulseIn(echoPin, HIGH);
    last_read_time = now;
    if( duration == 0)
    {
        duration = 3000;
    }
    return duration;
}