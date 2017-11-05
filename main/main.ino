#include <Servo.h>

Servo servo;

#define RUNTEST

// Pin locations
const int servoPin = 9;
const int trigPin = 11;
const int echoPin = 10;

// Constants for PID algorithm
const float Kp = 4;
const float Ki = 2;
const float Kd = -1;

// Global variables
double middlePoint = 15;
double intError = 0;
const long pollingTime = 30;
float pollingSecs = pollingTime/1000;
long lastTimeCalculated = 0;
int lastOutput = 0;
double lastError = 0;

int counter = 0;
bool CountDown = false;

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
    if (distance < 0) distance = 20;
    if (distance > 20) distance = 20;
    return distance;
}

int calculate(double input){
    double error = middlePoint - input;
    intError += error * pollingSecs;
    double derError = (error - lastError);

    double output = error * Kp + intError * Ki + derError * Kd;
    if (output > 25) output = 25;
    else if (output < 0) output = 0;

    if (abs(output - lastOutput) < 4) output = lastOutput;
    lastOutput = output;
    
    return output;
}

void loop(){
//    if(counter == 25)
//      CountDown = true;
//    else if(counter == 0)
//      CountDown = false;
//    servo.write(counter);
//    //analogWrite(servoPin,counter);
//    if(CountDown)
//      counter--;
//    else
//      counter++;
//    Serial.println(counter);
//    delay(50);
    long now = millis();
    if (now - lastTimeCalculated >= pollingTime){
        int ballLocation = getBallLocation();
        int output = calculate(ballLocation);
        lastTimeCalculated = now;
        servo.write(output);
        Serial.println(output);
    }
}

