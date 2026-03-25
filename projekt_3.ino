#include <Arduino.h>

const int potPin = A0;
const int ledPin = D1;

double y = 0.0;
double T = 0.5;
double dt = 0.02;
double alpha = dt / T;

double Kp = 1.0;
double Ki = 0.5;
double Kd = 0.0;

double setpoint = 0;
double integral = 0;
double last_error = 0;

double f(double u) {
    if (u < 200) return 0.03 * u;
    if (u < 700) return 0.25 * u - 40;
    return 140 + 0.05 * (u - 700);
}

double updatePlant(double u) {
    double target = f(u);
    y = y + alpha * (target - y);
    return y;
}

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
}

void loop() {
    static uint32_t lastMillis = 0;
    if (millis() - lastMillis >= (dt * 1000)) {
        lastMillis = millis();

        setpoint = analogRead(potPin) / 1023.0 * 150.0;

        double error = setpoint - y;
        integral += error * dt;
        double derivative = (error - last_error) / dt;
        
        double u = Kp * error + Ki * integral + Kd * derivative;
        
        if (u > 1023) u = 1023;
        if (u < 0) u = 0;
        
        last_error = error;

        double currentY = updatePlant(u);

        analogWrite(ledPin, (int)u);

        Serial.print("Setpoint:"); Serial.print(setpoint); Serial.print(",");
        Serial.print("Output_Y:"); Serial.print(currentY); Serial.print(",");
        Serial.print("Control_U:"); Serial.println(u / 5.0);
    }
}
