#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

const char* ssid = "biot";
const char* password = "12345678";

ESP8266WebServer server(80);

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
double u = 0;

double f(double u_in) {
    if (u_in < 200) return 0.03 * u_in;
    if (u_in < 700) return 0.25 * u_in - 40;
    return 140 + 0.05 * (u_in - 700);
}

double updatePlant(double u_in) {
    double target = f(u_in);
    y = y + alpha * (target - y);
    return y;
}

const char INDEX_HTML[] PROGMEM = 
"<!DOCTYPE html><html><head><meta charset='UTF-8'>"
"<script src='https://cdn.plot.ly/plotly-latest.min.js'></script>"
"<style>body{font-family:sans-serif;text-align:center;} #chart{height:400px;}</style></head>"
"<body><div id='chart'></div>"
"<div>Kp: <input type='number' id='kp' value='1.0' step='0.1'>"
"Ki: <input type='number' id='ki' value='0.5' step='0.1'>"
"Kd: <input type='number' id='kd' value='0.0' step='0.1'>"
"<button onclick='sendPID()'>Zapisz</button></div>"
"<script>"
"var traces=[{y:[],name:'SP'},{y:[],name:'Y'},{y:[],name:'U'}];"
"Plotly.newPlot('chart',traces);"
"function getData(){"
" fetch('/data').then(r=>r.json()).then(d=>{"
"  Plotly.extendTraces('chart',{y:[[d.sp],[d.y],[d.u]]},[0,1,2],100);"
" });"
"}"
"function sendPID(){"
" var p='kp='+document.getElementById('kp').value+'&ki='+document.getElementById('ki').value+'&kd='+document.getElementById('kd').value;"
" fetch('/setpid?'+p);"
"}"
"setInterval(getData,100);"
"</script></body></html>";

// --- HANDLERY SERWERA ---
void handleRoot() { server.send(200, "text/html", INDEX_HTML); }

void handleData() {
    StaticJsonDocument<128> doc;
    doc["sp"] = setpoint;
    doc["y"] = y;
    doc["u"] = u / 6.8; 
    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
}

void handleSetPID() {
    if(server.hasArg("kp")) Kp = server.arg("kp").toDouble();
    if(server.hasArg("ki")) Ki = server.arg("ki").toDouble();
    if(server.hasArg("kd")) Kd = server.arg("kd").toDouble();
    server.send(200, "text/plain", "OK");
}

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nIP: " + WiFi.localIP().toString());

    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/setpid", handleSetPID);
    server.begin();
}

void loop() {
    server.handleClient();

    static uint32_t lastMillis = 0;
    if (millis() - lastMillis >= (uint32_t)(dt * 1000)) {
        lastMillis = millis();

        setpoint = analogRead(potPin) / 1023.0 * 150.0;

        double error = setpoint - y;
        double p_term = Kp * error;
        double i_term = integral + (error * dt);
        double d_term = (error - last_error) / dt;
        
        double u_unsat = p_term + (Ki * i_term) + (Kd * d_term);
        u = u_unsat;
        
        if (u > 1023) u = 1023;
        if (u < 0) u = 0;
        
        if (u == u_unsat) integral = i_term;

        last_error = error;

        updatePlant(u);
        analogWrite(ledPin, (int)u);

        Serial.print("Setpoint:"); Serial.print(setpoint); Serial.print(",");
        Serial.print("Output_Y:"); Serial.print(y); Serial.print(",");
        Serial.print("Control_U:"); Serial.println(u / 6.8);
    }
}