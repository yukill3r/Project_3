#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

const char* ssid = "Cirno";
const char* password = "bakabaka";

const int potPin = A0;
const int ledPin = D5;

double y = 0.0;
double u = 0.0;
double T = 0.5;
double dt = 0.02;
double alpha = dt / T;

double Kp = 1.2, Ki = 0.5, Kd = 0.1;
double setpoint = 0;
double integral = 0;
double last_error = 0;

unsigned long loopTime = 0;

ESP8266WebServer server(80);
LiquidCrystal_I2C lcd(0x3F, 16, 2);

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

const char INDEX_HTML[] PROGMEM = 
"<!DOCTYPE html><html><head><meta charset='UTF-8'>"
"<title>ESP8266 PID Control</title>"
"<script src='https://cdn.plot.ly/plotly-latest.min.js'></script>"
"<style>body{font-family:sans-serif; text-align:center; background:#f4f4f4; margin:20px;} "
".container{background:white; padding:20px; border-radius:10px; display:inline-block; box-shadow:0 2px 10px rgba(0,0,0,0.1);} "
"#chart{height:400px; width:800px;} .stats{margin: 15px 0; padding:10px; background:#e9ecef; border-radius:5px;} "
".val{font-weight:bold; color:#007bff; margin-right:15px;} input{width:60px; padding:5px; margin:5px;} "
"button{padding:6px 15px; cursor:pointer; background:#28a745; color:white; border:none; border-radius:3px;}</style></head>"
"<body><div class='container'><h2>System Regulacji PID</h2><div id='chart'></div>"
"<div class='stats'><b>Wartości:</b> SP: <span id='val_sp' class='val'>-</span> Y: <span id='val_y' class='val'>-</span> U: <span id='val_u' class='val'>-</span> | Loop: <span id='val_lt' class='val'>-</span>μs</div>"
"<div class='stats'><b>Nastawy:</b> Kp: <span id='cur_kp' class='val'>-</span> Ki: <span id='cur_ki' class='val'>-</span> "
"Kd: <span id='cur_kd' class='val'>-</span> dt: <span id='cur_dt' class='val'>-</span>s</div>"
"<div>Kp: <input type='number' id='kp' value='1.75' step='0.1'> Ki: <input type='number' id='ki' value='0.5' step='0.1'> "
"Kd: <input type='number' id='kd' value='0.0' step='0.1'> dt: <input type='number' id='dt' value='0.02' step='0.01'> "
"<button onclick='sendPID()'>Zapisz Nastawy</button></div></div>"
"<script>var traces=[{y:[],name:'SP'},{y:[],name:'Y'},{y:[],name:'U'}]; Plotly.newPlot('chart',traces,{margin:{t:20}});"
"function getData(){fetch('/data').then(r=>r.json()).then(d=>{Plotly.extendTraces('chart',{y:[[d.sp],[d.y],[d.u]]},[0,1,2],100);"
"document.getElementById('val_sp').innerText=d.sp.toFixed(1);document.getElementById('val_y').innerText=d.y.toFixed(1);document.getElementById('val_u').innerText=d.u.toFixed(1);document.getElementById('val_lt').innerText=d.lt;"
"document.getElementById('cur_kp').innerText=d.kp.toFixed(2);document.getElementById('cur_ki').innerText=d.ki.toFixed(2);document.getElementById('cur_kd').innerText=d.kd.toFixed(2);document.getElementById('cur_dt').innerText=d.dt.toFixed(3);});}"
"function sendPID(){var p='kp='+document.getElementById('kp').value+'&ki='+document.getElementById('ki').value+'&kd='+document.getElementById('kd').value+'&dt='+document.getElementById('dt').value;"
"fetch('/setpid?'+p);} setInterval(getData,100);</script></body></html>";

void handleData() {
    StaticJsonDocument<300> doc;
    doc["sp"] = setpoint; doc["y"] = y; doc["u"] = u;
    doc["kp"] = Kp; doc["ki"] = Ki; doc["kd"] = Kd; doc["dt"] = dt;
    doc["lt"] = loopTime;
    String json; serializeJson(doc, json);
    server.send(200, "application/json", json);
}

void handleSetPID() {
    if(server.hasArg("kp")) Kp = server.arg("kp").toDouble();
    if(server.hasArg("ki")) Ki = server.arg("ki").toDouble();
    if(server.hasArg("kd")) Kd = server.arg("kd").toDouble();
    if(server.hasArg("dt")) { dt = server.arg("dt").toDouble(); alpha = dt / T; }
    server.send(200, "text/plain", "OK");
}

void setup() {
    system_update_cpu_freq(160);
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    lcd.init(); lcd.backlight();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500);
    server.on("/", [](){ server.send(200, "text/html", INDEX_HTML); });
    server.on("/data", handleData);
    server.on("/setpid", handleSetPID);
    server.begin();
}

void loop() {
    static uint32_t lastMillis = 0;
    static uint32_t lastLcdUpdate = 0;

    if (millis() - lastMillis >= (dt * 1000)) {
        unsigned long start = micros();
        
        lastMillis = millis();
        setpoint = analogRead(potPin) / 1023.0 * 150.0;
        
        if (setpoint > 150) setpoint = 150;
        if (setpoint < 0) setpoint = 0;

        double error = setpoint - y;
        integral += error * dt;
        double derivative = (error - last_error) / dt;
        
        u = Kp * error + Ki * integral + Kd * derivative;
        
        if (u > 1023) u = 1023;
        if (u < 0) u = 0;
        
        last_error = error;
        y = updatePlant(u);
        
        if (y > 150) y = 150;
        if (y < 0) y = 0;

        analogWrite(ledPin, (int)u);

        loopTime = (micros() - start) / 1000;

        if (millis() - lastLcdUpdate >= 250) {
            lastLcdUpdate = millis();
            lcd.setCursor(0, 0);
            lcd.print("SP:"); lcd.print((int)setpoint); lcd.print(" Y:"); lcd.print((int)y);
            lcd.print("    ");
            lcd.setCursor(0, 1);
            lcd.print("U:"); lcd.print((int)u); lcd.print(" LT:"); lcd.print(loopTime);
            lcd.print("ms   ");
        }
    }
    server.handleClient();
}