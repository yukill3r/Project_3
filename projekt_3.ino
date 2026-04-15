#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

const char* ssid = "Cirno";
const char* password = "bakabaka";

const int potPin = A0;
const int ledPin = D5;

double y = 0.0;
double T = 0.5;
double u = 0;
double Kp = 1.0, Ki = 0.5, Kd = 0.0;
double setpoint = 0, integral = 0, last_error = 0;

// Diagnostyka (ujednolicona do ms)
double currentLoopTimeMs = 0;
double currentRealDtMs = 0;
unsigned long lastProcessMicros = 0;

ESP8266WebServer server(80);
LiquidCrystal_I2C lcd(0x3F, 16, 2);

double f(double u_in) {
    if (u_in < 200) return 0.03 * u_in;
    if (u_in < 700) return 0.25 * u_in - 40;
    return 140 + 0.05 * (u_in - 700);
}

void updatePlant(double u_in, double dt_sec) {
    double target = f(u_in);
    double alpha = dt_sec / (T + dt_sec);
    y = y + alpha * (target - y);
}

const char INDEX_HTML[] PROGMEM = 
"<!DOCTYPE html><html><head><meta charset='UTF-8'>"
"<title>ESP8266 PID Control</title>"
"<script src='https://cdn.plot.ly/plotly-latest.min.js'></script>"
"<style>"
"  body{font-family:sans-serif; text-align:center; background:#f4f4f4; margin:20px;}"
"  .container{background:white; padding:20px; border-radius:10px; display:inline-block; box-shadow:0 2px 10px rgba(0,0,0,0.1);}"
"  #chart{height:400px; width:800px;}"
"  .stats{margin: 15px 0; padding:10px; background:#e9ecef; border-radius:5px;}"
"  .val{font-weight:bold; color:#007bff; margin-right:15px;}"
"  input{width:60px; padding:5px; margin:5px;}"
"  button{padding:6px 15px; cursor:pointer; background:#28a745; color:white; border:none; border-radius:3px;}"
"</style></head>"
"<body>"
"  <div class='container'>"
"    <h2>System Regulacji PID</h2>"
"    <div id='chart'></div>"
"    <div class='stats'>"
"      <b>Nastawy:</b> "
"      Kp: <span id='cur_kp' class='val'>-</span>"
"      Ki: <span id='cur_ki' class='val'>-</span>"
"      Kd: <span id='cur_kd' class='val'>-</span>"
"      | Loop: <span id='cur_lt' class='val'>-</span> ms"
"    </div>"
"    <div>"
"      Kp: <input type='number' id='kp' value='1.0' step='0.1'>"
"      Ki: <input type='number' id='ki' value='0.5' step='0.1'>"
"      Kd: <input type='number' id='kd' value='0.0' step='0.1'>"
"      <button onclick='sendPID()'>Zapisz Nastawy</button>"
"    </div>"
"  </div>"
"<script>"
"var traces=["
" {y:[],name:'Setpoint (SP)',line:{color:'red'}},"
" {y:[],name:'Wyjście (Y)',line:{color:'blue'}},"
" {y:[],name:'Sterowanie (U)',line:{color:'green',dash:'dot'}},"
" {y:[],name:'Loop [ms]',line:{color:'orange'}},"
" {y:[],name:'dt [ms]',line:{color:'black',width:1}}"
"];"
"Plotly.newPlot('chart',traces,{margin:{t:20}});"
"function getData(){"
" fetch('/data').then(r=>r.json()).then(d=>{"
"  Plotly.extendTraces('chart',{y:[[d.sp],[d.y],[d.u],[d.lt],[d.rdt]]},[0,1,2,3,4],100);"
"  document.getElementById('cur_kp').innerText = d.kp.toFixed(2);"
"  document.getElementById('cur_ki').innerText = d.ki.toFixed(2);"
"  document.getElementById('cur_kd').innerText = d.kd.toFixed(2);"
"  document.getElementById('cur_lt').innerText = d.lt.toFixed(2);"
" });"
"}"
"function sendPID(){"
" var p='kp='+document.getElementById('kp').value+'&ki='+document.getElementById('ki').value+'&kd='+document.getElementById('kd').value;"
" fetch('/setpid?'+p);"
"}"
"setInterval(getData,100);"
"</script></body></html>";

void handleData() {
    StaticJsonDocument<300> doc;
    doc["sp"] = setpoint;
    doc["y"] = y;
    doc["u"] = u;
    doc["kp"] = Kp;
    doc["ki"] = Ki;
    doc["kd"] = Kd;
    doc["lt"] = currentLoopTimeMs;   // Czas pętli w ms
    doc["rdt"] = currentRealDtMs;    // Realne dt w ms
    
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
    system_update_cpu_freq(160); 
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    lcd.init(); lcd.backlight();
    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    
    server.on("/", [](){ server.send(200, "text/html", INDEX_HTML); });
    server.on("/data", handleData);
    server.on("/setpid", handleSetPID);
    server.begin();
    lastProcessMicros = micros();
}

void loop() {
    uint32_t loopStart = micros();
    server.handleClient();

    if (micros() - lastProcessMicros >= 20000) {
        uint32_t nowMicros = micros();
        // Obliczamy realne dt (sekundy na potrzeby modelu, ms na wykres)
        double realDtSec = (nowMicros - lastProcessMicros) / 1000000.0;
        currentRealDtMs = realDtSec * 1000.0;
        lastProcessMicros = nowMicros;

        setpoint = analogRead(potPin) / 1023.0 * 150.0;
        double error = setpoint - y;
        double p_term = Kp * error;
        double i_term_next = integral + (error * realDtSec);
        double d_term = (error - last_error) / realDtSec;
        
        double u_unsat = p_term + (Ki * i_term_next) + (Kd * d_term);
        u = u_unsat;
        if (u > 1023) u = 1023; if (u < 0) u = 0;
        if (u == u_unsat) integral = i_term_next;
        last_error = error;

        updatePlant(u, realDtSec);
        analogWrite(ledPin, (int)u);
    }

    static uint32_t lastLcd = 0;
    if (millis() - lastLcd >= 500) {
        lastLcd = millis();
        lcd.setCursor(0,0);
        lcd.print("S:"); lcd.print((int)setpoint);
        lcd.print(" Y:"); lcd.print(y,1);
    }

    // Obliczanie czasu trwania loopa w ms
    currentLoopTimeMs = (micros() - loopStart) / 1000.0;
}