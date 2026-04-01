#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>

const char* ssid = "x";
const char* password = "x";

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
"  button:hover{background:#218838;}"
"</style></head>"
"<body>"
"  <div class='container'>"
"    <h2>System Regulacji PID</h2>"
"    <div id='chart'></div>"
"    <div class='stats'>"
"      <b>Aktywne nastawy:</b> "
"      Kp: <span id='cur_kp' class='val'>-</span>"
"      Ki: <span id='cur_ki' class='val'>-</span>"
"      Kd: <span id='cur_kd' class='val'>-</span>"
"    </div>"
"    <div>"
"      Kp: <input type='number' id='kp' value='1.0' step='0.1'>"
"      Ki: <input type='number' id='ki' value='0.5' step='0.1'>"
"      Kd: <input type='number' id='kd' value='0.0' step='0.1'>"
"      <button onclick='sendPID()'>Zapisz Nastawy</button>"
"    </div>"
"  </div>"
"<script>"
"var traces=[{y:[],name:'Setpoint (SP)',line:{color:'red'}},"
"            {y:[],name:'Wyjście (Y)',line:{color:'blue'}},"
"            {y:[],name:'Sterowanie (U)',line:{color:'green',dash:'dot'}}];"
"Plotly.newPlot('chart',traces,{margin:{t:20}});"
"function getData(){"
" fetch('/data').then(r=>r.json()).then(d=>{"
"  Plotly.extendTraces('chart',{y:[[d.sp],[d.y],[d.u]]},[0,1,2],100);"
"  document.getElementById('cur_kp').innerText = d.kp.toFixed(2);"
"  document.getElementById('cur_ki').innerText = d.ki.toFixed(2);"
"  document.getElementById('cur_kd').innerText = d.kd.toFixed(2);"
" });"
"}"
"function sendPID(){"
" var p='kp='+document.getElementById('kp').value+'&ki='+document.getElementById('ki').value+'&kd='+document.getElementById('kd').value;"
" fetch('/setpid?'+p);"
"}"
"setInterval(getData,100);"
"</script></body></html>";

void handleRoot() { 
    server.send(200, "text/html", INDEX_HTML); 
}

void handleData() {
    StaticJsonDocument<256> doc;
    doc["sp"] = setpoint;
    doc["y"] = y;
    doc["u"] = u / 6.8; // Skalowanie do wykresu
    doc["kp"] = Kp;
    doc["ki"] = Ki;
    doc["kd"] = Kd;
    
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
    Serial.println("\nPołączono!");
    Serial.println("IP: " + WiFi.localIP().toString());

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

        Serial.print("SP:"); Serial.print(setpoint); Serial.print(",");
        Serial.print("Y:"); Serial.print(y); Serial.print(",");
        Serial.print("U_scaled:"); Serial.println(u / 6.8);
    }
}