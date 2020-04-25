#include "WiFi.h"
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"

// YAW Gear ratio: 4:1
// PITCH Gear ratio: 39.0625:1
// Stepper1 Default: 800
// Stepper2 Default: 7812

#define STEPPER1ROTATIONSTEPS 25600
#define STEPPER2ROTATIONSTEPS 250000

/** Set RPM for both Axes
 *  NOTE: Actual RPM is 1/4 of stated RPM for 2 axis rotation
 *  NOTE: Actual RPM is 1/2 of stated RPM for 1 axis rotation
 */
#define STEPPER1RPM           20 
#define STEPPER2RPM           20

#define ARDUINO_RUNNING_CORE 1

// Microsteps
#define MICROSTEPS       32
#define STEPPER1GEARING  4
#define STEPPER2GEARING  39.0625

#define YAWSTEP       25
#define YAWDIR        26
#define YAWEN         27

#define PITCHSTEP     14
#define PITCHDIR      12
#define PITCHEN       13

#define TICK          1 // Tick time is 1 miliseconds
 
const char *ssid = "ESP32AP";
const char *password = "potatosalad";
 
AsyncWebServer server(80);

const char* PARAM_INPUT_1 = "yaw";
const char* PARAM_INPUT_2 = "pitch";

// HTML web page to handle 3 input fields (input1, input2, input3)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1>WARNING: MAY GET INTO AN INFINITE LOOP</h1>
  <form action="/get">
    Yaw: <input type="number" name="yaw" step="0.01"><br>
    Pitch: <input type="number" name="pitch" step="0.01"><br>
    <input type="submit" value="Submit">
  </form><br>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

int32_t yawSteps = 0;
int32_t pitchSteps = 0;

void setup(){
  Serial.begin(115200);

  pinMode(YAWSTEP, OUTPUT);
  pinMode(YAWDIR, OUTPUT);
  pinMode(YAWEN, OUTPUT);
  
  pinMode(PITCHSTEP, OUTPUT);
  pinMode(PITCHDIR, OUTPUT);
  pinMode(PITCHEN, OUTPUT);

  digitalWrite(YAWEN, LOW);
  digitalWrite(PITCHEN, LOW);
 
  WiFi.softAP(ssid, password);
 
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });
  
  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String yawInput;
    String pitchInput;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      yawInput = request->getParam(PARAM_INPUT_1)->value();
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    if (request->hasParam(PARAM_INPUT_2)) {
      pitchInput = request->getParam(PARAM_INPUT_2)->value();
    }
    request->send(200, "text/html", "HTTP GET request sent to your ESP with yaw: " + yawInput + " degrees and pitch: " + pitchInput + 
                                     "degrees<br><a href=\"/\">Return to Home Page</a>");
    float yawDeg = yawInput.toFloat();
    float pitchDeg = pitchInput.toFloat();
    yawSteps = yawDeg/360 * STEPPER1ROTATIONSTEPS;
    pitchSteps = pitchDeg/360 * STEPPER2ROTATIONSTEPS;
    rotate(&yawSteps,&pitchSteps);
  });
  server.onNotFound(notFound);
 
  server.begin();


  
}
 
void loop(){}

void rotate(int32_t* yawSteps, int32_t* pitchSteps){
//  Serial.println(yawSteps);
//  Serial.println(pitchSteps);
  Serial.println("Rotation Commenced");
  xTaskCreatePinnedToCore(moveYaw,"moveYaw",1024,(void*)yawSteps,1,NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(movePitch,"movePitch",1024,(void*)pitchSteps,2,NULL,ARDUINO_RUNNING_CORE);
}

void moveYaw(void *parameter){
  int32_t steps = *((int32_t*)parameter);
  float stepsPerTick = (((STEPPER1ROTATIONSTEPS * STEPPER1RPM)/60)/1000) * TICK; // Num steps per rotation * RPM divided by seconds per minute and miliseconds per second * tick
  uint32_t stepDelay = (((TICK/(float)stepsPerTick) * 1000)/2) - 1;
  if (steps > 0){
    digitalWrite(YAWDIR, HIGH);
  } else {
    digitalWrite(YAWDIR, LOW);
  }
  uint32_t stepsRemaining = abs(steps);
  while (stepsRemaining > 0){
    if (stepsRemaining < stepsPerTick){
      for (uint16_t i = 0; i < stepsRemaining; i++){
        digitalWrite(YAWSTEP, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(YAWSTEP, LOW);
        delayMicroseconds(stepDelay);
      }
      break;
    } else {
      stepsRemaining -= (int)stepsPerTick;
      for (uint16_t i = 0; i < (int)stepsPerTick; i++){
        digitalWrite(YAWSTEP, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(YAWSTEP, LOW);
        delayMicroseconds(stepDelay);
      }
    }
    //taskYIELD();
    vTaskDelay(TICK/portTICK_PERIOD_MS);
  }
  Serial.println("Yaw Complete");
  vTaskDelete(NULL);
};

void movePitch(void *parameter){
  int32_t steps = *((int32_t*)parameter);
  float stepsPerTick = (((STEPPER2ROTATIONSTEPS * STEPPER2RPM)/60)/1000) * TICK; // Num steps per rotation * RPM divided by seconds per minute and miliseconds per second * tick
  uint32_t stepDelay = ((TICK/stepsPerTick) * 1000)/2;
  if (steps > 0){
    digitalWrite(PITCHDIR, HIGH);
  } else {
    digitalWrite(PITCHDIR, LOW);
  }
  uint32_t stepsRemaining = abs(steps);
  
  while (stepsRemaining > 0){
    if (stepsRemaining < stepsPerTick){
      for (uint16_t i = 0; i < stepsRemaining; i++){
        digitalWrite(PITCHSTEP, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(PITCHSTEP, LOW);
        delayMicroseconds(stepDelay);
      }
      break;
    } else {
      stepsRemaining -= (int)stepsPerTick;
      for (uint16_t i = 0; i < (int)stepsPerTick; i++){
        digitalWrite(PITCHSTEP, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(PITCHSTEP, LOW);
        delayMicroseconds(stepDelay);
      }
    }
    //taskYIELD();
    vTaskDelay(TICK/portTICK_PERIOD_MS);
  }
  Serial.println("Pitch Complete");
  vTaskDelete(NULL);
};
