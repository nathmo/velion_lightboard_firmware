#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_VEML7700.h>

// ================= PIN MAP =================
#define PIN_PIEZO_X 9
#define PIN_PIEZO_Y 10

#define ADC_ECHO 0  // GPIO0

// ================= I2C =====================
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115 ads0, ads1;
Adafruit_VEML7700 veml;

// ================= PWM =====================
#define PWM_FREQ 40000
#define PWM_RES 8
#define PWM_CH_X 0
#define PWM_CH_Y 1

// ================= SONAR ===================
enum { SONAR_A, SONAR_B, SONAR_C };
int sonarIndex = 0;
bool sonarRunning = false;
unsigned long sonarTimer = 0;

// ================= DSP =====================
#define FC 40000.0
#define FS 320000.0
#define N 128

float sinLUT[N], cosLUT[N];
uint16_t adcBuf[N];

// ================= HELP ====================
void printHelp() {
  Serial.println(F("\n=== Velion Lightboard Test ==="));
  Serial.println(F("M n 0/1   -> MOSFET n OFF/ON (0-7)"));
  Serial.println(F("P A/B/C 0/1 -> Enable piezo channel"));
  Serial.println(F("I         -> Print all currents"));
  Serial.println(F("C n       -> Print current n"));
  Serial.println(F("S START/STOP -> Start/Stop sonar cycle"));
  Serial.println(F("H 0/1     -> Enable H-bridge"));
  Serial.println(F("T         -> One sonar test"));
  Serial.println(F("V         -> Luminosity"));
}

// ================= SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(3,4);

  mcp.begin_I2C(0x20);
  for(int i=8;i<16;i++) mcp.pinMode(i, OUTPUT);
  for(int i=0;i<3;i++) mcp.pinMode(i, OUTPUT);

  ads0.begin(0x48);
  ads1.begin(0x49);
  ads0.setGain(GAIN_SIXTEEN);
  ads1.setGain(GAIN_SIXTEEN);

  veml.begin();

  ledcSetup(PWM_CH_X, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_Y, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PIEZO_X, PWM_CH_X);
  ledcAttachPin(PIN_PIEZO_Y, PWM_CH_Y);

  for(int i=0;i<N;i++){
    float a = 2*PI*FC*i/FS;
    sinLUT[i]=sin(a);
    cosLUT[i]=cos(a);
  }

  printHelp();
}

// ================= MOSFET ==================
void setMOS(int n,bool v){
  int pin = 8 + (7-n);
  mcp.digitalWrite(pin,v);
}

// ================= PIEZO ===================
void enablePiezo(int ch,bool v){
  mcp.digitalWrite(ch,v);
}

void hbridge(bool en){
  if(en){
    ledcWrite(PWM_CH_X,255);
    ledcWrite(PWM_CH_Y,0);
  } else {
    ledcWrite(PWM_CH_X,0);
    ledcWrite(PWM_CH_Y,0);
  }
}

// ================= CURRENT =================
float readCurrent(int n){
  int16_t adc;
  if(n<4) adc=ads0.readADC_SingleEnded(3-n);
  else adc=ads1.readADC_SingleEnded(7-n);
  return adc * 0.0078125; // mA
}

// ================= DSP =====================
float detectCarrier(){
  float I=0,Q=0;
  for(int i=0;i<N;i++){
    adcBuf[i]=analogRead(ADC_ECHO);
    I+=adcBuf[i]*cosLUT[i];
    Q+=adcBuf[i]*sinLUT[i];
  }
  return sqrt(I*I+Q*Q);
}

// ================= SONAR ===================
void selectSonar(int s){
  enablePiezo(0,0);
  enablePiezo(1,0);
  enablePiezo(2,0);
  enablePiezo(s,1);
}

void sonarOnce(){
  selectSonar(sonarIndex);
  hbridge(true);
  delayMicroseconds(200);
  hbridge(false);
  delayMicroseconds(100);
  float amp = detectCarrier();
  Serial.printf("SONAR %d AMP=%.1f\n",sonarIndex,amp);
  sonarIndex=(sonarIndex+1)%3;
}

// ================= SERIAL ==================
void handleCmd(){
  if(!Serial.available()) return;
  String c=Serial.readStringUntil('\n');
  c.trim();

  if(c=="I"){
    for(int i=0;i<8;i++)
      Serial.printf("I%d = %.2f mA\n",i,readCurrent(i));
  }
  else if(c.startsWith("M")){
    int n,v;
    sscanf(c.c_str(),"M %d %d",&n,&v);
    setMOS(n,v);
  }
  else if(c.startsWith("P")){
    char ch; int v;
    sscanf(c.c_str(),"P %c %d",&ch,&v);
    enablePiezo(ch=='A'?0:ch=='B'?1:2,v);
  }
  else if(c.startsWith("C")){
    int n; sscanf(c.c_str(),"C %d",&n);
    Serial.printf("I%d = %.2f mA\n",n,readCurrent(n));
  }
  else if(c=="S START") sonarRunning=true;
  else if(c=="S STOP") sonarRunning=false;
  else if(c.startsWith("H")){
    int v; sscanf(c.c_str(),"H %d",&v);
    hbridge(v);
  }
  else if(c=="T") sonarOnce();
  else if(c=="V") Serial.println(veml.readLux());
  else printHelp();
}

// ================= LOOP ====================
void loop(){
  handleCmd();
  if(sonarRunning && millis()-sonarTimer>1000){
    sonarTimer=millis();
    sonarOnce();
  }
}
