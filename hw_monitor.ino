#include <Wire.h>
#include <Adafruit_SSD1306.h>

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

#define GPU_PIN 0
#define LOOP_PIN 2


/*********************************************************/
void setup()
{
  Serial.begin(9600);
  Serial.println("Hello, World");
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // OLED Splash Screen
  int16_t i;
  display.clearDisplay();
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  for(i=display.width(); i>0; i-=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_BLACK);
    display.display();
    delay(1);
  }

  for(i=display.height(); i>0; i-=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_BLACK);
    display.display();
    delay(1);
  }

  display.clearDisplay(); 
}

/*********************************************************/
float adcToTemp(int adcVal)
{
    // Converts input from a thermistor voltage divider to a temperature value.
    // The voltage divider consists of thermistor Rt and series resistor R0.
    // The value of R0 is equal to the thermistor resistance at T0.
    // You must set the following constants:
    //                  adcMax  ( ADC full range value )
    //                  analogPin (Arduino analog input pin)
    //                  invBeta  (inverse of the thermistor Beta value supplied by manufacturer).
    // Use Arduino's default reference voltage (5V or 3.3V) with this module.
    //

//    const int analogPin = 0; // replace 0 with analog pin
//  const float invBeta = 1.00 / 3380.00;   // replace "Beta" with beta of thermistor
//
//  const  float adcMax = 1024.00;
//  const float invT0 = 1.00 / 298.15;   // room temp in Kelvin

//  int numSamples = 5;
//  int adcVal  = 0;
//  for (int i = 0; i < numSamples; i++)
//   {
//     adcVal = adcVal + analogRead(analogPin);
//     delay(100);
//   }
//  adcVal = adcVal/numSamples;
//    int adcVal = analogRead(analogPin);

//  float K = 1.00 / (invT0 + invBeta*(log ( adcMax / (float) adcVal - 1.00)));

  double K = log(10000.0 * ((1024.0/adcVal-1)));
  K = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * K * K )) * K );
  float celcius = K - 273.15;  // convert to Celsius
  return celcius;
}


bool measure_environment( float *temperature, float *loop_temp)
{
  static unsigned long startMillis = 0;
  static uint16_t rollingAdc_GPU = 0;
  static uint16_t rollingAdc_LOOP = 0;
  static uint8_t i = 0;

  unsigned long currentMillis = millis();  //some global variables available anywhere in the program
  uint16_t delta = currentMillis - startMillis;
  const unsigned long period = 500;  //the value is a number of milliseconds

  if (delta % 100 == 0)
  {
    rollingAdc_GPU += analogRead(GPU_PIN);
    rollingAdc_LOOP += analogRead(LOOP_PIN);
    i++;
  }
  /* Measure once every {period/1000} seconds. */
  if(delta >= period)
  {  
      startMillis = currentMillis; // reset 
      rollingAdc_GPU = rollingAdc_GPU/i; // set to average of prior rolling
      rollingAdc_LOOP = rollingAdc_LOOP/i;
      i = 1; // reset avg count
      *temperature = adcToTemp(rollingAdc_GPU);
      *loop_temp = adcToTemp(rollingAdc_LOOP);
      return( true );
  }

  return( false );
}

void graph(float *gpu_temp, float *loop_temp)
{

  static float maxTemp;
  static float minTemp;
//  in case i want to draw both lines someday
//  float minSensor = min(*gpu_temp, *loop_temp);
//  float maxSensor = max(*gpu_temp, *loop_temp);
//  minTemp = min(minTemp, minSensor);
//  maxTemp = max(maxTemp, maxSensor);

  minTemp = min(minTemp, *gpu_temp);
  maxTemp = max(maxTemp, *gpu_temp);
//  Serial.println(minTemp);

  static int history[128];

  int y = ((*gpu_temp - minTemp) * 32 )/(maxTemp - minTemp);
  history[127] = y;
  
  display.drawPixel(127, (63 - history[127]), WHITE);
  
  for (int i = 0; i < 127; i++)
  {
    history[i] = history[i+1];
    display.drawPixel(i, (63 - history[i]), WHITE);

//    if (history[i] < maxTemp)
//    {
//      maxTemp = history[i];
//    }
//    else if (history[i] > minTemp)
//    {
//      minTemp = history[i]; 
//    }
  }
}

void write_oled(float *gpu_temp, float *loop_temp)
{
    // OLED
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);

  // Write Temperature line
  display.setCursor(0,0);
  display.print(*gpu_temp,1);
  display.drawCircle(52,2,2,WHITE); //degree symbol

  display.setCursor(72,0);
  display.print(*loop_temp,1);
  display.drawCircle(123,2,2,WHITE); //degree symbol

  display.setTextSize(1);
  display.setCursor(0,20);
  display.print("GPU");
  display.setCursor(99,20);
  display.print("LOOP");
  
  graph(gpu_temp, loop_temp);
  display.display();
}
 

void loop()
{
  float gpu_temp;
  float loop_temp;
  if( measure_environment(&gpu_temp, &loop_temp) == true )
  {
    write_oled(&gpu_temp, &loop_temp);
  }
}
