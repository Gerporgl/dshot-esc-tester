/*
 * ----------------------------------------------------------------------------
 * "THE PROP-WARE LICENSE" (Revision 42):
 * <https://github.com/JyeSmith> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me some props in return.   Jye Smith
 * ----------------------------------------------------------------------------
 */

/* Some of the below code is taken from examples provided by Felix on RCGroups.com
 * 
 * KISS ESC 24A Serial Example Code for Arduino.
 * https://www.rcgroups.com/forums/showthread.php?2555162-KISS-ESC-24A-Race-Edition-Flyduino-32bit-ESC
 * https://www.rcgroups.com/forums/showatt.php?attachmentid=8521072&d=1450345654 * 
 */

#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "Arduino.h"
#include "esp32-hal.h"

#include <PID_v1.h>

//PID: Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
// Good starting point, TMotor MN5008, free spinning, sample time set at 6ms and telemetry almost the same
//PID myPID(&Input, &Output, &Setpoint,3.00,20.0,0.05, DIRECT);
// Not bad 2
//PID myPID(&Input, &Output, &Setpoint,3.00,30.0,0.04, DIRECT);
// Not bad 3, all those first 3 PID settings are at 6ms sample and %64 telemetry
//PID myPID(&Input, &Output, &Setpoint,2.00,20.0,0.04, DIRECT);
// New test PID sample rate at 3ms and %32 telemetry
// Not bad 4 (starting point)
//PID myPID(&Input, &Output, &Setpoint,2.00,20.0,0.02, DIRECT);
// Better 5
//PID myPID(&Input, &Output, &Setpoint,1.00,10.0,0.02, DIRECT);
// Better 6
PID myPID(&Input, &Output, &Setpoint,1.00,8.0,0.01, DIRECT);

#define MOTOR_POLES 28
#define POT_PIN 4
#define DSHOT_PIN 5

rmt_data_t dshotPacket[18];
rmt_obj_t* rmt_send = NULL;

hw_timer_t * timerDshot = NULL;

HardwareSerial MySerial(1);

uint8_t receivedBytes = 0;
volatile bool requestTelemetry = false;
uint16_t dshotUserInputValue = 0;
int16_t ESC_telemetrie[5]; // Temperature, Voltage, Current, used mAh, eRpM
int target_value=0; // Target value from POT after filtering
int current_value=0; // Smoothed value
int target_rpm=0;
uint16_t dshotCommand=0;

bool pid_on=false;
uint32_t last_rpm=0;

uint8_t temperature = 0;
uint8_t temperatureMax = 0;
float voltage = 0;
float voltageMin = 99;
uint32_t current = 0;
uint32_t currentMax = 0;
uint32_t erpm = 0;
uint32_t erpmMax = 0;
uint32_t rpm = 0;
uint32_t rpmMAX = 0;
uint32_t kv = 0;
uint32_t kvMax = 0;

uint8_t telemetryskips=0;
int pid_val=0;

void IRAM_ATTR onDshotTimer(){
    telemetryskips++;
 
    if(pid_on)
    {
        if(rpm<65)
            Input=65;
        else
            Input = rpm;
        Setpoint = target_rpm; //map(current_value,60,1999,130,8000);
        myPID.Compute();
        pid_val=map(Output, 130, 8000, 60, 1999); // Working with rpms although it wasn't probably required...
    }
    else
    {
        // Continue to feed the PID loop fooling it to think the motor is at stable RPM and low setpoint so that it doesn't get back crazy
        // It stil does for a short blip...
        Input = 135;
        Setpoint = 130;
        myPID.Compute();
        pid_val=map(Output, 130, 8000, 60, 1999);
    }
    
    if(current_value>=70)
    {
        
        if(pid_on)
            dshotUserInputValue=pid_val+48;
        else if(current_value<=160)
            dshotUserInputValue=60+48;    
        else
            dshotUserInputValue=current_value+48;  
    }
    else
        dshotUserInputValue=current_value+48;   

    if(dshotCommand==0)
        dshotOutput(dshotUserInputValue, (telemetryskips%32==0));
    else
        dshotOutput(dshotCommand, (telemetryskips%32==0));
   
    if (requestTelemetry) {                
        requestTelemetry = false;
        receivedBytes = 0;
    }   
}

void startDShotTimer() {
    timerDshot = timerBegin(1, 80, true); // timer_id = 0; divider=80; countUp = true;
    timerAttachInterrupt(timerDshot, &onDshotTimer, true); // edge = true
    timerAlarmWrite(timerDshot, 100, true);  //1000 = 1 ms
    timerAlarmEnable(timerDshot);
}

void setup() {

    digitalWrite(DSHOT_PIN, 0);
    pinMode(DSHOT_PIN, OUTPUT);
    pinMode(POT_PIN,INPUT);
    Serial.begin(115200);
    MySerial.begin(115200, SERIAL_8N1, 16, 17); 
    
    if ((rmt_send = rmtInit(DSHOT_PIN, true, RMT_MEM_64)) == NULL) {
        Serial.println("init sender failed\n");
    }

    float realTick = rmtSetTick(rmt_send, 12.5); // 12.5ns sample rate
    Serial.printf("rmt_send tick set to: %fns\n", realTick);

    Input = 0;
    Setpoint = 0;

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(130, 8000); // Again, we work with RPMs...
    myPID.SetSampleTime(3); // 3 ms is the fastest so far, and should be close to the average telemetry receive frequency

    dshotOutput(48, false);
    dshotUserInputValue=0;
    
    // Empty Rx Serial of garbage telemtry
    while(MySerial.available())
        MySerial.read();
    
    requestTelemetry = true;
    
    startDShotTimer();


}

int val=0;

uint32_t now=micros();
uint32_t ticks=now;
int last_val=val;

uint32_t last_stat=0;
uint32_t last_smooth=0;
float adc_val=0;

void loop() {
    now=micros();
    if( now - last_rpm >= 25000 )
        pid_on=false;
    else
        pid_on=true;

    if(now-ticks >= 100)
    {
        ticks=now;

        adc_val = (adc_val * 0.95) + ((analogRead(POT_PIN)*32.0F)*0.05);

        // Just a quick fixed value table for testing at different reference points.
        val=map(int(adc_val), 0, 4095*32, 0, 1999);
        if(val>25 && val<200)
            val=120;
        else if(val>=200 && val<600)
            val=200;
        else if(val>=600 && val<1000)
            val=300;
        else if(val>=1000 && val<1400)
            val=400;
        else if(val>=1400 && val<1800)
            val=500;
        else if(val>=1800)
            val=600;
        else
            val=0;

        if(last_val != val && (val<=0 || val >= 1999 || abs(last_val-val) >= 5)) // Filter even more for small variations
        {
            last_val=val;
            
            if(val<=50)
                target_value=0;
            else if(val<60)
                target_value=60;
            else
                target_value=val;
           
        }

        if(now - last_smooth >= 1000)
        {
            last_smooth=now;

            if(current_value < target_value)
            {
                if(current_value<60)
                    current_value=60;
                else
                    current_value+=1;

            }
            else if(current_value > target_value)
            {
                if(target_value < 60)
                    current_value=0;
                else
                    current_value-=1;
            }
            target_rpm = constrain(map(current_value,60,1999,130,8000),0,8000);
        }
          
         if(now - last_stat >= 1000000)
        {
            last_stat=now;
            Serial.printf("ms: %d rpm: %d rpm_age: %d volt: %.2f dshot: %d target_rpm %d target_val: %d cv: %d pid: %d\n",now/1000, rpm, (now-last_rpm)/1000, voltage, dshotUserInputValue, target_rpm, target_value, current_value, pid_val);
        }
    }
   
    if(!requestTelemetry) {
       receiveTelemtrie();
    } 
    
}

void receiveTelemtrie(){
    static uint8_t SerialBuf[10];

    if(MySerial.available()){
        SerialBuf[receivedBytes] = MySerial.read();
        receivedBytes++;
    }

    if(receivedBytes > 9){ // transmission complete
        
        uint8_t crc8 = get_crc8(SerialBuf, 9); // get the 8 bit CRC
        
        if(crc8 != SerialBuf[9]) {
            // These errors appear to happen on regular basis, at least on the only ESC I tested with
            // I couldn't see on the scope if anything was wrong, or if this is just the ESC not sending all the data properly
            // regardless, at very fast (around 1 ms) telemetry request speed, the average telemetry successful responses
            // always seems to come at 0-2 ms intervals.
            //Serial.println("CRC transmission failure");
            
            // Empty Rx Serial of garbage telemtry
            while(MySerial.available())
                MySerial.read();
            
            requestTelemetry = true;
        
            return; // transmission failure 
        }
        
        // compute the received values
        ESC_telemetrie[0] = SerialBuf[0]; // temperature
        ESC_telemetrie[1] = (SerialBuf[1]<<8)|SerialBuf[2]; // voltage
        ESC_telemetrie[2] = (SerialBuf[3]<<8)|SerialBuf[4]; // Current
        ESC_telemetrie[3] = (SerialBuf[5]<<8)|SerialBuf[6]; // used mA/h
        ESC_telemetrie[4] = (SerialBuf[7]<<8)|SerialBuf[8]; // eRpM *100
        
        requestTelemetry = true;
        
        temperature = 0.9*temperature + 0.1*ESC_telemetrie[0];
        if (temperature > temperatureMax) {
            temperatureMax = temperature;
        }
        
        voltage = 0.9*voltage + 0.1*(ESC_telemetrie[1] / 100.0);
        if (voltage < voltageMin) {
            voltageMin = voltage;
        }
        
        current = 0.9*current + 0.1*(ESC_telemetrie[2] * 100);
        if (current > currentMax) {
            currentMax = current;
        }
        
        // This averaging is also working ok for the current PID loop and settings, but I've lowered the weight of older samples
        erpm = 0.8*erpm + 0.2*(ESC_telemetrie[4] * 100);
        if (erpm > erpmMax) {
            erpmMax = erpm;
        }
        
        rpm = erpm / (MOTOR_POLES / 2);
        if (rpm > rpmMAX) {
            rpmMAX = rpm;
        }
        last_rpm=micros();
        
        if (rpm) {                  // Stops weird numbers :|
            kv = rpm / voltage / ( (float(dshotUserInputValue) - dshotmin) / (dshotmax - dshotmin) );
        } else {
            kv = 0;
        }
        if (kv > kvMax) {
            kvMax = kv;
        }
        
    }

  return;
  
}

void dshotOutput(uint16_t value, bool telemetry) {
    
    uint16_t packet;
    
    // telemetry bit    
    if (telemetry) {
        requestTelemetry=true;
        packet = (value << 1) | 1;
    } else {
        packet = (value << 1) | 0;
    }

    // https://github.com/betaflight/betaflight/blob/09b52975fbd8f6fcccb22228745d1548b8c3daab/src/main/drivers/pwm_output.c#L523
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    packet = (packet << 4) | csum;

    // durations are for dshot600
    // https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
    // Bit length (total timing period) is 1.67 microseconds (T0H + T0L or T1H + T1L).
    // For a bit to be 1, the pulse width is 1250 nanoseconds (T1H – time the pulse is high for a bit value of ONE)
    // For a bit to be 0, the pulse width is 625 nanoseconds (T0H – time the pulse is high for a bit value of ZERO)
    for (int i = 0; i < 16; i++) {
        if (packet & 0x8000) {
              dshotPacket[i].level0 = 1;
              dshotPacket[i].duration0 = 100;
              dshotPacket[i].level1 = 0;
              dshotPacket[i].duration1 = 34;
          } else {
              dshotPacket[i].level0 = 1;
              dshotPacket[i].duration0 = 50;
              dshotPacket[i].level1 = 0;
              dshotPacket[i].duration1 = 84;
          }
        packet <<= 1;
    }

    // Tried to use rmtLoop from latest ESP32 master branch, but it doesn't seem to allow seemless synchronyzed updates,
    // but also since we need to turn on and off the telemetry bit it wasn't practical so reverted back to the rmtWrite in
    // a fast loop. It would be nice to find a way to have this dshot loop perfectly working and synced eventually so that there is no
    // jitter and glitch.
   /* dshotPacket[0].level0 = 0;
    dshotPacket[0].duration0 = 586;
    dshotPacket[0].level1 = 0;
    dshotPacket[0].duration1 = 586;
    dshotPacket[17].level0 = 0;
    dshotPacket[17].duration0 = 586;
    dshotPacket[17].level1 = 0;
    dshotPacket[17].duration1 = 586;*/
 
    rmtWrite(rmt_send, dshotPacket, 16);

    
    return;

}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}