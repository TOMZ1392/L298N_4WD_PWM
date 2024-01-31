#include "joystickDrvDefs.h"
#define TESTMODE

const joyStickConf_t joysticksPinsCfg[NUM_JOYSTICKS] =
{
  {
    ANALOG_PINA1,
    ANALOG_PINA2,
    DIGITAL_PIN7
  },
  {
    ANALOG_PINA3,
    ANALOG_PINA4,
    DIGITAL_PIN8
  }
};

joyStickData_t  joyStickDataList[NUM_JOYSTICKS];
joyStickOldData_t   joyStickOldValList[NUM_JOYSTICKS];

void genericJoystickCallBack();

const drvCallBack_t drvCallBkList[NUM_JOYSTICKS]=
{
  &genericJoystickCallBack,
  &genericJoystickCallBack
};

void genericJoystickCallBack()
{
  for (uint8_t ctlIdx = 0; ctlIdx < NUM_JOYSTICKS; ctlIdx++)
  {
    sint16_t x_val = analogRead(joyStickDataList[ctlIdx].pin_x);
    sint16_t y_val = analogRead(joyStickDataList[ctlIdx].pin_y);
    uint8_t swState = digitalRead(joyStickDataList[ctlIdx].pin_sw);
    
    if (x_val != joyStickOldValList[ctlIdx].x_valPrv)
    {
      joyStickDataList[ctlIdx].x_val = x_val;
      joyStickOldValList[ctlIdx].x_valPrv = x_val;
    }

    if (y_val != joyStickOldValList[ctlIdx].y_valPrv)
    {
      joyStickDataList[ctlIdx].y_val = y_val;
      joyStickOldValList[ctlIdx].y_valPrv = y_val;
    }

    if (swState != joyStickOldValList[ctlIdx].swStatePrv)
    {
      joyStickDataList[ctlIdx].swState = swState;
      joyStickOldValList[ctlIdx].swStatePrv = swState;
    }
  }
}



void initJoysticks()
{
  for (uint8_t ctlIdx = 0; ctlIdx < NUM_JOYSTICKS; ctlIdx++)
  {
    pinMode(joysticksPinsCfg[ctlIdx].pin_x,INPUT);
    pinMode(joysticksPinsCfg[ctlIdx].pin_y,INPUT);
    pinMode(joysticksPinsCfg[ctlIdx].pin_sw,INPUT);
    
    joyStickDataList[ctlIdx].pin_x=joysticksPinsCfg[ctlIdx].pin_x;
    joyStickDataList[ctlIdx].pin_y=joysticksPinsCfg[ctlIdx].pin_y;
    joyStickDataList[ctlIdx].pin_sw=joysticksPinsCfg[ctlIdx].pin_sw;
    joyStickDataList[ctlIdx].drvCallBackFn=drvCallBkList[ctlIdx];
    
  }
}


void readJoystickState()
{
  for (uint8_t ctlIdx = 0; ctlIdx < NUM_JOYSTICKS; ctlIdx++)
  {
    joyStickDataList[ctlIdx].drvCallBackFn();  
  }
}

joyStickData_t* getJoystickData()
{
  
return (joyStickData_t*)&joyStickDataList;  
}

#ifdef TESTMODE
void setup() {
  Serial.begin(9600);
  
  initJoysticks();
  Serial.println("init Done!");

}


void loop() {
  readJoystickState();
  Serial.print("j_x:");
  Serial.println(joyStickDataList[0].x_val);
  Serial.print("j_y:");
  Serial.println(joyStickDataList[0].y_val);
  Serial.print("j_sw:");
  Serial.println(joyStickDataList[0].swState);
  delay(500);

}
#endif
