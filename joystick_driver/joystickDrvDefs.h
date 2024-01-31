#define NUM_JOYSTICKS  2

typedef short sint16_t;
typedef enum
{
  ANALOG_PINA0=A0,
  ANALOG_PINA1=A1,
  ANALOG_PINA2=A2,
  ANALOG_PINA3=A3,
  ANALOG_PINA4=A4,
  DIGITAL_PIN7=7,
  DIGITAL_PIN8=8
    
}pinMap_ent;

typedef struct
{
  pinMap_ent pin_x;
  pinMap_ent pin_y;
  pinMap_ent pin_sw;

} joyStickConf_t;

typedef void (*drvCallBack_t)(void);
typedef struct
{
  pinMap_ent pin_x;
  pinMap_ent pin_y;
  pinMap_ent pin_sw;
  sint16_t x_val;
  sint16_t y_val;
  uint8_t swState;
  drvCallBack_t drvCallBackFn;
} joyStickData_t;

typedef struct
{
  sint16_t x_valPrv;
  sint16_t y_valPrv;
  uint8_t swStatePrv;

} joyStickOldData_t;