// PowerTrain IO defines
#define FWD_LFT 4  //2
#define FWD_RGT 0  //d3
#define REV_LFT 2  //d4
#define REV_RGT 14 //d5
#define PWMRGT 5
#define PWMLFT 12
#define PWM_STEP 50
#define PWM_LOLIM 50
#define PWM_UPLIM 970
#define PWM_MIN 50

#define TURN_STEP_DURN_MS 500
#define MAX_THROTT_LIMITER 900
#define CUTOFF_THROTT_PWM 200
#define RAMP_STEP_PWM  100
#define ROT_PWM   500
#define ROT_STEP_DURN_MS 300

#define EN_SERIAL_DATA 

volatile uint16_t drivePwm = 0;
volatile uint32_t left_startTi = 0;
volatile uint32_t right_startTi = 0;
volatile uint32_t cwRot_startTi = 0;
volatile uint32_t ccwRot_startTi = 0;
volatile uint32_t speedDelta_rampStart = 0;
volatile int8_t dirHead = 0;

volatile uint8_t pwmLft_g;
volatile uint8_t pwmRgt_g;


//Module user requests
volatile bool requestFwd_flg;
volatile bool requestRev_flg;
volatile bool requestLft_flg;
volatile bool requestRgt_flg;
volatile bool requestStp_flg;
volatile bool requestCCWRot_flg;
volatile bool requestCWRot_flg;

#define isFwdRequested()  requestFwd_flg

#define isRevRequested() requestRev_flg

#define bool isLeftRequested() requestLft_flg

#define bool isRightRequested() requestRgt_flg

#define bool isStopRequested() requestStp_flg

#define bool isCCWRotRequested() requestCCWRot_flg

#define bool isCWRotRequested() requestCWRot_flg

#define setPwmLft_g(x)   pwmLft_g = x;
#define getPwmLft_g()  pwmLft_g
#define setPwmRgt_g(x)   pwmRgt_g = x;
#define getPwmRgt_g()  pwmRgt_g;
#define FIXED_RAMP


void clrRequests()
{
  requestFwd_flg = False;
  requestRev_flg = False;
  requestLft_flg = False;
  requestRgt_flg = False;
  requestStp_flg = False;
  requestCCWRot_flg = False;
  requestCWRot_flg = False;
}



void setMotorPwm(uint16_t lft, uint16_t rgt) // set speed of traction motors thru PWM
{
  if (lft <= PWM_LOLIM)
    lft = PWM_MIN;
  if (lft >= PWM_UPLIM)
    lft = PWM_UPLIM;
  if (rgt <= PWM_LOLIM)
    rgt = PWM_MIN;
  if (rgt >= PWM_UPLIM)
    rgt = PWM_UPLIM;

  analogWrite(PWMLFT, lft);
  analogWrite(PWMRGT, rgt);
  
  setPwmLft_g(lft);
  setPwmRgt_g(rgt);
  
  #ifdef EN_SERIAL_DATA
  Serial.print("Lftpwm: ");
  Serial.print(getPwmLft_g());
  Serial.print(" Rgtpwm: ");
  Serial.println(getPwmRgt_g());
  #endif
}

void pullAllLow() // emergency Stop
{
  digitalWrite(REV_LFT, LOW);
  digitalWrite(REV_RGT, LOW);
  digitalWrite(FWD_LFT, LOW);
  digitalWrite(FWD_RGT, LOW);
  
  //delay(200); -- need a ramp instead
}


// ramp down to stop from whichever state maybe
// use the macro FIXED_RAMP to create a fixed ramp function or variable ramp



#ifdef FIXED_RAMP
void rampDownToZero() 
{
  const uint8_t RAMP_STEP=20;
  const uint16_t RAMP_DELY=100;
#else
void rampDownToZero(uint8_t RAMP_STEP,uint16_t RAMP_DELY) 
{
#endif
  volatile uint8_t lft=getPwmLft_g();
  volatile uint8_t rgt=getPwmRgt_g();
  
  while(TRUE)
  {
    if(lft>RAMP_STEP)
    {
      lft-=RAMP_STEP
    }
    else
    {
      lft=0;
    }
     if(rgt>RAMP_STEP)
    {
      rgt-=RAMP_STEP
    }
    else
    {
      rgt=0;
    }
  
    analogWrite(PWMLFT, lft);
    analogWrite(PWMRGT, rgt);
    delay(RAMP_DELY); -- need a ramp instead
    if(lft==0 && rgt==0)
    {
       break;  
    }
  }
  pullAllLow();
  
}



void cwRot()
{
  pullAllLow();
  digitalWrite(REV_LFT, LOW);
  digitalWrite(REV_RGT, HIGH);
  digitalWrite(FWD_LFT, HIGH);
  digitalWrite(FWD_RGT, LOW);
}

void ccwRot()
{
  pullAllLow();
  digitalWrite(REV_LFT, HIGH);
  digitalWrite(REV_RGT, LOW);
  digitalWrite(FWD_LFT, LOW);
  digitalWrite(FWD_RGT, HIGH);
}


void setDrivePwm(uint16_t pwm)
{
  drivePwm = pwm;
}
void stopDrive()
{
  dirHead = 0;
  pullAllLow();
}

void setDriveDirState(int8_t dir)
{
  if (dir > 0)
  {
    pullAllLow();
    digitalWrite(REV_LFT, LOW);
    digitalWrite(REV_RGT, HIGH);
    digitalWrite(FWD_LFT, LOW);
    digitalWrite(FWD_RGT, HIGH);
  }
  else if (dir < 0)
  {
    pullAllLow();
    digitalWrite(REV_LFT, HIGH);
    digitalWrite(REV_RGT, LOW);
    digitalWrite(FWD_LFT, HIGH);
    digitalWrite(FWD_RGT, LOW);
  }
  else
  {
    pullAllLow();
  }
}


void driveMotionCtl(bool dirFlag)
{
  if (drivePwm > CUTOFF_THROTT_PWM && dirFlag)
  {
    if (drivePwm + RAMP_STEP_PWM <= MAX_THROTT_LIMITER)
    {
      drivePwm += RAMP_STEP_PWM;
    }
    else
    {
      drivePwm = MAX_THROTT_LIMITER;
    }
    setMotorPwm(drivePwm, drivePwm);
    setDriveDirState(dirHead);
  }
  else
  {
    dirHead = 0;
    stopDrive();
  }

  if (!dirFlag)
  {
    if (drivePwm - RAMP_STEP_PWM > MAX_THROTT_LIMITER)
    {
      drivePwm -= RAMP_STEP_PWM;
      setMotorPwm(drivePwm, drivePwm);
      setDriveDirState(dirHead);
    }
    else
    {
      drivePwm = CUTOFF_THROTT_PWM;
      dirHead = 0;
      stopDrive();
    }


  }
}

void startTimer(uint32_t* startTime)
{

  if (*startTime == 0)
  {
    *startTime = millis();
  }

}


void stopTimer(uint32_t* startTime)
{
  *startTime = 0;
}


// DriveTrain periodic Task

void Task_DriveTrainControl()
{

  if (isStopRequested())
  {
    dirHead = 0;
    stopDrive();
  }

  if (isFwdRequested())
  {
    if (dirHead == 0)
    {
      dirHead = 1;
    }

    driveMotionCtl(dirHead > 0);


    clrRequests();
  }

  if (isRevRequested())
  {
    if (dirHead == 0)
    {
      dirHead = -1;
    }

    driveMotionCtl(dirHead < 0);


    clrRequests();
  }


  if (isLeftRequested() && drivePwm > TURN_STEP_PWM)
  {
    startTimer(&left_startTi);
    setMotorPwm(drivePwm - TURN_STEP_PWM , drivePwm);
    if (millis() - left_startTi > TURN_STEP_DURN_MS)
    {
      clrRequests();
      stopTimer(&left_startTi);
    }
  }

  if (isRightRequested() && drivePwm > TURN_STEP_PWM )
  {
    startTimer(&right_startTi);
    setMotorPwm(drivePwm , drivePwm - TURN_STEP_PWM);
    if (millis() - right_startTi > TURN_STEP_DURN_MS)
    {
      clrRequests();
      stopTimer(&right_startTi);
    }
  }


  if (isCWRotRequested())
  {
    startTimer(&cwRot_startTi);
    setMotorPwm(ROT_PWM , ROT_PWM);
    cwRot();
    if (millis() - cwRot_startTi > ROT_STEP_DURN_MS)
    {
      clrRequests();
      stopDrive();
      stopTimer(&cwRot_startTi);
    }

  }

  if (isCCWRotRequested())
  {
    startTimer(&ccwRot_startTi);
    setMotorPwm(ROT_PWM , ROT_PWM);
    ccwRot();
    if (millis() - ccwRot_startTi > ROT_STEP_DURN_MS)
    {
      clrRequests();
      stopDrive();
      stopTimer(&ccwRot_startTi);
    }

  }


}

void definePinModes()
{
  pinMode(REV_RGT, OUTPUT); //d5
  pinMode(PWMLFT, OUTPUT); //d6
  pinMode(13, OUTPUT); //d7
  pinMode(FWD_RGT, OUTPUT);  //d3
  pinMode(REV_LFT, OUTPUT);  //d4
  pinMode(FWD_LFT, OUTPUT);  //d2
  pinMode(PWMRGT, OUTPUT);  //d1
}

void initPinStates()
{
  digitalWrite(PWMRGT, LOW);  //d1 pwm
  digitalWrite(PWMLFT, LOW); //d6 pwm
  digitalWrite(FWD_LFT, LOW);  //d2
  digitalWrite(FWD_RGT, LOW);  //d3
  digitalWrite(REV_LFT, LOW);  //d4
  digitalWrite(REV_RGT, LOW); //d5
  digitalWrite(13, LOW); //d7
}

// initialization
void initDriveTrain()
{
  definePinModes();
  initPinStates();
}
