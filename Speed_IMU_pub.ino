#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
std_msgs::Float32MultiArray msg;
ros::Publisher chatter("chatter", &msg);
ros::NodeHandle nh;

float all_measurement[7]={0,0,0,0,0,0,0};


HardwareTimer timer(TIM4);
volatile int counter1=0;
volatile int counter2=0;
volatile int counter3=0;
volatile int counter4=0;

int sample_time_ms = 200;
double sample_time_sec = (double)sample_time_ms/1000;
float Interrupt_FREQ=1/sample_time_sec;
int timerOverflow = 65535;    //16 bit registry

const int fori = 20;
double diameter = 79.0/1000.0; 


#include "motion_fx.h"

#include "LSM6DSLSensor.h"

#define ALGO_FREQ  100U /* Algorithm frequency 100Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f

#define STATE_SIZE                      (size_t)(2432)

#define SAMPLETODISCARD                 15

#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)
#define GBIAS_MAG_TH_SC                 (2.0f*0.001500f)

#define DECIMATION                      1U

#if !(__CORTEX_M == 0U)
static MFX_knobs_t iKnobs;
static MFX_knobs_t *ipKnobs = &iKnobs;
static uint8_t mfxstate[STATE_SIZE];
#endif

static volatile int sampleToDiscard = SAMPLETODISCARD;
static int discardedCount = 0;

char LibVersion[35];
int LibVersionLen;

static volatile uint32_t TimeStamp = 0;

int32_t accelerometer[3];
int32_t gyroscope[3];
int32_t magnetometer[3];
int32_t MagOffset[3];


LSM6DSLSensor AccGyr(&Wire);

HardwareTimer *MyTim;

volatile uint8_t fusion_flag;

bool mag_calibrated = true;

void fusion_update(void)
{
  fusion_flag = 1;
}



MFX_input_t data_in; 
MFX_output_t data_out;
float delta_time = MOTION_FX_ENGINE_DELTATIME;




void count1() {
  counter1++;
}
void count2(){
  counter2++;
}
void count3() {
  counter3++;
}
void count4(){
  counter4++;
}

void WheelsSpeed(){
  double number_of_rounds1 = (double)counter1/fori;
  double number_of_rounds2 = (double)counter2/fori;
  double number_of_rounds3 = (double)counter3/fori;
  double number_of_rounds4 = (double)counter4/fori;

  double timeNow_sec = timer.getCount()/timerOverflow;
  
  double angular_speed1 = (number_of_rounds1)/(sample_time_sec+timeNow_sec);
  double angular_speed2 = (number_of_rounds2)/(sample_time_sec+timeNow_sec);
  double angular_speed3 = (number_of_rounds3)/(sample_time_sec+timeNow_sec);
  double angular_speed4 = (number_of_rounds4)/(sample_time_sec+timeNow_sec);
 
  all_measurement[0] = angular_speed1 * PI * diameter;
  all_measurement[1] = angular_speed2 * PI * diameter;
  all_measurement[2] = angular_speed3 * PI * diameter;
  all_measurement[3] = angular_speed4 * PI * diameter;
  
  counter1 = 0;
  counter2 = 0;
  counter3 = 0;
  counter4 = 0;
  
  timer.setCount(1);
  
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}







void setup() {
  /* Initialize Serial */
  Serial.begin(115200);
  pinMode(PB13,INPUT);
  pinMode(PB14,INPUT);
  pinMode(PB1,INPUT);
  pinMode(PB2,INPUT);
  
  float ans_float;
  MFX_MagCal_input_t mag_data_in;
  MFX_MagCal_output_t mag_data_out;
  
  while (!Serial) yield();

  /* Initialize LED */
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialize I2C bus */
  Wire.begin();
  Wire.setClock(400000);

  /* Start communication with IMU */
  AccGyr.begin();
  AccGyr.Set_X_ODR((float)ALGO_FREQ);
  AccGyr.Set_X_FS(4);
  AccGyr.Set_G_ODR((float)ALGO_FREQ);
  AccGyr.Set_G_FS(2000);
  AccGyr.Enable_X();
  AccGyr.Enable_G();
  delay(10);
  
  MotionFX_initialize((MFXState_t *)mfxstate);

  MotionFX_getKnobs(mfxstate, ipKnobs);

  ipKnobs->acc_orientation[0] = 'n';
  ipKnobs->acc_orientation[1] = 'w';
  ipKnobs->acc_orientation[2] = 'u';
  ipKnobs->gyro_orientation[0] = 'n';
  ipKnobs->gyro_orientation[1] = 'w';
  ipKnobs->gyro_orientation[2] = 'u';
  ipKnobs->mag_orientation[0] = 'n';
  ipKnobs->mag_orientation[1] = 'e';
  ipKnobs->mag_orientation[2] = 'u';

  ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
  ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
  ipKnobs->gbias_mag_th_sc = GBIAS_MAG_TH_SC;

  ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
  ipKnobs->LMode = 1;
  ipKnobs->modx = DECIMATION;

  MotionFX_setKnobs(mfxstate, ipKnobs);
  MotionFX_enable_6X(mfxstate, MFX_ENGINE_DISABLE);
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_ENABLE);

  /* OPTIONAL */
  /* Get library version */
  LibVersionLen = (int)MotionFX_GetLibVersion(LibVersion);

  /* Enable magnetometer calibration */
  MotionFX_MagCal_init(ALGO_PERIOD, 1);
  


  nh.initNode();
  msg.data_length = 7;
  nh.advertise(chatter);
  
  MyTim = new HardwareTimer(TIM3);
  MyTim->setOverflow(ALGO_FREQ, HERTZ_FORMAT);
  MyTim->attachInterrupt(fusion_update);
  MyTim->resume();

  timer.setOverflow(Interrupt_FREQ, HERTZ_FORMAT);
  timer.attachInterrupt(WheelsSpeed);
  timer.refresh();
  timer.resume();
  
  attachInterrupt(PB13,count1,RISING);
  attachInterrupt(PB14,count2,RISING);
  attachInterrupt(PB1,count3,RISING);
  attachInterrupt(PB2,count4,RISING);
        
}








void loop() {
    if(fusion_flag)
    {
      fusion_flag = 0;
      AccGyr.Get_X_Axes(accelerometer);
      AccGyr.Get_G_Axes(gyroscope);

      /* Convert angular velocity from [mdps] to [dps] */
      data_in.gyro[0] = (float)gyroscope[0] * FROM_MDPS_TO_DPS;
      data_in.gyro[1] = (float)gyroscope[1] * FROM_MDPS_TO_DPS;
      data_in.gyro[2] = (float)gyroscope[2] * FROM_MDPS_TO_DPS;

      /* Convert acceleration from [mg] to [g] */
      data_in.acc[0] = (float)accelerometer[0] * FROM_MG_TO_G;
      data_in.acc[1] = (float)accelerometer[1] * FROM_MG_TO_G;
      data_in.acc[2] = (float)accelerometer[2] * FROM_MG_TO_G;

      /* Don't set mag values because we use only acc and gyro */
      data_in.mag[0] = 0.0f;
      data_in.mag[1] = 0.0f;
      data_in.mag[2] = 0.0f; 
       
      if (discardedCount == sampleToDiscard)
      {
        MotionFX_propagate(mfxstate, &data_out, &data_in, &delta_time);
        MotionFX_update(mfxstate, &data_out, &data_in, &delta_time, NULL);
      }
      else
      {
        discardedCount++;
      }

      all_measurement[4]=data_out.rotation[0];
      if(all_measurement[4]>180){
        all_measurement[4]=mapfloat(all_measurement[4],180,360,-180,0);
      }
      
      all_measurement[5]=data_out.linear_acceleration[0];
      all_measurement[6]=data_out.linear_acceleration[1];
    }
      msg.data=all_measurement;
      chatter.publish( &msg );  
  
      nh.spinOnce();
    
  
  Serial.print(all_measurement[0]);
  Serial.print(" ");
  Serial.print(all_measurement[1]);
  Serial.print(" ");
  Serial.print(all_measurement[2]);
  Serial.print(" ");
  Serial.print(all_measurement[3]);
  Serial.print(" ");
  Serial.print(all_measurement[4]);
  Serial.print(" ");
  Serial.print(all_measurement[5]);
  Serial.print(" ");
  Serial.print(all_measurement[6]);
  Serial.println(" ");
  
}
