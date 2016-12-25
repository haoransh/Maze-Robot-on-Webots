#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <webots/compass.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>

#define TIME_STEP 16
#define DEFAULT_SPEED 400
#define NB_LEDS 10
#define ON        1
#define OFF       0
#define COMPASS_BIAS 100
int i;
int curled=0;
double dis_left,dis_right,dis_forward;
double threshold,wallThreshold;
double ps_values[8];
double left_speed;
double right_speed;
double history_ds[20]={0};

WbDeviceTag led[NB_LEDS];

WbDeviceTag ps[8];
char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
};

WbDeviceTag compass;
double* compass_values;

WbDeviceTag cam;
int width, height;
void init()
{

  wb_robot_init();
  wb_differential_wheels_enable_encoders(TIME_STEP);
  wb_differential_wheels_set_encoders(0,0);
  compass=wb_robot_get_device("compass");
  wb_compass_enable(compass,TIME_STEP);
  for (i=0; i<8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  char text[5]="led0";  
  for(i=0;i<NB_LEDS;i++) {
    led[i]=wb_robot_get_device(text);
    printf("led device\n");
    text[3]++;
  }
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam,TIME_STEP);
  width = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam); 
  dis_left     =0.0;
  dis_right    =0.0;
  dis_forward  =0.0;
  threshold    = 100;
  wallThreshold = 160;
  left_speed   =DEFAULT_SPEED;
  right_speed  =DEFAULT_SPEED;
  srand(time(NULL));
}


/*void led_switch() {
  int i;
  for(i=0;i<NB_LEDS;i++) {
    wb_led_set(led[i],OFF);
  }
  i=curled;
  wb_led_set(led[i],ON);
  curled++;
}*/

int leftWall()
{
    double cur_left = ps_values[5];
    double diff = fabs(cur_left-dis_left);
    dis_left = cur_left;
    if(diff>threshold)
      return 1;
    else
      return 0;
}

int rightWall()
{
    double cur_right = ps_values[2];
    double diff = fabs(cur_right-dis_right);
    dis_right = cur_right;
    if(diff>threshold)
      return 1;
    else
      return 0;
}

int forwardWall()
{
    double cur_forward = (ps_values[0]+ps_values[7])/2;
    double diff = fabs(cur_forward-dis_forward);
    dis_forward = cur_forward;
    if(diff>threshold)
      return 1;
    else
      return 0;
}

bool wallChanged()
{
  return leftWall()||rightWall()||forwardWall();
}

double ave(double *a, int len)
{
  double sum=0;
  for(i=0;i<len;++i)
    sum+=a[i];
  sum=sum/len;
  return sum;
}

void bias_back()
{
  //led_switch();
  double curvalue = wb_distance_sensor_get_value(ps[5]);
  for(i=0;i<19;++i)
    history_ds[i]=history_ds[i+1];
  history_ds[9]=curvalue;
  double avef=ave(history_ds,10);
  double avel=ave(history_ds+10,10);
  if(avef-avel>100)
  {
    printf("Correct Bias\n");
    wb_differential_wheels_set_speed(100,200);
    wb_robot_step(TIME_STEP);
  }
  compass_values=wb_compass_get_values(compass);
  //printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]); 
  //printf("Correct Compass\n");
  while (compass_values[0]>0&&compass_values[0]<0.5)
  {
      if(compass_values[0]>0.02)
      {
        wb_differential_wheels_set_speed(COMPASS_BIAS,-COMPASS_BIAS);
        wb_robot_step(TIME_STEP);     
        printf("%lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
      } else break;
    compass_values=wb_compass_get_values(compass);
  }
  while (compass_values[0]>0.5&&compass_values[0]<1)
    {
      if(compass_values[0]<0.98)
      {
        wb_differential_wheels_set_speed(COMPASS_BIAS,-COMPASS_BIAS);
        wb_robot_step(TIME_STEP);
        printf("%lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
      } else break;
    compass_values=wb_compass_get_values(compass);
    }
  while (compass_values[0]<-0.5)
    {
      if(compass_values[0]>-0.98)
      {
        wb_differential_wheels_set_speed(COMPASS_BIAS,-COMPASS_BIAS);
        wb_robot_step(TIME_STEP);
        printf("%lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
      } else break;
    compass_values=wb_compass_get_values(compass);
    }
  while (compass_values[0]<0&&compass_values[0]>-0.5)
    {
      if(compass_values[0]<-0.02)
      {
        wb_differential_wheels_set_speed(COMPASS_BIAS,-COMPASS_BIAS);
        wb_robot_step(TIME_STEP);
            printf("%lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);

      } else break;
    compass_values=wb_compass_get_values(compass);
    }
    compass_values=wb_compass_get_values(compass);
    //printf("%lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
}

int reachable()
{
    bool left_free =
      ps_values[5] < wallThreshold;
    bool right_free =
      ps_values[2] < wallThreshold;
    bool forward_free =
      ps_values[0] < wallThreshold &&
      ps_values[7] < wallThreshold;
    return (int)left_free<<2|
    (int)forward_free<<1|
    (int)right_free;
}

void turn_left()
{
  for(i=0;i<45;++i)
    wb_robot_step(TIME_STEP);
  printf("Turn Left\n");
  left_speed = -DEFAULT_SPEED;
  right_speed =DEFAULT_SPEED;
  wb_differential_wheels_set_speed(left_speed, right_speed);
  for(i=0;i<55;++i)
    wb_robot_step(TIME_STEP);
  left_speed = DEFAULT_SPEED;
  right_speed =DEFAULT_SPEED;
  wb_differential_wheels_set_speed(left_speed, right_speed);
}

void turn_right()
{
  for(i=0;i<45;++i)
    wb_robot_step(TIME_STEP);
  printf("Turn Right\n");
  left_speed = DEFAULT_SPEED;
  right_speed =-DEFAULT_SPEED;
  wb_differential_wheels_set_speed(left_speed, right_speed);
  for(i=0;i<55;++i)
    wb_robot_step(TIME_STEP);
  left_speed = DEFAULT_SPEED;
  right_speed =DEFAULT_SPEED;
  wb_differential_wheels_set_speed(left_speed, right_speed);
}

void turn_which()
{
  if(!(reachable()>>2))
  {
     if(reachable()==0)
     {
         turn_right();
         turn_right();
     }
     if(reachable()==1)
        turn_right();
     else 
        return;
 }
  else
  {
    turn_left();
    wb_differential_wheels_set_speed(left_speed, right_speed);
    for(i=0;i<65;++i)
        wb_robot_step(TIME_STEP);
    ps_values[5] = wb_distance_sensor_get_value(ps[5]);
    if(ps_values[5]<wallThreshold)
    {
      printf("No Access\n");
      turn_left();
      wb_differential_wheels_set_speed(left_speed, right_speed);
     }
     for(i=0;i<80;++i)
        wb_robot_step(TIME_STEP);
   }
  return;
}

int main(int argc, char **argv)
{
  init();
  while (1) {
     int delay = wb_robot_step(TIME_STEP);
    if (delay == -1)
        break;
    for (i=0; i<8 ; i++)
        ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    if(wallChanged())
    {
        turn_which();
    }
    else
    {
        bias_back();
    }
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }
  wb_robot_cleanup();
  return 0;
}
