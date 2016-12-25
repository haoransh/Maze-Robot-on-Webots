/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
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

int i;
int curled=0;
double dis_left,dis_right,dis_forward;
double threshold,wallThreshold;
double ps_values[8];
double left_speed;
double right_speed;

int dir = 0; //右转加1，左转减1

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

void toSouth()
{
  //纠正方向，z方向与北对齐
  wb_robot_step(TIME_STEP);
  compass_values=wb_compass_get_values(compass);
  printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
  while(compass_values[2] < 0 || fabs(compass_values[0]) > 0.002)
  {
    printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
    if(compass_values[2] < 0)
    {
      //右转
      wb_differential_wheels_set_speed(100,-100);
      wb_robot_step(TIME_STEP); 
      compass_values=wb_compass_get_values(compass);
      continue;
    }
    if(compass_values[0] > 0)
    {
      //左转
      wb_differential_wheels_set_speed(-20,20);
      wb_robot_step(TIME_STEP);  
      compass_values=wb_compass_get_values(compass);
    }
    else
    {
      //右转
      wb_differential_wheels_set_speed(20,-20);
      wb_robot_step(TIME_STEP); 
      compass_values=wb_compass_get_values(compass);
    }
  }
  printf("have already faced South!---------------\n");
}

void toWest()
{
  //纠正方向，z方向与东对齐，x方向与北对齐
  wb_robot_step(TIME_STEP);
  compass_values=wb_compass_get_values(compass);
  printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
  while(compass_values[0] < 0 || fabs(compass_values[2]) > 0.002)
  {
    printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
    if(compass_values[0] < 0)
    {
      //右转
      wb_differential_wheels_set_speed(100,-100);
      wb_robot_step(TIME_STEP); 
      compass_values=wb_compass_get_values(compass);
      continue;
    }
    if(compass_values[2] > 0)
    {
      //右转
      wb_differential_wheels_set_speed(20,-20);
      wb_robot_step(TIME_STEP);  
      compass_values=wb_compass_get_values(compass);
    }
    else
    {
      //左转
      wb_differential_wheels_set_speed(-20,20);
      wb_robot_step(TIME_STEP); 
      compass_values=wb_compass_get_values(compass);
    }
  }
  printf("have already faced West!---------------\n");
}

void toNorth()
{
  //纠正方向，z反方向与北对齐
  wb_robot_step(TIME_STEP);
  compass_values=wb_compass_get_values(compass);
  printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
  while(compass_values[2] > 0 || fabs(compass_values[0]) > 0.002)
  {
    printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
    if(compass_values[2] > 0)
    {
      //右转
      wb_differential_wheels_set_speed(100,-100);
      wb_robot_step(TIME_STEP); 
      compass_values=wb_compass_get_values(compass);
      continue;
    }
    if(compass_values[0] > 0)
    {
      //右转
      wb_differential_wheels_set_speed(20,-20);
      wb_robot_step(TIME_STEP);  
      compass_values=wb_compass_get_values(compass);
    }
    else
    {
      //左转
      wb_differential_wheels_set_speed(-20,20);
      wb_robot_step(TIME_STEP); 
      compass_values=wb_compass_get_values(compass);
    }
  }
  printf("have already faced North!---------------\n");
}

void toEast()
{
  //纠正方向，z方向与东对齐，x方向与北对齐
  wb_robot_step(TIME_STEP);
  compass_values=wb_compass_get_values(compass);
  printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
  while(compass_values[0] > 0 || fabs(compass_values[2]) > 0.002)
  {
    printf("compass: %lf %lf %lf\n",compass_values[0],compass_values[1],compass_values[2]);
    if(compass_values[0] > 0)
    {
      //右转
      wb_differential_wheels_set_speed(100,-100);
      wb_robot_step(TIME_STEP); 
      compass_values=wb_compass_get_values(compass);
      continue;
    }
    if(compass_values[2] > 0)
    {
      //左转
      wb_differential_wheels_set_speed(-20,20);
      wb_robot_step(TIME_STEP);  
      compass_values=wb_compass_get_values(compass);
    }
    else
    {
      //右转
      wb_differential_wheels_set_speed(20,-20);
      wb_robot_step(TIME_STEP); 
      compass_values=wb_compass_get_values(compass);
    }
  }
  printf("have already faced East!---------------\n");
}

void correctDir()
{
  int tmp = dir%4;
  switch (tmp)
  {
  case 0: { toNorth(); break; }
  case 1: { toEast(); break; }
  case 2: { toSouth(); break; }
  case 3: { toWest(); break; }
  default: break;
  }
  return;
}


void turnLeft()
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

void turnRight()
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

void turn()
{
  if(!(reachable()>>2))
  {
     if(reachable()==0)
     {
         turnRight();
         dir ++;
         correctDir();
         turnRight();
         dir ++;
         correctDir();
     }
     if(reachable()==1)
     {
        turnRight();
        dir ++;
        correctDir();
     }
     else 
        return;
 }
  else
  {
    turnLeft();
    dir --;
    correctDir();
    wb_differential_wheels_set_speed(left_speed, right_speed);
    for(i=0;i<65;++i)
        wb_robot_step(TIME_STEP);
    ps_values[5] = wb_distance_sensor_get_value(ps[5]);
    if(ps_values[5]<wallThreshold)
    {
      printf("No Access\n");
      turnLeft();
      dir --;
      correctDir();
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
  correctDir();
  
  //靠近左边的墙壁
  //左转
  wb_differential_wheels_set_speed(-DEFAULT_SPEED, DEFAULT_SPEED);
  for(i = 0;i < 55;++ i)
  {
    wb_robot_step(TIME_STEP);
  }
  printf("have already turned left!-------------\n");
  //前进
  ps_values[2] = wb_distance_sensor_get_value(ps[2]);
  ps_values[7] = wb_distance_sensor_get_value(ps[7]);
  double cur_forward = (ps_values[0]+ps_values[7])/2;
  while(cur_forward < 200)
  {
    printf("current forward sensor value:%.1f\n", cur_forward);
     wb_differential_wheels_set_speed(DEFAULT_SPEED, DEFAULT_SPEED);
     wb_robot_step(TIME_STEP);
     ps_values[2] = wb_distance_sensor_get_value(ps[2]);
     ps_values[7] = wb_distance_sensor_get_value(ps[7]);
     cur_forward = (ps_values[0]+ps_values[7])/2;
  }
  printf("have already approached wall!-------------\n");
  
  correctDir();
  
  //pledge algorithm
  while (1) 
  {
    int delay = wb_robot_step(TIME_STEP);
    if (delay == -1)
       break;
       
    for (i = 0; i < 8 ; i++)
        ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    
    if(dir == 0)
    {
      //towards North
      
      //前进直到撞墙
      double cur_forward = (ps_values[0]+ps_values[7])/2;
      while(cur_forward < 200)
      {
         printf("current forward sensor value:%.1f\n", cur_forward);
         wb_differential_wheels_set_speed(DEFAULT_SPEED, DEFAULT_SPEED);
         wb_robot_step(TIME_STEP);
         ps_values[2] = wb_distance_sensor_get_value(ps[2]);
         ps_values[7] = wb_distance_sensor_get_value(ps[7]);
         cur_forward = (ps_values[0]+ps_values[7])/2;
      }
      //右转
      turnRight();
      dir ++;
      toEast();
    }
    else
    {
      //wall following
      if(wallChanged()) //三种情况
      {
        turn();
      }
      else //直走
      {
        wb_differential_wheels_set_speed(DEFAULT_SPEED, DEFAULT_SPEED);
      }
    }
  
  }
  wb_robot_cleanup();
  return 0;
}
