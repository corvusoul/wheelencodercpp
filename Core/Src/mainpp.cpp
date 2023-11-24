/*
 * mainpp.cpp
 *
 *  Created on: Nov 20, 2023
 *      Author: S Vedram
 */



#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include <mainpp.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <ros/time.h>

int flag = 0;
 int leftenc = 0, leftenco = 0, rightenc = 0, rightenco = 0;
 int pos_act_left = 0, pos_act_right = 0;
 int rightvel = 0, rpm = 0;
 float demandx=0;
 float demandz=0;
 float temp = 0.0;
 double demand_speed_left;
 double demand_speed_right;

 void cmd_vel_cb( const geometry_msgs::Twist& twist){
   demandx = twist.linear.x;
   demandz = twist.angular.z;
 }


ros::NodeHandle nh;
std_msgs::Int16 left_wheel_msg;
ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);
std_msgs::Int16 right_wheel_msg;
ros::Publisher right_wheel_pub("rwheel", &right_wheel_msg);

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );

void publishPos()
{
  left_wheel_msg.data = pos_act_left;
  right_wheel_msg.data = pos_act_right;
  left_wheel_pub.publish(&left_wheel_msg);
  right_wheel_pub.publish(&right_wheel_msg);
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	rpm = (int)((rightenc - rightenco)*0.6); //(No. of ticks Now - No. of Ticks Old)*60/(50*10^-3) (old calc)
	rightvel = (int)((rpm * 6.28 * 7.5)/(60));
	rightenco = rightenc;
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	pos_act_left = leftenc;
	pos_act_right = rightenc;

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	if(GPIO_PIN == LeftEncoderA_Pin)
	{
		if(HAL_GPIO_ReadPin(LeftEncoderA_GPIO_Port, LeftEncoderA_Pin) == 1)
		{
			if(HAL_GPIO_ReadPin(GPIOA, LeftEncoderB_Pin) == 1) leftenc++;
			else if(HAL_GPIO_ReadPin(GPIOA, LeftEncoderB_Pin) == 0) leftenc--;
		}
		else if(HAL_GPIO_ReadPin(LeftEncoderA_GPIO_Port, LeftEncoderA_Pin) == 0)
		{
			if(HAL_GPIO_ReadPin(GPIOA, LeftEncoderB_Pin) == 0) leftenc++;
			else if(HAL_GPIO_ReadPin(GPIOA, LeftEncoderB_Pin) == 1) leftenc--;
		}
	}
	else if(GPIO_PIN == RightEncoderA_Pin)
	{
		if(HAL_GPIO_ReadPin(RightEncoderA_GPIO_Port, RightEncoderA_Pin) == 1)
		{
			if(HAL_GPIO_ReadPin(GPIOB, RightEncoderB_Pin) == 0) rightenc++;
			else if(HAL_GPIO_ReadPin(GPIOB, RightEncoderB_Pin) == 1) rightenc--;
		}
		else if(HAL_GPIO_ReadPin(RightEncoderA_GPIO_Port, RightEncoderA_Pin) == 0)
		{
			if(HAL_GPIO_ReadPin(GPIOB, RightEncoderB_Pin) == 1) rightenc++;
			else if(HAL_GPIO_ReadPin(GPIOB, RightEncoderB_Pin) == 0) rightenc--;
		}
	}
	flag = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup()
{
	nh.initNode();
	nh.advertise(left_wheel_pub);
	nh.advertise(right_wheel_pub);
	nh.subscribe(sub);


}
void loop()
{
	publishPos();
	nh.spinOnce();

	demand_speed_left = demandx - (demandz*temp);
	demand_speed_right = demandx + (demandz*temp);
	if(demandx > 0)
	{
		TIM3->CCR1 = 50;
		TIM3->CCR2 = 50;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 0;
	}
	else if(demandx < 0)
	{
		TIM3->CCR1 = 0;
        TIM3->CCR2 = 0;
		TIM3->CCR3 = 50;
		TIM3->CCR4 = 50;
	}
	else if (demandx == 0)
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 0;
	}
	HAL_Delay(10);
}
