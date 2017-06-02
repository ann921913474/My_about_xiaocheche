#include "ctrl.h"
#include "FlashUI.h"
#include "pwm.h"

extern PID streepid;       //转向pid
extern PID speedpid;       //速度pid
extern int speed;          //速度
extern PID linepid;        //直线pid

extern int ad1;
extern int ad2;
extern int ad3;
extern int average1,average2;
int ad1_value;              //ad1滤波后的值
int ad2_value;              //ad2滤波后的值
extern float sum_diff;
int ad_all;
//int max1,min1;                //滤波最大最小值
//int max2,min2;                //滤波最大最小值
//int sum1,sum2;                //虑完波的值
int stop=0;

int ENCODE;
int resultturn;

//int result;                //
float error[5];

u8 inclu;   //丢线标志位


void speedCtrl(void)
{
	u32 duty = 0;
	speedPID(ENCODE);
	duty +=result;
	
	if(duty >5000)
	duty = 5000;
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH3,0);
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2,duty);
}

//速度控制
void speedPID(u32 nowValue)                    
{
	 float  SpeedKP = speedpid.p ;
   float  SpeedKI = speedpid.i ;
   float  SpeedKD = speedpid.d ;

   static int16_t LastSpeedCut0,LastSpeedCut1,LastSpeedCut2 ,SpeedLastPWMK ;
   int16_t  SpeedPWMKP ,SpeedPWMKI ,SpeedPWMKD,SpeedPWMK ;
   int16_t  SpeedPWMOUT;
   int16_t  SpeedDifference0=0;
   int16_t  speedDEARE1,speedDEARE2,DSpeed ;

   LastSpeedCut0 =  nowValue ;                //编码器的值
   DSpeed =(int16_t) speed*10;                   //速度赋值

   SpeedDifference0 = DSpeed - LastSpeedCut0  ; 
   speedDEARE1 = LastSpeedCut0 - LastSpeedCut1;  
   speedDEARE2 = LastSpeedCut2+LastSpeedCut0-2*LastSpeedCut1;  
   LastSpeedCut2  = LastSpeedCut1;
   LastSpeedCut1  = LastSpeedCut0;

          SpeedPWMKP = SpeedKP*SpeedDifference0;   
          if(SpeedPWMKP>KPPLUSMAX){
           SpeedPWMKP = KPPLUSMAX;
          }else if (SpeedPWMKP <KPNEGATIVEMAX){
          SpeedPWMKP = KPNEGATIVEMAX;
          }
          SpeedPWMKI = SpeedKI* speedDEARE1;
          if(SpeedPWMKI > KIPLUSMAX){
          SpeedPWMKI = KIPLUSMAX;
          } else if(SpeedPWMKI < KINEGATIVEMAX){
          SpeedPWMKI = KINEGATIVEMAX;
          }
          SpeedPWMKD = SpeedKD* speedDEARE2;
          if(SpeedPWMKD > KDPLUSMAX){
             SpeedPWMKD = KDPLUSMAX;
          } else if(SpeedPWMKD < KDNEGATIVEMAX){
            SpeedPWMKD = KDNEGATIVEMAX;
          }
          SpeedPWMK = SpeedPWMKD-SpeedPWMKI+SpeedPWMKP ;
          if(SpeedPWMK > KWPLUSMAX){
          SpeedPWMK = KWPLUSMAX;
          }else if(SpeedPWMK < KWNEGATIVEMAX){
          SpeedPWMK = KWNEGATIVEMAX;
          }
         SpeedPWMOUT = SpeedLastPWMK + SpeedPWMK ;
         if(SpeedPWMOUT < 0 ){
            SpeedPWMOUT = 0 ;
         } else if(SpeedPWMOUT > KOUPLUSMAX){
           SpeedPWMOUT = KOUPLUSMAX ;
         }
         SpeedLastPWMK = SpeedPWMOUT ;
         result = SpeedPWMOUT;	
}


//转向控制
void streePID(int nowValue)
{
//	FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH1, 1660);            //1940  1660  1370

	
    float stree_p,stree_i,stree_d;
	  float stree_zhi_p,stree_zhi_i,stree_zhi_d;
    int pid_count = 0,pid_errsum = 0;
		float HIGH_RUN;
		int Kpvalue;
	  int Kdvalue;
    float p1,p2,p3,p4;
	  int x;
	
    stree_p = streepid.p;
    stree_i = streepid.i;
    stree_d = streepid.d;
	
	  p1 =   4.969e-09 ;
		p2 =  -2.026e-07  ;
		p3 =    0.008555 ; 
		p4 =      0.8588 ;
	  x = p1*nowValue*nowValue*nowValue + p2*nowValue*nowValue + p3*nowValue + p4;
	
		error[0] = x;
		Kpvalue =(int) (stree_p * error[0]);
		Kdvalue = stree_d * (error[0] - error[1]);
		resultturn =3500 + Kpvalue + Kdvalue;
		error[1] = error[0];		
  }

	
void linePID(int nowValue)
{
//	FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH1, 1660);            //1940  1660  1370

	
    float stree_p,stree_i,stree_d;
	  float stree_zhi_p,stree_zhi_i,stree_zhi_d;
    int pid_count = 0,pid_errsum = 0;
		float HIGH_RUN;
   
	  
	
    stree_p = linepid.p;
    stree_i = linepid.i;
    stree_d = linepid.d;

//		x = p1*nowValue+p2;
	
	  
	  
	
		error[0] = nowValue;
		pid_errsum +=  error[1] * stree_i;
		resultturn =(int) (stree_p * error[1]);
		resultturn += pid_errsum;
		resultturn += stree_d * (error[1] - error[0]);
//		pid_count ++;
		error[1] = error[0];
		
  }
	
void ctrl(void)
{
	//ad1是右边的
	int spe;   //丢线标志位 1左，2右
//	int ad;
	int overflow1=0,overflow2=0;     //检测旁边跑到变量
	int sum_ad=0;
	u8 side_flag=0;                    //检测旁边跑道标志位
//	u8 stop=0;
	u8 Lose=0;
	
	if((ad1>30)||(ad3>30))
	{
		spe=0;
//	  ad_all=((average1-average2)/2);
		
//		ad_all=(((double)sqrt(average1)*(double)1500)-((double)sqrt(average2)*(double)1500))/((double)(average1)+(double)average2);
		
//		ad_all=((average1+average2)/(average1-average2));
//		resultturn=Turn_speed(sum_ad);
//		ad_all=(1.986*(sum_ad/1000)*(sum_ad/1000)*(sum_ad/1000) + 0.2274*(sum_ad/1000)*(sum_ad/1000) + 0.006591*sum_ad + 15.44-15);

//	  if(average1>700&&average2>700)
//			linePID(ad_all);
//		else
		  streePID(sum_diff);
		  stop=0;
		
	}
	else
	{
//		stop++;
//	  if(stop>=4000)
//		{
//		FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2,0);
////		stop=0;
//		}
	}
//		sum_diff=0;
/******************************丢线*************************/
	if(ad1>200&&ad3>200&&ad1>ad3)
		Lose=1;
	else if(ad1>200&&ad3>200&&ad1<ad3)
		Lose=2;
	else
		;
	if(Lose==1)
	{
		if(ad1<200&&ad3<200)
		{
			spe=1;          //左边丢线
		}
  }
	if(Lose==2)
	{
		if(ad1<200&&ad3<200)
		{
	//		if(spe==0)
				 spe=2;          //右边丢线
		}
  }
	switch(spe)
	{
		case 1:
			resultturn=3500+950;
		  break;
		case 2:
			resultturn=3500-900;
		  break;
		default:
			break;
	}
/*************************丢线********************************/
		FTM_PWM_ChangeDuty(HW_FTM1, HW_FTM_CH1, resultturn);    //  -900  3500  +900  resultturn
}

//void filter(void)       //滤波
//{
//	u8 count;
//	int value_buf1[6],value_buf2[6];
//	int i=0;

//	value_buf1[i]=ad1;
//	value_buf2[i]=ad2;
//	
//	if(i==0)
//	{
//		sum1=ad1;
//		sum2=ad2;
//	}
//	i++;
//	
//	if(value_buf1[i]>max1)
//		max1=value_buf1[i];
//	if(value_buf1[i]<min1)
//		min1=value_buf1[i];
//	
//	if(value_buf2[i]>max2)
//		max2=value_buf2[i];
//	if(value_buf2[i]<min2)
//		min2=value_buf2[i];
//	
//	if(i==6)
//	{
//		i=0;
//		sum1=(sum1-min1)/(max1-min1);    //新数据=（原数据-极小值）/（极大值-极小值）   现在是最大最小值
//		sum2=(sum2-min2)/(max2-min2);
//		max1=0;
//		min1=0;
//		max2=0;
//		min2=0;
//	}
//  
//}







///**********************防检测旁边赛道*********************************************/
////	overflow1=ad;            //当前值赋给overflow1
//  
//	
//	if(average2<20)            //左边为0 判断右边
//	{
//		overflow1=average1;
//		if((average1<1200)||((overflow1-overflow2)>10))  //这次减上次大于0
//		{
//			side_flag=1;
//		}
//		else
//		{
//			side_flag=0;
//		}
//		
//		overflow2=overflow1;
//	}
//	
//	if(average1<20)            //右边边为0 判断左边
//	{
//		overflow1=average2;
//		if((average2<1200)&&((overflow2-overflow1)>10))  //这次减上次大于0  向内移动
//		{
//			side_flag=1;
//		}
//		else
//		{
//			side_flag=0;
//		}
//		overflow2=overflow2;
//	}
////	overflow2=overflow1;     //上次值赋给overflow2
///**********************************************************************/
	
	
	int Turn_speed(int sum_ad)
{
  int pwm_turnD;
  int turn_pwm;
  int turn_error_max=0;//(??)
  int turnErrSave=0;
	int error[4],GoStritOn[5];
  float turn_Kd=streepid.p*0.1;
  float turn_Kp=streepid.d*0.1;
//  turn.Error[0]=turn_e;
  
   turn_error_max=(1.986*(sum_ad/1000)*(sum_ad/1000)*(sum_ad/1000) + 0.2274*(sum_ad/1000)*(sum_ad/1000) + 0.006591*sum_ad + 15.44-15); //16??? 
            error[0]=turn_error_max;
            if(error[0]>300) error[0] = 200;//????
			   if(error[0]<-300) error[0] = -200;//????
            
            turnErrSave=error[0];
            GoStritOn[4]= GoStritOn[3];
            GoStritOn[3]= GoStritOn[2];
            GoStritOn[2]= GoStritOn[1];
            GoStritOn[1]= GoStritOn[0];
	GoStritOn[0]= error[0]-GoStritOn[1]>8?GoStritOn[1]+8:error[0]-GoStritOn[1]<-8?GoStritOn[1]-8:error[0];//????  GoStritOn[1]-Turn.Error>5? GoStritOn[1]-5: 
            //8??
            
            error[0]= GoStritOn[0];//(int )(GoStritOn[0] * 0.7 +GoStritOn[1]*0.3);
            turnErrSave=GoStritOn[0];
  
  
  pwm_turnD = (error[0]-error[1])*turn_Kd;  //????d?
  if(pwm_turnD>turn_Kd*8) pwm_turnD = turn_Kd*8;
  else if(pwm_turnD<-turn_Kd*8) pwm_turnD = -turn_Kd*8;
//  TurnP_LINSHI=Normal.Turn_Pnum;
  turn_pwm = (int)((GoStritOn[0]*turn_Kp*0.8+pwm_turnD));//*0.1
  error[2] = error[1];
  error[1] = GoStritOn[0];
  
  return turn_pwm;
}