#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "alogrithm.h"
#include "settings.h"
#include "device_init.h"
#include "debug_def.h"
#include "math.h"


#define WEIZHI 0
#define TINGZHI 1
#define YUNXING 2
#define GUZHANG 3
#define MEISHANGDIAN 4

volatile Uint16 DMABuf1[80];
interrupt void epwmint_ISR(void);
interrupt void pdpint_ISR(void);
void init_data(void);
void mydelay(unsigned long int t); 
void sendpack(char start,signed int data);
void loopforever();
void ad_pianyi_jisuan(void);
void sendpack1(char start,signed int send);
void check_recevice_scic_new(void);
int scib_xmit(int a);
int scic_xmit(int a);

//global variable define
//采样变量
unsigned char adcdataindex=0;
signed int udc_int,uab_int,ubc_int,ia_int,ib_int,uab1_int,ubc1_int,ia1_int,ib1_int;// xx_buf 滤波后的值
signed int udc_offset_int,uab_offset_int,ubc_offset_int,ia_offset_int,ib_offset_int,uab1_offset_int,ubc1_offset_int,ia1_offset_int,ib1_offset_int;
float udc_fd,uab_fd,ubc_fd,ia_fd,ib_fd,uab1_fd,ubc1_fd,ia1_fd,ib1_fd,ud1_given;
unsigned int savadatacounter=0;
#define RECEVICE_BUFFER_SIZE 64
signed int recevice_buffer[RECEVICE_BUFFER_SIZE];//接收缓冲
int recevice_num=0; 
int counter1ms=0;
char tempchar;
char tempbuffer[3];

float udc_last,udc_this,udc_last2;
int udc_sample_counter=0;

char is_dead;

int il_int;
float il_fd; 
float xudianchi_voltage;
int xudianchidianya_int;
 

long int status;//bit1 bit0 indicate svg status  00--OFF  01----ON  10---FO ERROR    11-----OVERFLOWERROR
			//bit3 bit2 indicate dvr status  00--OFF  01----ON  10---FO ERROR    11-----OVERFLOWERROR
char diaodian_flags=1;//掉电标志
char diaodian_flags_last=1;


#define SVGSTATUSOFF 0
#define SVGSTATUSON 1
#define SVGSTATUSFOERROR 2
#define SVGSTATUSOVERERROR 3
#define DVRSTATUSOFF (0<<2)
#define DVRSTATUSON (1<<2)
#define DVRSTATUSFOERROR  (2<<2)
#define DVRSTATUSOVERERROR  (3<<2)
#define SVGSTATUSMASK 3
#define DVRSTATUSMASK (3<<2)

#define QUANSU_FANGDIAN_MOSHI 1   //全速放电模式
#define CHONGDIAN_MOSHI 2     //充电模式
#define STOP_MOSHI 3       //停止模式


char run_mode;

int counter500ms=0;


//临时变量
unsigned int temp_isr1,temp_isr2;
unsigned int temp_main;
float temp_isr_float,temp_isr1_float,temp_isr2_float,temp_isr3_float;

float udc_givens;
float il_givens;

float udc_kp;
float udc_ki;
float il_kp;
float il_ki;

//控制算法模块
PI_Ctrl udc_pi,il_pi;

int duty_pre;
int debug_duty_pre=3000;

FILTER myfilter;



//发送到上位机的数据变量
#define SENDBUFFSIZE 127     //0-127
char sendbuffer[SENDBUFFSIZE+1];
unsigned char readindex=0;
unsigned char writeindex=0;
//unsigned int sendtimer=0;
//char sendflags=0;

#define SENDBUFFSIZE1 63     //0-63
char sendbuffer1[SENDBUFFSIZE1+1];
unsigned char readindex1=0;
unsigned char writeindex1=0;

long int temp_long;

void main(void)
{

  // unsigned int counter;

   DINT;    
   device_init(); 
   InitPieCtrl();//Enable PIEIER.ePWM1_INT and PIEIER.ePWM1.TZINT
   vsi1_running(DISABLE);//关闭PWM输出
   vsi2_running(DISABLE);
   IER=0;
   IFR=0;
   InitPieVectTable();  
   EALLOW;  
   PieVectTable.EPWM1_TZINT=&pdpint_ISR;
   PieVectTable.EPWM1_INT=&epwmint_ISR;
   EDIS;
   IER|=(M_INT3|M_INT2);
   IFR = 0x0000; 
   init_data();   
   mydelay(100);
   ad_pianyi_jisuan();   

   mydelay(200); 
   GpioDataRegs.GPBDAT.bit.GPIO32=0;//停止过压保护
   GpioDataRegs.GPBCLEAR.bit.GPIO32=1;
   GpioDataRegs.GPBDAT.bit.GPIO33=1;//软件清除故障
   GpioDataRegs.GPBSET.bit.GPIO33=1;
   mydelay(200);
   GpioDataRegs.GPBDAT.bit.GPIO33=0;//软件不清除故障  
   GpioDataRegs.GPBCLEAR.bit.GPIO33=1; 
   EnableInterrupts();  
   ERTM;
     vsi1_running(1);//关闭PWM输出
   run_mode=STOP_MOSHI;
   status=TINGZHI;
   loopforever();
}
void loopforever()
{	
	 for(;;)
	 {
		 if(writeindex!=readindex)
		 {
		 if(scib_xmit(sendbuffer[readindex])==0){readindex++;readindex&=SENDBUFFSIZE;}
		 }

		 if(writeindex1!=readindex1)
		 {
		 if(scic_xmit(sendbuffer1[readindex1])==0){readindex1++;readindex1&=SENDBUFFSIZE1;}
		 }

	     check_recevice_scic_new();
	 }
}


interrupt void epwmint_ISR(void)//10kz
{
	temp_isr2=0; 	
	for(temp_isr1=0;temp_isr1<8;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	udc_int=temp_isr2>>3; 
	udc_int-=udc_offset_int;

	temp_isr2=0;
	for(temp_isr1=8;temp_isr1<16;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	uab_int=temp_isr2>>3;
	uab_int-=uab_offset_int;

	temp_isr2=0;
	for(temp_isr1=16;temp_isr1<24;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	ubc_int=temp_isr2>>3;
	ubc_int-=ubc_offset_int;

 //电压采样电阻91 current  100  udc     
	udc_fd=(float)(udc_int*0.2660);//(float)((udc_int*3.0)/4095.0*5.0/3.0)*166.6667;//udc_int*1.0*UDC_RATIO+UDC_BASE;//误差纠正
 	xudianchi_voltage=(float)(ubc_int*0.0814);//(float)((uab_int*3.0)/4095.0*5.0/3.0)*333.333;//uab_int*1.0*UAB_RATIO+UAB_BASE;
	il_fd=(float)(uab_int*0.0122);//(float)((ubc_int*3.0)/4095.0*5.0/3.0)*333.333;//ubc_fd=(float)((ubc_int*3.0)/4095.0*5.0/3.0-2.5)*66.6667+3;//
 
	myfilter.Xn=udc_fd;
	filter_calc(&myfilter);
 	udc_fd=myfilter.Yn;

    if(run_mode!=QUANSU_FANGDIAN_MOSHI)  //记住掉电前的电压
	{
		if(++udc_sample_counter>=5000)
		{
		  udc_sample_counter=0;
		  udc_last2=udc_last;
		  udc_last=udc_this;
		  udc_this=udc_fd;
		}
	}

	if(udc_fd>800.0 || il_fd>50.0 || il_fd<-50.0)
	{
		GpioDataRegs.GPBDAT.bit.GPIO32=1;//过压保护
		GpioDataRegs.GPBSET.bit.GPIO32=1;
	 	vsi1_running(0);
		status=GUZHANG;
		sendpack1(0xac,status);
	 	while(1)
	 	{
	 	     if(writeindex1!=readindex1)
			 {
			  if(scic_xmit(sendbuffer1[readindex1])==0){readindex1++;readindex1&=SENDBUFFSIZE1;}
			 }
		}
	} 


   if(++counter1ms>=40)
	{
  	   sendpack(0xAC,udc_fd);//volt_clarke.As);
      
       sendpack(0xae,xudianchi_voltage);
       sendpack(0xbc,il_fd*10);
 

	     counter1ms=0;
	     if(writeindex!=readindex)
		 {
		 	 if(scib_xmit(sendbuffer[readindex])==0){readindex++;readindex&=SENDBUFFSIZE;}
		 }
	}

//    if(run_mode==QUANSU_FANGDIAN_MOSHI)
//  	{
 	udc_givens= 350;
 	udc_pi.Kp=udc_kp;
	udc_pi.Ki=udc_ki;
    udc_pi.Ref=udc_givens;
	udc_pi.Fdb=udc_fd;
	pi_calc(&udc_pi);
//  	}
    
// 	if(run_mode==QUANSU_FANGDIAN_MOSHI)
// 	il_givens=-udc_pi.Out;
 // 	else 
 // 	il_givens=25;


	il_pi.Kp=il_kp;
	il_pi.Ki=il_ki;
	il_pi.Fdb=il_fd;
	il_pi.Ref=il_givens;//;	 
	pi_calc(&il_pi);
 
    duty_pre=(il_pi.Out/il_pi.OutMax+1)*PWMPeriod/2;

	if(duty_pre>=PWMPeriod*0.9)duty_pre=PWMPeriod*0.9;
	else if(duty_pre<=PWMPeriod*0.1)duty_pre=PWMPeriod*0.1;

	EPwm1Regs.CMPA.half.CMPA =duty_pre;
	EPwm2Regs.CMPA.half.CMPA =duty_pre;
	EPwm3Regs.CMPA.half.CMPA =duty_pre;
 	EPwm4Regs.CMPA.half.CMPA =duty_pre;
	EPwm5Regs.CMPA.half.CMPA =duty_pre;
	EPwm6Regs.CMPA.half.CMPA =duty_pre;

	if(++counter500ms>3000)
	{
	 counter500ms=0;
	 il_int=(int)il_fd;
	 xudianchidianya_int=(int)xudianchi_voltage;
	 sendpack1(0xbe,il_int);
	 sendpack1(0xca,xudianchidianya_int);
	 sendpack1(0xac,status);
	}

 	EPwm1Regs.ETCLR.bit.INT=1;//clear interrupt flags
	PieCtrlRegs.PIEACK.all=0xFFFF;
	PieCtrlRegs.PIEACK.all = 0xFFFF; 
}
interrupt void pdpint_ISR(void)
{
	 status=GUZHANG;
	 is_dead=1;
     vsi1_running(DISABLE);
     vsi2_running(DISABLE); 
      
	 EALLOW;
     EPwm1Regs.TZCLR.bit.CBC = 1;
   	 EPwm2Regs.TZCLR.bit.CBC = 1;
   	 EPwm3Regs.TZCLR.bit.CBC = 1;
	 EPwm4Regs.TZCLR.bit.CBC = 1;
   	 EPwm5Regs.TZCLR.bit.CBC = 1;
   	 EPwm6Regs.TZCLR.bit.CBC = 1;
 	 EPwm1Regs.TZCLR.bit.INT = 1;	
 	 EPwm2Regs.TZCLR.bit.INT = 1;	
 	 EPwm3Regs.TZCLR.bit.INT = 1;
 	 EPwm4Regs.TZCLR.bit.INT = 1;	
 	 EPwm5Regs.TZCLR.bit.INT = 1;	
 	 EPwm6Regs.TZCLR.bit.INT = 1;	 
	 PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
	 EDIS;
	 return;		
}


void init_data(void)
{
 int i;
  myfilter.a=0.001;
  myfilter.fL=60;
  myfilter.fs=3000;
  myfilter.Xn=0;
  myfilter.Yn=0;
  myfilter.Yn1=0;
 
  udc_givens=300;
  il_givens=20;
  udc_kp=1;
  udc_ki=0.0001;
  il_kp=60;
  il_ki=0.1;

   udc_pi.Errp=0;
   udc_pi.Fdb=0;
   udc_pi.Ki=udc_ki;
   udc_pi.Kp=udc_kp;
   udc_pi.Out=0;
   udc_pi.OutMax=30;
   udc_pi.OutMin=-30;
   udc_pi.Ref=udc_givens;
   udc_pi.Ui=0;
   udc_pi.Up=0;

   il_pi.Errp=0;
   il_pi.Fdb=0;
   il_pi.Ki=il_ki;
   il_pi.Kp=il_kp;
   il_pi.Out=0;
   il_pi.OutMax=300;
   il_pi.OutMin=-300;
   il_pi.Ref=il_givens;
   il_pi.Ui=0;
   il_pi.Up=0;


	adcdataindex=0;
 
	savadatacounter=0;
	readindex=0;
	writeindex=0;
	
	for(i=0;i<=SENDBUFFSIZE;i++)    
   sendbuffer[i]=0;

   for(i=0;i<RECEVICE_BUFFER_SIZE;i++)
   recevice_buffer[i]=0; 




    adcdataindex=0;
    recevice_num=0;
    counter1ms=0;
	 
	 counter500ms=0;
 



	 readindex1=0;
     writeindex1=0;
     for(i=0;i<=SENDBUFFSIZE1;i++)
     sendbuffer1[i]=0;

     temp_long=0;


	   is_dead=1; 
}

void ad_pianyi_jisuan(void)
{
    unsigned int i=0;
	for(i=0;i<65533;i++);//等待AD转化一段时间
    temp_isr2=0; 	
	for(temp_isr1=0;temp_isr1<8;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	udc_offset_int=temp_isr2>>3; 	
    if(udc_offset_int>500)while(1);

	temp_isr2=0;
	for(temp_isr1=8;temp_isr1<16;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	uab_offset_int=temp_isr2>>3;
	if(uab_offset_int>2500)while(1);
//	if(uab_offset_int>

	temp_isr2=0;
	for(temp_isr1=16;temp_isr1<24;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	ubc_offset_int=temp_isr2>>3;
	if(ubc_offset_int>2500)while(1);
	 
}



void mydelay(unsigned long int t)
{
	unsigned int i;
	while(t--)
	for(i=0;i<65533;i++);
}

void sendpack(char start,signed int send)
{
    char i; 
	if(send<0)
	{
	start+=1;
	send=-send;
	}
	tempchar=(send/1000)<<4; //1234
	tempchar+=(send/100)%10;
	tempbuffer[0]=tempchar;//0x12
	tempchar=((send/10)%10)<<4;//0x30
	tempchar+=send%10;//0x34
	tempbuffer[1]=tempchar;
	tempbuffer[2]=start;
	for(i=0;i<3;i++)
	{
		sendbuffer[writeindex++]=tempbuffer[i];writeindex&=SENDBUFFSIZE;
	}  
}


void sendpack1(char start,signed int send)
{
    char i; 
	if(send<0)
	{
	start+=1;
	send=-send;
	}
	tempchar=(send/1000)<<4; //1234
	tempchar+=(send/100)%10;
	tempbuffer[0]=tempchar;//0x12
	tempchar=((send/10)%10)<<4;//0x30
	tempchar+=send%10;//0x34
	tempbuffer[1]=tempchar;
	tempbuffer[2]=start;
	for(i=0;i<3;i++)
	{
		sendbuffer1[writeindex1++]=tempbuffer[i];writeindex1&=SENDBUFFSIZE1;
	}  
}


void check_recevice_scic_new(void)
{
	char a;
    while(ScicRegs.SCIFFRX.bit.RXFFST!=0)
	{
		a=((ScicRegs.SCIRXBUF.all) & 0xff);		
		switch(a)
		{
          case 8: break;//启动SVG
		  case 1: break;//停止SVG
		  case 2: break;//启动DVR
		  case 3: break;//停止DVR
		  case 4: //启动充电
		  if(is_dead==0)
		  {
			  udc_pi.Ui=0;
			  il_pi.Ui=0;
			  run_mode=CHONGDIAN_MOSHI;
			  status=YUNXING;
			  vsi1_running(1);	
		  }	  
		  break;

		  case 5:  //停止充电
		  if(is_dead==0)
		  {
			  status=TINGZHI;
			  run_mode=CHONGDIAN_MOSHI;
			  vsi1_running(0);	
		  }	  
		  break;

		  case 6:  //全速稳压
		  if(is_dead==0)
		  {
			  udc_pi.Ui=20;
			  il_pi.Ui=0;
			  run_mode=QUANSU_FANGDIAN_MOSHI;
			  status=YUNXING;
			  vsi1_running(1);	
		  }	  
		  break;

		  case 7:   //不稳压
		  if(is_dead==0)
		  {
			  status=TINGZHI;
			  vsi1_running(0);
		  }
		  break;

		  case 10:is_dead=0;break;
		}	
	}
}
