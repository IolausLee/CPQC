#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "alogrithm.h"
#include "settings.h"
#include "device_init.h"
#include "debug_def.h"
#include "math.h"
//0 未知   1 停止    2运行    3故障    4没上电
#define WEIZHI 0
#define TINGZHI 1
#define YUNXING 2
#define GUZHANG 3
#define MEISHANGDIAN 4

void putchar(char a);


#pragma DATA_SECTION(DMABuf1,"DMARAML4");
#pragma CODE_SECTION(epwmint_ISR, "ramfuncs");
#pragma CODE_SECTION(pdpint_ISR, "ramfuncs");
#pragma CODE_SECTION(sendpack, "ramfuncs");
#pragma CODE_SECTION(loopforever, "ramfuncs");
#pragma CODE_SECTION(sendpack1,"ramfuncs");
#pragma CODE_SECTION(check_recevice_scic_new,"ramfuncs");
#pragma CODE_SECTION(start_svg,"ramfuncs"); 
#pragma CODE_SECTION(stop_svg,"ramfuncs");
#pragma CODE_SECTION(start_dvr,"ramfuncs");
#pragma CODE_SECTION(stop_dvr,"ramfuncs"); 


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
void start_svg();
void stop_svg();
void start_dvr();
void stop_dvr();
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
float tapwm,tbpwm,tcpwm;
int counter1ms=0;
char tempchar;
char tempbuffer[3];

int jinxiandianya;//进线电压
int chuxiandianya;//
int wugongdianliu;

char shangdiancishu;
 

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





int counter500ms=0;


//临时变量
unsigned int temp_isr1,temp_isr2;
unsigned int temp_main;
float temp_isr_float,temp_isr1_float,temp_isr2_float,temp_isr3_float;

float udc_givens;	 
float iqgiven;
float udc_p;
float udc_i;
float id_p;
float id_i;
float iq_p;
float iq_i;

float id1_p;
float id1_i;
float iq1_p;
float iq1_i;
float ud1_p;
float ud1_i;
float uq1_p;
float uq1_i; 


float ubc_last,ubc_this;
int T;
float sintheta_timer;
float costheta_timer;


//控制算法模块
PI_Ctrl udc_pi,iq_pi,id_pi,ud1_pi,uq1_pi,id1_pi,iq1_pi,spll_pi;
CLARKE volt_clarke,curt_clarke,dvr_curt_clarke,dvr_volt_clarke;
PARK volt_park,curt_park,dvr_volt_park,dvr_curt_park;
IPARK curt_ipark,dvr_curt_ipark;
ICLARKE curt_iclarke,dvr_curt_iclarke;
 
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

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

void   InitSysCtrl();//Disable WDG	;Set SYSCLKOUT=150Mhz;	init periphclock
void   InitXintf16Gpio(); 
void   init_gpio(); 
void   init_scib();
void   init_scic();
void   InitFlash(); 
void   init_adc();    
void   init_DMA();
void   init_epwm(); 

void main(void)
{

  // unsigned int counter;

   DINT;    
   InitSysCtrl();//Disable WDG	;Set SYSCLKOUT=150Mhz;	init periphclock
   InitXintf16Gpio(); 
   init_gpio();//already lock all pwm pins 
   init_scib();
   init_scic();	
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);     
   InitFlash();  
   init_epwm(); putchar('3'); 
   InitPieCtrl();//Enable PIEIER.ePWM1_INT and PIEIER.ePWM1.TZINT   
   IER=0; putchar('4');
   IFR=0;
   InitPieVectTable();  putchar('5'); 
   EALLOW;  
   PieVectTable.EPWM1_TZINT=&pdpint_ISR;
   PieVectTable.EPWM1_INT=&epwmint_ISR;
   EDIS;
   IER|=(M_INT3|M_INT2);
   IFR = 0x0000;  putchar('6');   
   init_adc();     
   init_DMA();   
   init_data();  
   status=TINGZHI*1000+TINGZHI*100;	 
   ad_pianyi_jisuan(); putchar('9');
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

   EALLOW;
   SysCtrlRegs.WDCR = 0x0028;   
   EDIS;  
   

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
    

	#ifdef SPWM_DIRECT	
	static unsigned int index=0;
	float temp;
	temp=2*3.14159*index*50.0/10000.0;
 	EPwm1Regs.CMPA.half.CMPA=(PWMPeriod>>1)*(sin(temp)+1);
	EPwm2Regs.CMPA.half.CMPA=(PWMPeriod>>1)*(sin(temp+2*3.14159/3)+1);
	EPwm3Regs.CMPA.half.CMPA=(PWMPeriod>>1)*(sin(temp-2*3.14159/3)+1);
	EPwm4Regs.CMPA.half.CMPA=(PWMPeriod>>1)*(sin(temp)+1);
	EPwm5Regs.CMPA.half.CMPA=(PWMPeriod>>1)*(sin(temp+2*3.14159/3)+1);
	EPwm6Regs.CMPA.half.CMPA=(PWMPeriod>>1)*(sin(temp-2*3.14159/3)+1);
 	if(++index==200)index=0;
	#endif
     

	#ifdef DUTY50_DIRECT
 	EPwm1Regs.CMPA.half.CMPA = PWMPeriod/5; // adjust duty for output EPWM1A
 	EPwm2Regs.CMPA.half.CMPA =PWMPeriod/5; // adjust duty for output EPWM2A
 	EPwm3Regs.CMPA.half.CMPA =  PWMPeriod*4/5; // adjust duty for output EPWM3A
	EPwm4Regs.CMPA.half.CMPA = PWMPeriod/5; // adjust duty for output EPWM1A
 	EPwm5Regs.CMPA.half.CMPA =PWMPeriod/5; // adjust duty for output EPWM2A
 	EPwm6Regs.CMPA.half.CMPA =  PWMPeriod*4/5; // adjust duty for output EPWM3A
	#endif



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

 	temp_isr2=0;
	for(temp_isr1=24;temp_isr1<32;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	ia_int=temp_isr2>>3;
	ia_int-=ia_offset_int;

	temp_isr2=0;
	for(temp_isr1=32;temp_isr1<40;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	ib_int=temp_isr2>>3; 
	ib_int-=ib_offset_int;

	temp_isr2=0; 	
	for(temp_isr1=48;temp_isr1<56;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	uab1_int=temp_isr2>>3; 	
	uab1_int-=uab1_offset_int;

	temp_isr2=0;
	for(temp_isr1=56;temp_isr1<64;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	ubc1_int=temp_isr2>>3;
	ubc1_int-=ubc1_offset_int;

 	temp_isr2=0;
	for(temp_isr1=64;temp_isr1<72;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	ia1_int=temp_isr2>>3;
	ia1_int-=ia1_offset_int;

	temp_isr2=0;
	for(temp_isr1=72;temp_isr1<80;temp_isr1++)
	{
		  temp_isr2+=DMABuf1[temp_isr1];
	}
	ib1_int=temp_isr2>>3; 
	ib1_int-=ib1_offset_int;

	//电压 采样dianzu 91
	udc_fd=(float)(udc_int*0.2660);//(float)((udc_int*3.0)/4095.0*5.0/3.0)*166.6667;//udc_int*1.0*UDC_RATIO+UDC_BASE;//误差纠正
 	uab_fd=(float)(uab_int*0.4070);//(float)((uab_int*3.0)/4095.0*5.0/3.0)*333.333;//uab_int*1.0*UAB_RATIO+UAB_BASE;
	ubc_fd=(float)(ubc_int*0.4070);//(float)((ubc_int*3.0)/4095.0*5.0/3.0)*333.333;//ubc_fd=(float)((ubc_int*3.0)/4095.0*5.0/3.0-2.5)*66.6667+3;//
 	ia_fd=(float)(ia_int*0.0122);//(float)((ia_int*3.0)/4095.0*5.0/3.0)*(10);//(ia_int*1.0*IA_RATIO+IA_BASE);//按整流模式的算法
	ib_fd=(float)(ib_int*0.0122);//(float)((ib_int*3.0)/4095.0*5.0/3.0)*(10);

    uab1_fd=(float)(uab1_int*0.4070);//(float)((uab1_int*3.0)/4095.0*5.0/3.0)*333.333;//uab_int*1.0*UAB_RATIO+UAB_BASE;
	ubc1_fd=(float)(ubc1_int*0.4070);//(float)((ubc1_int*3.0)/4095.0*5.0/3.0)*333.333;//ubc_fd=(float)((ubc_int*3.0)/4095.0*5.0/3.0-2.5)*66.6667+3;//
 	ia1_fd=(float)(ia1_int*0.0122);//(float)((ia1_int*3.0)/4095.0*5.0/3.0)*(10);//(ia_int*1.0*IA_RATIO+IA_BASE);//按整流模式的算法
	ib1_fd=(float)(ib1_int*0.0122);//(float)((ib1_int*3.0)/4095.0*5.0/3.0)*(10);

	temp_isr1_float=ia_fd; ia_fd=ia1_fd; ia1_fd=temp_isr1_float;
	temp_isr1_float=ib_fd; ib_fd=ib1_fd; ib1_fd=temp_isr1_float;

	myfilter.Xn=udc_fd;
	filter_calc(&myfilter);
 	udc_fd=myfilter.Yn;
	if(udc_fd>800.0||ia_fd>30.0||ib_fd>30.0||ia_fd<-30.0||ib_fd<-30.0)
	{
		GpioDataRegs.GPBDAT.bit.GPIO32=1;//过压保护
		GpioDataRegs.GPBSET.bit.GPIO32=1;
	 	vsi1_running(0);
		putchar('g');
		status=GUZHANG*1000+GUZHANG*100; 
		sendpack1(0xac,status+diaodian_flags*10);
	 	while(1)
	 	{
	 	     if(writeindex1!=readindex1)
			 {
			  if(scic_xmit(sendbuffer1[readindex1])==0){readindex1++;readindex1&=SENDBUFFSIZE1;}
			 }
		}
	} 

	volt_clarke.As=uab_fd*0.666667+ubc_fd*0.333333;//转换成相电压输入到clarke变换模块中
	volt_clarke.Bs=(ubc_fd-uab_fd)*0.333333;
	clarke_calc(&volt_clarke);

	curt_clarke.As=ia_fd;
	curt_clarke.Bs=ib_fd;
	clarke_calc(&curt_clarke);

	curt_park.Alpha=curt_clarke.Alpha;
	curt_park.Beta=curt_clarke.Beta;

   if(++counter1ms>=40)
	{
  	  sendpack(0xAC,udc_fd);//volt_clarke.As);
   //   sendpack(0xBC,curt_park.Qs*10);
  //	  sendpack(0xBE,dvr_curt_park.Ds*10);

  //    sendpack(0xae,dvr_volt_park.Ds);
 //	 sendpack(0xba,dvr_volt_park.Qs);

  //  sendpack(0xAC,curt_park.cosa*100);
 //	 sendpack(0xAE,costheta_timer*100);

     counter1ms=0;
     if(writeindex!=readindex)
	 {
	 	 if(scib_xmit(sendbuffer[readindex])==0){readindex++;readindex&=SENDBUFFSIZE;}
	 }
	}

	/////////////////////////////使用过0检测结合软件计算定向角//////////////////////////
    ubc_this=ubc_fd;
	temp_isr3_float=uab_fd*uab_fd+ubc_fd*ubc_fd;

	diaodian_flags_last=diaodian_flags;
	if(temp_isr3_float<10000)
	{
	  diaodian_flags=1;
	}
	else
	{
	  diaodian_flags=2;
	}
	if(diaodian_flags!=diaodian_flags_last)
	{
	  temp_long=status+diaodian_flags*10;
	  sendpack1(0xac,temp_long);
	  if(diaodian_flags==2 && diaodian_flags_last==1)
	  {
	    if(shangdiancishu==0)
		shangdiancishu++;
		else
	    start_svg();
	  }else if(diaodian_flags==1 && diaodian_flags_last==2)
	  {
	    stop_svg();
	  }
	}
	
	if(++T>=20000)T=0;
	if(ubc_last<=0 && ubc_this>0 && temp_isr3_float>10000)
	{
	 T=0;
	} 
	ubc_last=ubc_this;
    temp_isr1_float=(T%200)*0.0314159;//(T%200)*6.28/200;
	sintheta_timer=sin(temp_isr1_float);
	costheta_timer=-cos(temp_isr1_float);

 	temp_isr_float=sqrt(volt_clarke.Alpha*volt_clarke.Alpha+volt_clarke.Beta*volt_clarke.Beta);	
  	temp_isr1_float=volt_clarke.Beta/temp_isr_float;
  	temp_isr2_float=volt_clarke.Alpha/temp_isr_float;

	jinxiandianya=(int)(temp_isr_float*0.707);
     
	if(temp_isr3_float<10000) //定向角掉电切换
	{
    curt_park.sina=sintheta_timer; // temp_isr1_float;//;sintheta_timer // -myspll.sin[0];
    curt_park.cosa=costheta_timer; //temp_isr2_float;//;//; -myspll.cos[0] costheta_timer
	}else
	{
	  curt_park.sina= temp_isr1_float;//;sintheta_timer // -myspll.sin[0];
      curt_park.cosa= temp_isr2_float;//;//; -myspll.cos[0] costheta_timer
	}
/*******spll replace****/
	park_calc(&curt_park);

	wugongdianliu=(int)(curt_park.Qs*0.707);
	
	udc_pi.Kp=udc_p;
	udc_pi.Ki=udc_i;
    udc_pi.Ref=udc_givens;
	udc_pi.Fdb=udc_fd;
	pi_calc(&udc_pi);

	id_pi.Kp=id_p;
	id_pi.Ki=id_i;
	id_pi.Fdb=curt_park.Ds;
	id_pi.Ref=-udc_pi.Out;	 // 0;	   //
	pi_calc(&id_pi);

	iq_pi.Kp=iq_p;
	iq_pi.Ki=iq_i;
	iq_pi.Fdb=curt_park.Qs;
	iq_pi.Ref=iqgiven;
	pi_calc(&iq_pi);   

	curt_ipark.Qs=iq_pi.Out;
	curt_ipark.Ds=id_pi.Out;
	curt_ipark.sina=curt_park.sina;
	curt_ipark.cosa=curt_park.cosa;
	ipark_calc(&curt_ipark);
  
	curt_iclarke.Alpha=curt_ipark.Alpha;
	curt_iclarke.Beta=curt_ipark.Beta;
	iclarke_calc(&curt_iclarke);

#ifdef CLOSE_LOOP_SVG	
	EPwm4Regs.CMPA.half.CMPA =  ((curt_iclarke.As/IQ_PICTRL_OUTMAX)+1)*PWMPeriod/2;
 	EPwm5Regs.CMPA.half.CMPA =  ((curt_iclarke.Bs/IQ_PICTRL_OUTMAX)+1)*PWMPeriod/2;
 	EPwm6Regs.CMPA.half.CMPA =  ((curt_iclarke.Cs/IQ_PICTRL_OUTMAX)+1)*PWMPeriod/2;
#endif

	dvr_volt_clarke.As=uab1_fd*0.666667+ubc1_fd*0.333333;//转换成相电压输入到clarke变换模块中
	dvr_volt_clarke.Bs=(ubc1_fd-uab1_fd)*0.333333;
	clarke_calc(&dvr_volt_clarke);
	dvr_volt_park.Alpha=dvr_volt_clarke.Alpha;
	dvr_volt_park.Beta=dvr_volt_clarke.Beta;
	dvr_volt_park.cosa=curt_park.cosa;
	dvr_volt_park.sina=curt_park.sina;
	park_calc(&dvr_volt_park);

	dvr_curt_clarke.As=ia1_fd;
	dvr_curt_clarke.Bs=ib1_fd;
	clarke_calc(&dvr_curt_clarke);
	dvr_curt_park.Alpha=dvr_curt_clarke.Alpha;
	dvr_curt_park.Beta=dvr_curt_clarke.Beta;
	dvr_curt_park.cosa=curt_park.cosa;
	dvr_curt_park.sina=curt_park.sina;
	park_calc(&dvr_curt_park);

	chuxiandianya=(int)(dvr_volt_park.Ds*0.707);

	ud1_pi.Ref=ud1_given;
	ud1_pi.Kp=ud1_p;
	ud1_pi.Ki=ud1_i;
	ud1_pi.Fdb=dvr_volt_park.Ds;
	pi_calc(&ud1_pi);

	uq1_pi.Fdb=dvr_volt_park.Qs;
	uq1_pi.Kp=uq1_p;
	uq1_pi.Ki=uq1_i;
	pi_calc(&uq1_pi);

	id1_pi.Ref=-1*ud1_pi.Out;
	id1_pi.Kp=id1_p;
	id1_pi.Ki=id1_i;
	id1_pi.Fdb=dvr_curt_park.Ds;
	pi_calc(&id1_pi);

	iq1_pi.Ref=-1*uq1_pi.Out;
	iq1_pi.Kp=iq1_p;
	iq1_pi.Ki=iq1_i;
	iq1_pi.Fdb=dvr_curt_park.Qs;
	pi_calc(&iq1_pi);

	dvr_curt_ipark.Ds=id1_pi.Out;
	dvr_curt_ipark.Qs=iq1_pi.Out;
	dvr_curt_ipark.sina=curt_park.sina;
	dvr_curt_ipark.cosa=curt_park.cosa;
	ipark_calc(&dvr_curt_ipark);
	
	dvr_curt_iclarke.Alpha=dvr_curt_ipark.Alpha;
	dvr_curt_iclarke.Beta=dvr_curt_ipark.Beta;
	iclarke_calc(&dvr_curt_iclarke);

	#ifdef CLOSE_LOOP_DVR	
	EPwm1Regs.CMPA.half.CMPA =  ((dvr_curt_iclarke.As/ID1_PICTRL_OUTMAX)+1)*PWMPeriod/2;
 	EPwm2Regs.CMPA.half.CMPA =  ((dvr_curt_iclarke.Bs/ID1_PICTRL_OUTMAX)+1)*PWMPeriod/2;
 	EPwm3Regs.CMPA.half.CMPA =  ((dvr_curt_iclarke.Cs/ID1_PICTRL_OUTMAX)+1)*PWMPeriod/2;
	#endif	

	if(++counter500ms>3000)
	{
	 counter500ms=0;
	 temp_long=status+diaodian_flags*10;
	 sendpack1(0xac,temp_long);
	 sendpack1(0xae,wugongdianliu);
	 sendpack1(0xba,jinxiandianya);
	 sendpack1(0xbc,chuxiandianya);
	}

	//feed dog 
     EALLOW;
     SysCtrlRegs.WDKEY = 0x0055;
     SysCtrlRegs.WDKEY = 0x00AA;
     EDIS; 

 	EPwm1Regs.ETCLR.bit.INT=1;//clear interrupt flags
	PieCtrlRegs.PIEACK.all=0xFFFF;
	PieCtrlRegs.PIEACK.all = 0xFFFF; 
}
interrupt void pdpint_ISR(void)
{
	 status=GUZHANG*1000+GUZHANG*100;
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
	counter500ms=0;
	udc_pi.Kp=UDC_PICTRL_P;
	udc_pi.Ki=UDC_PICTRL_I;
	udc_pi.OutMax=UDC_PICTRL_OUTMAX;
	udc_pi.OutMin=-1*UDC_PICTRL_OUTMAX;
	udc_pi.Ref=UDC_GIVEN;
	udc_pi.Errp=0;
	udc_pi.Fdb=0;
	udc_pi.Out=0;
	udc_pi.OutPreSat=0;
	udc_pi.Ui=0;
	udc_pi.Up=0;

	id_pi.Kp=ID_PICTRL_P;
	id_pi.Ki=ID_PICTRL_I;
	id_pi.OutMax=280;
	id_pi.OutMin=-1*280;
	id_pi.Ref=0;
	id_pi.Errp=0;
	id_pi.Fdb=0;
	id_pi.Out=0;
	id_pi.OutPreSat=0;
	id_pi.Ui=0;
	id_pi.Up=0;

	iq_pi.Kp=IQ_PICTRL_P;
	iq_pi.Ki=IQ_PICTRL_I;
	iq_pi.OutMax=100;
	iq_pi.OutMin=-1*100;
	iq_pi.Ref=IQ_GIVEN;
	iq_pi.Errp=0;
	iq_pi.Fdb=0;
	iq_pi.Out=0;
	iq_pi.OutPreSat=0;
	iq_pi.Ui=0;
	iq_pi.Up=0;
	
	ud1_pi.Errp=0;
	ud1_pi.Fdb=0;
	ud1_pi.Kp=0;
	ud1_pi.Ki=0;
	ud1_pi.Out=0;
	ud1_pi.OutMax=20;
	ud1_pi.OutMin=-20;
	ud1_pi.OutPreSat=0;
	ud1_pi.Ref=0;
	ud1_pi.Ui=0;
	ud1_pi.Up=0;

	uq1_pi.Errp=0;
	uq1_pi.Fdb=0;
	uq1_pi.Ki=0;
	uq1_pi.Kp=0;
	uq1_pi.Out=0;
	uq1_pi.OutMax=20;
	uq1_pi.OutMin=-20;
	uq1_pi.OutPreSat=0;
	uq1_pi.Ref=0;
	uq1_pi.Ui=0;
	uq1_pi.Up=0;

	id1_pi.Errp=0;
	ud1_pi.Fdb=0;
	id1_pi.Kp=0;
	id1_pi.Ki=0;
	id1_pi.Out=0;
	id1_pi.OutMax=289;
	id1_pi.OutMin=-289;
	id1_pi.OutPreSat=0;
	id1_pi.Ref=0;
	id1_pi.Ui=0;
	id1_pi.Up=0;

	iq1_pi.Errp=0;
	iq1_pi.Fdb=0;
	iq1_pi.Ki=0;
	iq1_pi.Kp=0;
	iq1_pi.Out=0;
	iq1_pi.OutMax=75;
	iq1_pi.OutMin=-75;
	iq1_pi.OutPreSat=0;
	iq1_pi.Ref=0;
	iq1_pi.Ui=0;
	iq1_pi.Up=0;


	shangdiancishu=0;


 

  udc_givens=400;	 
  iqgiven=15;
  udc_p=1;
  udc_i=0.01;
  id_p=10;
  id_i=0.1;
  iq_p=10;
  iq_i=0.1;


  ud1_given=311; 
  ud1_p=0.1;
  ud1_i=0.01;
  uq1_p=0.1;
  uq1_i=0.01;

  id1_p=0.5;
  id1_i=0.01;
  iq1_p=0.5;
  iq1_i=0.01;

  myfilter.a=0.001;
  myfilter.fL=60;
  myfilter.fs=3000;
  myfilter.Xn=0;
  myfilter.Yn=0;
  myfilter.Yn1=0;
 

 ubc_this=0;
 ubc_last=0;


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
	 jinxiandianya=0;//进线电压
	 chuxiandianya=0;//
	 wugongdianliu=0; 
	 diaodian_flags=1;
	 diaodian_flags_last=1;
	 counter500ms=0;
	 ubc_last=0;
	 ubc_this=0;
	 sintheta_timer=0;
	 costheta_timer=1;



	 readindex1=0;
     writeindex1=0;
     for(i=0;i<=SENDBUFFSIZE1;i++)
     sendbuffer1[i]=0;

     temp_long=0;
}

void ad_pianyi_jisuan(void)
{
    unsigned int i=0;
	for(i=0;i<65533;i++);//等待AD转化一段时间
	for(i=0;i<65533;i++);//等待AD转化一段时间
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

 	temp_isr2=0;
	for(temp_isr1=24;temp_isr1<32;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	ia_offset_int=temp_isr2>>3;
	if(ia_offset_int>2500)while(1); 

	temp_isr2=0;
	for(temp_isr1=32;temp_isr1<40;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	ib_offset_int=temp_isr2>>3; 
	if(ib_offset_int>2500)while(1);

	temp_isr2=0; 	
	for(temp_isr1=48;temp_isr1<56;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	uab1_offset_int=temp_isr2>>3; 
	if(uab1_offset_int>2500)while(1);	

	temp_isr2=0;
	for(temp_isr1=56;temp_isr1<64;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	ubc1_offset_int=temp_isr2>>3;
	if(ubc1_offset_int>2500)while(1);


 	temp_isr2=0;
	for(temp_isr1=64;temp_isr1<72;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	ia1_offset_int=temp_isr2>>3;
    if(ia1_offset_int>2500)while(1);

	temp_isr2=0;
	for(temp_isr1=72;temp_isr1<80;temp_isr1++)
	{
		  temp_isr2+=(DMABuf1[temp_isr1]);
	}
	ib1_offset_int=temp_isr2>>3; 
	if(ib1_offset_int>2500)while(1);	 
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
          case 8:start_svg();break;//启动SVG
		  case 1:stop_svg();break;//停止SVG
		  case 2:start_dvr();break;//启动DVR
		  case 3:stop_dvr();break;//停止DVR
		  case 4:break;//启动充电
		  case 5:break;//停止充电
		  case 6: //全速稳压
		  break;
		  case 7:
		  break;//不稳压 
		  case 11:iqgiven=0; 
		  break;

		  case 12:iqgiven=15; 
		  break;

		  case 13:iqgiven=-15; 
		  break;
		}	
	}
}

void start_svg()
{
  udc_pi.Ui=0;
  udc_pi.Out=0;
  id_pi.Ui=0;
  id_pi.Out=0;
  iq_pi.Out=0;
  iq_pi.Ui=0;
  status=(status%1000+YUNXING*1000);
  vsi2_running(1); //svg 
}
void stop_svg()
{
  status=(status%1000+TINGZHI*1000);
  vsi2_running(0); //svg
}

void start_dvr()
{
 
  ud1_pi.Ui=0;
  ud1_pi.Out=0;
  uq1_pi.Ui=0;
  uq1_pi.Out=0;
  id1_pi.Ui=0;
  id1_pi.Out=0;
  iq1_pi.Ui=0;
  iq1_pi.Out=0;
  status=(status/1000)*1000+(status%100)+YUNXING*100;
  vsi1_running(1); //dvr 
}
void stop_dvr()
{
  status=(status/1000)*1000+(status%100)+TINGZHI*100;
  vsi1_running(0); //dvr
}
