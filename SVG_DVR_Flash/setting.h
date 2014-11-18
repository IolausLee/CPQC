#ifndef SETTINGS_H
#define SETTINGS_H

#define PWMPeriod_5k  15000
#define PWMPeriod_10k 7500
#define PWMPeriod_15k 5000
#define PWMPeriod_20k 3750
#define PWMPeriod_40k 1875
#define PWMPeriod_50k 1500
#define PWMPeriod_100k 750
#define DeadTime_5us 750
#define DeadTime_4us 600
#define DeadTime_3us5 525
#define DeadTime_2us 300
#define DeadTime_1us 150


#ifndef CLOSE_LOOP_SVG  //闭环运行
#define CLOSE_LOOP_SVG
#endif

#ifndef CLOSE_LOOP_DVR
#define CLOSE_LOOP_DVR
#endif

#ifndef SPWM_DIRECT  //开环运行，直接发出正弦PWM
//#define SPWM_DIRECT
#endif

#ifndef DUTY50_DIRECT //开环运行，发出占空比50的PWM
 //#define DUTY50_DIRECT
#endif

#ifndef  ALL_CUTOFF
#define ALL_CUTOFF
#endif

//#define PWMPeriod PWMPeriod_5k
#define PWMPeriod PWMPeriod_10k  //设置PWM的频率
//#define PWMPeriod PWMPeriod_15k
//#define PWMPeriod PWMPeriod_20k
//#define PWMPeriod PWMPeriod_40k
//#define PWMPeriod PWMPeriod_50k
//#define PWMPeriod PWMPeriod_100k



//#define DeadTime DeadTime_5us //设置死区时间 
//#define DeadTime DeadTime_1us
#define DeadTime DeadTime_5us

#ifndef DIRECT_RATIO  //设置占空比寄存器越大，则有效的电平（管子导通的时间）越大
#define DIRECT_RATIO
#endif

#ifndef HIGH_ON //设置是否高导通
//#define HIGH_ON
#endif

//udcgivens=100 iqgiven=5.0  udc_p 0.2    udci0.01    id_p 10 id_i 0.01  iq_p 10  iq_i0.01
//udcgivens=100 iqgiven=5.0  udc_p 0.1    udci0.01    id_p 50 id_i 0.01  iq_p 50  iq_i0.01
//udcgiven=180 iqgiven=5 udc_p=0.2   udc_i=0.01 id_p=50  id_i=0.01 iq_d=50 iq_i=0.01
//udcgiven=180 iqgiven=5.0 udc_p=0.2 udci=0.01 id_p=40 id_i=0.0005 iq_p=40 iq_i=40
// udcgiven=180 iqgiven=5.0 udc_p=0.2 udci=0.01 id_p=65 id_i=0 iq_p=65 iq_i=0.0001
//各个PI调节器的初始化值
#define UDC_GIVEN  50
#define UDC_PICTRL_P  5
#define UDC_PICTRL_I  0.01
#define UDC_PICTRL_OUTMAX 5

#define ID_PICTRL_P  10
#define ID_PICTRL_I  0.01
#define ID_PICTRL_OUTMAX 300

#define IQ_GIVEN 0
#define IQ_PICTRL_P 10
#define IQ_PICTRL_I 0.01
#define IQ_PICTRL_OUTMAX 300

//DVR
#define UD1_GIVEN  30
#define UD1_PICTRL_P  3
#define UD1_PICTRL_I  0.005
#define UD1_PICTRL_OUTMAX 10

#define UQ1_GIVEN  0
#define UQ1_PICTRL_P  3
#define UQ1_PICTRL_I  0.005
#define UQ1_PICTRL_OUTMAX 10

#define ID1_PICTRL_P  10
#define ID1_PICTRL_I  0.005
#define ID1_PICTRL_OUTMAX 300

#define IQ1_PICTRL_P 10
#define IQ1_PICTRL_I 0.005
#define IQ1_PICTRL_OUTMAX 300
/*
#define IA_RATIO (0.012710925714665 )
#define IA_BASE (-25.409546644403040)
#define IB_RATIO (0.013325650519647)////jiaozheng 20130331 ceshichenggong
#define IB_BASE (-0.555584843918139)//jiaozheng 20130331  ceshichenggong
#define UDC_RATIO (0.2449806316)
#define UDC_BASE  (-28.89094902)
#define UAB_RATIO (0.2180919797)
#define UAB_BASE  (-422.87673765)
#define UBC_RATIO (0.091471124366583)   ////jiaozheng 20130331  ceshichenggong
#define UBC_BASE (-4.144032869735256)  ///jiaozheng 20130331  ceshichenggong
*/

#define IA_RATIO (0.012710925714665 )
#define IA_BASE (-25.409546644403040)
#define IB_RATIO (0.013458892754927)
#define IB_BASE (-26.637451362238416)

#define UDC_RATIO (0.2449806316)
#define UDC_BASE  (-40.89094902)
#define UAB_RATIO (0.2180919797)
#define UAB_BASE  (-422.87673765)
#define UBC_RATIO (0.2268494117)
#define UBC_BASE (-452.8360009)

//jiaozheng 20130331
#define IA1_RATIO (0.013574653741384)
#define IA1_BASE (-26.943628372339269)

#define IB1_RATIO (0.013948323579879)
#define IB1_BASE (-27.918363671158247)

#define UAB1_RATIO (0.4521071049384)
#define UAB1_BASE  (-906.6370194683280)
#define UBC1_RATIO (0.4522872612461)
#define UBC1_BASE (-908.9774422742869)

#ifndef ENABLE
#define ENABLE 1
#endif

#ifndef DISABLE
#define DISABLE 0
#endif


#endif


