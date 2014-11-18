#ifndef ALGORITHM_H
#define ALGORITHM_H


#pragma CODE_SECTION(pi_calc,"ramfuncs");
#pragma CODE_SECTION(clarke_calc,"ramfuncs");
#pragma CODE_SECTION(iclarke_calc,"ramfuncs"); 
#pragma CODE_SECTION(park_calc,"ramfuncs"); 
#pragma CODE_SECTION(ipark_calc,"ramfuncs");
#pragma CODE_SECTION(filter_calc,"ramfuncs");



typedef struct PI_CTRL_TYPEDEF{  
	 			  float32  Kp;			// Parameter: Proportional gain  比例系数
				  float32  Ki;			// Parameter: Integral gain  积分强度
				  float32  Ref;   		// Input: Reference input 给定值
				  float32  Fdb;   		// Input: Feedback input 反馈值
				  float32  Errp;			// Variable: Error   误差
				  float32  Up;			// Variable: Proportional output  比例输出 
				  float32  Ui;			// Variable: Integral output  积分输出
				  float32  OutPreSat;	// Variable: Pre-saturated output 预输出
				  float32  OutMax;		// Parameter: Maximum output 输出最大值
				  float32  OutMin;		// Parameter: Minimum output 输出最小值
				  float32  Out;   		// Output: PID output PID输出
				 } PI_Ctrl;	 
void pi_calc(PI_Ctrl *v) {	 
 // Compute the error
    v->Errp = v->Ref - v->Fdb;
    
    // Compute the proportional output
    v->Up = v->Kp*v->Errp;

    // Compute the integral output
    v->Ui = (v->OutPreSat==v->Out)?(v->Ui + v->Ki*v->Errp):v->Ui;

    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui ;     
    

    if (v->OutPreSat > v->OutMax)                   
	{
	 v->Out =  v->OutMax;
	}
    else if (v->OutPreSat < v->OutMin)
	{ 
	 v->Out =  v->OutMin; 
	}
   else
	 v->Out = v->OutPreSat;   	  
} 


typedef struct CLARKE_TYPEDEF
				 {  float32  As;  	// Input: phase-a stator variable  
				  float32  Bs;		// Input: phase-b stator variable  
				  float32  Alpha;	// Output: stationary d-axis stator variable 
				  float32  Beta;	// Output: stationary q-axis stator variable 
				 } CLARKE;
void clarke_calc(CLARKE *v){ 
   v->Alpha = v->As;
   v->Beta = (v->As + 2*v->Bs)*0.57735026918963;  // 1/sqrt(3) = 0.57735026918963  
} 


typedef struct ICLARKE_TYPEDEF
				 {  float32  Alpha;  	// 
				  float32  Beta;	//   
				  float32  As;
				  float32  Bs;
				  float32  Cs;
				 } ICLARKE;	            
void iclarke_calc(ICLARKE *v) { 
     
	 v->As=v->Alpha;
     v->Bs=0.866*v->Beta-0.5*v->Alpha;
	 v->Cs=-0.866*v->Beta-0.5*v->Alpha;
} 


typedef struct PARK_TYPEDEF
				{  float32  Alpha;  	// Input: stationary d-axis stator variable 
				  float32  Beta;	// Input: stationary q-axis stator variable 
				  float32  cosa;	// Input: 
                  float32  sina;    //input;
				  float32  Ds;		// Output: rotating d-axis stator variable 
				  float32  Qs;		// Output: rotating q-axis stator variable 
				 } PARK;
void park_calc(PARK *v){  
      v->Ds = v->Alpha*v->cosa + v->Beta*v->sina;
     v->Qs = v->Beta*v->cosa - v->Alpha*v->sina;
     } 


typedef struct IPARK_TYPEDEF
				{  float32  Alpha;  	// 
				  float32  Beta;	//  
				  float32  cosa;	// Input: 
				  float32  sina;    //Input
				  float32  Ds;		// Input: rotating d-axis stator variable 
				  float32  Qs;		// Input: rotating q-axis stator variable  
				} IPARK;					
void ipark_calc(IPARK *v){  
  
     v->Alpha=v->Ds*v->cosa - v->Qs*v->sina;
     v->Beta=v->Qs*v->cosa+v->Ds*v->sina;    
} 

 
typedef struct SPLL_TYPEDEF
				{  float32  wn;  	// Input: phase-a stator variable  
				  float32  delta_w;		// Input: phase-b stator variable  
				  float32  we;	// Output: stationary d-axis stator variable 
				  float32  delta_t;
				  float32  cos[3];	// Output: stationary q-axis stator variable 
				  float32  sin[3];
				 } SPLL;
				 	            
void spll_calc(SPLL *v){ 
	v->we = v->wn + v->delta_w;
   	v->sin[0]=v->sin[1]+(((v->delta_t*v->we))*v->cos[1]);
	v->cos[0]=v->cos[1]-(((v->delta_t*v->we))*v->sin[1]);

	if(v->sin[0]>(0.9999))
		v->sin[0]=(0.9999);
	else if(v->sin[0]<(-0.9999))
		v->sin[0]=(-0.9999);
	
	if(v->cos[0]>(0.9999))
		v->cos[0]=(0.9999);
	else if(v->cos[0]<(-0.9999))
		v->cos[0]=(-0.9999);

	v->sin[1]=v->sin[0];
	v->cos[1]=v->cos[0];

	v->sin[2] = 0.8660254*v->sin[0] - 0.500*v->cos[0];
	v->cos[2] = 0.8660254*v->cos[0] + 0.500*v->sin[0];
} 
 

typedef struct {  float32  Xn;   		// Input: 
				  float32  Yn;   		// Output:  
				  float32  Yn1;			// 
				  float32  a;          // 
				  float32  fL;			// Parameter: 
				  float32  fs;			// Variable: 
				 } FILTER;
void filter_calc(FILTER *v)
{	
    // Compute the error
    v->a = 2*3.14*v->fL/v->fs;
    v->Yn = v->a*v->Xn + (1-v->a)*v->Yn1;
	v->Yn1 = v->Yn;  
    
}
#endif




