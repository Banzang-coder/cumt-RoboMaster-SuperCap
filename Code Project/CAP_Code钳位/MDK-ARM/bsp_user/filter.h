#ifndef _FILTER_
#define _FILTER_

#include "function.h"
#include "power_calc.h"
uint16_t getValue(void);

#define low_filter_default {0, 0, 0, 0, 0, 0}

typedef struct
{
	float  Input;
	float  Output;
	float  Fc;
	float  Fs;
	float  Ka;
	float  Kb;
	
}LOWPASS_FILTER_STRUCT;

#define PI2  2 * PI
#define N 10 // ÂË²¨Æ÷´°¿Ú´óÐ¡

extern LOWPASS_FILTER_STRUCT lpf[5];

extern void bubble_sort( uint32_t *a ,unsigned char Num);
extern uint32_t middle_filter(uint32_t *a ,unsigned char Num,char extremum);
extern float recursive_filter(float a);
extern void low_filter_calc(LOWPASS_FILTER_STRUCT *p);
extern void low_filter_init(LOWPASS_FILTER_STRUCT *p,float sample_f,float cutoff_f);

extern int compare(const void *a, const void *b);
//extern void update_filter(uint16_t value); 
//extern uint16_t get_filtered_value(void) ;
extern uint32_t median_filter(uint32_t value,uint32_t values_t[N], uint32_t sorted_values_t[N]);


#endif /*_FILTER_*/
