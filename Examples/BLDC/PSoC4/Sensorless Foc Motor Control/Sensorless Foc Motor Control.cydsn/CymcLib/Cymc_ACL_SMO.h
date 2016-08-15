/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#ifndef __CYMC_ACL_SMO_H__
#define __CYMC_ACL_SMO_H__
#include <cytypes.h>	
#include "Cymc_GFL_Math.h"
	
typedef struct 	{ q15_t  Vbus;		/* DC-bus voltage */
				  q15_t  volAlpha;
				  q15_t  volBeta;
	              q15_t  speedRef;	
	              q15_t  Ggain;    	/* control gain               */
                  q15_t  Fgain;    	/* Filter gain				  */ 
                  q15_t  slideCoff;   /* cofficient K for slide control */
                  q15_t  slideError;    /* error threshold for slide control */  
                  q15_t  curAlpha;  /* α-axis stator current      */
                  q15_t  curBeta;  	/* β-axis stator current   	  */
                  q15_t  Theta;     /* Output:  angle 			  */
		  	  	} SMO;	

void Cymc_ACL_SMOCal(SMO *smo);
void Cymc_ACL_SMOParamtersInit(float Rs, float Ls, float Ts, float baseFrequency, SMO *smo);

#endif	
/* [] END OF FILE */
