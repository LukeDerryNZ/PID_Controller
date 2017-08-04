/*
 *  pwmGen.h
 *
 *  Created on: 24.3.2017
 *  Author: fbf10
 *  Last Modified: 28.3.2017
 */

#ifndef PWMGEN_H_
#define PWMGEN_H_

void initClocks (void);
void initialisePWM (void);
void setMainPWM (uint32_t u32Freq, uint32_t u32Duty);
void setTailPWM (uint32_t u32Freq, uint32_t u32Duty);
void SetMainOn(bool OnOff);
void SetTailOn(bool OnOff);


#endif /* PWMGEN_H_ */
