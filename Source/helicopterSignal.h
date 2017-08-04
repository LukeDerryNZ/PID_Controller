/*
 * helicopterSignal.h
 *
 *  Created on: May 17, 2017
 *      Author: fbf10
 */

#ifndef HELICOPTERSIGNAL_H_
#define HELICOPTERSIGNAL_H_

void PinChangeIntHandler (void);
void InitPin (void);
float fCalculateDegrees (uint32_t yaw);
int32_t fCalculateFromDegrees (float degree);
uint32_t ui32SampleAltitude (void);
void SampleStoreAltitude (void);
int32_t i32CalcAltPercent (void);
void calcAltLimits(void);



#endif /* HELICOPTERSIGNAL_H_ */
