#ifndef STEPPERMOTORCONTROLLER_H
#define	STEPPERMOTORCONTROLLER_H

void SMC_StepperInit(void);
void StabilityMonitorTask(void *param);
void SMC_StepperControl(void *param);

#endif
