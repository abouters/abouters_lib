#ifndef _IOS_DISP_H_
#define _IOS_DISP_H_
int receive_ios(void);
void ios_disp(int d);
void txd(char d);
void dnl(void);
extern int rxdata_ios[3];
#endif
