#include "ios_disp.h"
#include <Arduino.h>
int rxdata_ios[3]={};
int receiveIos(void){
  static int data_num = 0;
  int rx = Serial1.read();
  if(rx == 255){
    data_num = 0;
  }
  rxdata_ios[data_num] = rx;
  if(data_num<2){
    data_num++;
  }
  if(data_num == 2 && rx == 1){
    rxdata_ios[2] = 1;
  }
  return rxdata_ios[1];
}
void dispIos(int d){
    int x[7];
    x[0] = ' ';
    if(d<0){
      x[1] = '-';
    }else{
      x[1] = '+';
    }
    int j;
    for(j=6; j>1; j--){
      x[j] = d%10;
      x[j] += 48;
      d /= 10;
    }
    for(j=0; j<7; j++){
      txdIos(x[j]);
    }
}
void dnlIos(){
  txdIos(18);
}
void txdIos(char a){
  Serial1.write(a);
  for(int i=0; i<10000; i++){
    delay(0.01);
    if(rxdata_ios[2] == 1){
      rxdata_ios[2] = 0;
      break;
    }
  }
}

