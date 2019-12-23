#include "smooth.h"
#include "abouters.h"
#include <bits/stdc++.h>
int smooth(int *n,int target,int degree){
    if(*n > target){
        *n = min(*n + degree,target);
    }
    if(*n < target){
        *n = max(*n - degree,target);
    }
    return *n;
}
int smoothRising(int *n,int target,int degree_normal,int degree_slow,int threshold){
    if(abs(*n) < threshold)smooth(n,threshold,degree_slow);
    else{
        int t=*n;
        int sign = t/abs(t);
        smooth(&t,target,degree_normal);
        if(isBetween(sign*threshold,*n,t)){
            *n = sign*threshold;
            return 0;
        }
        *n = t;
        return *n == target;

    }
    return 0;
}
