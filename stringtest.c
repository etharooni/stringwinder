#include <stdio.h>
#include <math.h>

#define lambda10 0.00154
#define lambda20 0.00175
#define maxDensity 0.0127
#define b 0.25
#define l 1.0
#define r 0.0005

#define angularv 41.89

#define rotpermeter  200
#define stepsperrot 200

int main(){
	for(int i=0; i<(int)(l*1000.0); i++){
		double x = (double)i/1000.0;
		
		double dtheta_dx=(((maxDensity*(1-b))/(1+b*cos((2*M_PI*x)/l)))-lambda10)*(1/lambda20);
		dtheta_dx = dtheta_dx*dtheta_dx-1;
		dtheta_dx = sqrt(dtheta_dx)/r;
		double metersPerSecond = angularv/dtheta_dx;
		double rpm_carriage = metersPerSecond*rotpermeter*60;
		printf("%f,%f,%f\n", dtheta_dx, metersPerSecond, rpm_carriage);
	}
}