#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "libpid.h"

#define NUM_PNTS 150
#define NUM_CMND 2

pidc_t pid1,pid2;

int main(){

	double yk,ykminus1,ck;
	int k=0;
	ykminus1=0;

	//plant  sy(s)=u(s)-y(s)
	// y[n+1]=u[n]-y[n]
	//u[n]=1 step input of the system

	char *cmd_gnuplot[]={"set title \"Input r(k)= 1.5 u(k),Plant = 1/(s+1), response\"","plot'data1.temp' with points pointtype 1"};
	FILE * temp = fopen("data1.temp","w");
	FILE * gnuplotPipe = popen("gnuplot -persistent","w");

	set_reference(&pid1,1.5);
	input_range(&pid1,-3,3);
	output_range(&pid1,-4,4);
	initialize(&pid1,0.0,0.0,0.0,0.0);

	for(k=0;k<NUM_PNTS;k++){
		//set_reference(&pid1,2*sin(50*k));
		if(k==0) set_feedback(&pid1,ykminus1);
		else set_feedback(&pid1,yk);

		//ck=pid(&pid1,1.7368,11.2053,-0.11128,8.779,0.01,FORWARD_EULER);
		ck=pid(&pid1,1.6519,3.14,-0.11721,9.2479,0.01,BACKWARD_EULER);
		//ck=pid(&pid1,0.28207,56.4148,0.063907,10.2988,0.01,TRAPEZOIDAL);

		yk=ck+(0.9512*ykminus1);
		ykminus1=yk;//update the variable

		fprintf(temp,"%lf %lf \n",k*0.01,yk);
	}

	int i=0;
	for (i=0; i < NUM_CMND; i++){
		/*Send commands to gnuplot one by one.*/
		fprintf(gnuplotPipe, "%s \n", cmd_gnuplot[i]); 
	}

	//plant two data to plot
	yk=0;
	ykminus1=0;
	ck=0;
	k=0;

	char *cmd_gnuplot2[]={"set title \"Input r(k)=2sin(50k + 1.707),Plant = 1/(s+1), response\"","plot'data2.temp' with points pointtype 1"};
	FILE * temp2 = fopen("data2.temp","w");
	FILE * gnuplotPipe2 = popen("gnuplot -persistent","w");

	//set_reference(&pid2,1.5);
	input_range(&pid2,-3,3);
	output_range(&pid2,-4,4);
	initialize(&pid2,0.0,0.0,0.0,0.0);

	for(k=0;k<NUM_PNTS;k++){
		set_reference(&pid2,2*sin((50*k) + 1.707));
		if(k==0) set_feedback(&pid2,ykminus1);
		else set_feedback(&pid2,yk);

		//ck=pid(&pid2,1.7368,11.2053,-0.11128,8.779,0.01,FORWARD_EULER);
		ck=pid(&pid2,1.6519,3.14,-0.11721,9.2479,0.01,BACKWARD_EULER);
		//ck=pid(&pid2,0.28207,56.4148,0.063907,10.2988,0.01,TRAPEZOIDAL);

		yk=ck+(0.9512*ykminus1);
		ykminus1=yk;//update the variable

		fprintf(temp2,"%lf %lf \n",k*0.01,yk);
	}

	int j=0;
	for (j=0; j < NUM_CMND; j++){
		/*Send commands to gnu-plot one by one.*/
		fprintf(gnuplotPipe2, "%s \n", cmd_gnuplot2[j]); 
	}
	return 0;
}


