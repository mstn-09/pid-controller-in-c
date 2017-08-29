#ifndef _LIBPID_H_
#define _LIBPID_H_

#define INPUT 1
#define OUTPUT 2
#define ERRSUM 3
#define FORWARD_EULER 4
#define BACKWARD_EULER 5
#define TRAPEZOIDAL  6


typedef struct
{
	double kp;
	double ki;
	double kd;
	double kt;
	double n;
	//double bias;
	double dt;
	double td;		/*erivative time constant*/
	double set_point;
	//double b;
	//double c;

	double max_input;
	double min_input;
	double max_output;
	double min_output;

	double ck;		/*the control signal at instant k*/
	double cka;  		/*Actuator out put*/
	double ckm1;	/*The previous control signal for kminus1 set to 0 at start*/
	double ckm2;

	double ek;		/*the error signal at instant k*/
	double ekm1;	/*the error signal at instant kminus1 set to 0 intially*/
	double ekm2;	/*the error signal at instant kminus2 set to 0 intially*/
	double feedback;	/*process_variable; //feedBack variable value*/

} pidc_t;



void set_reference(pidc_t *pidp,double reference);
void input_range(pidc_t *pidp,double minima,double maxima);
void output_range(pidc_t *pidp,double minima,double maxima);
void set_feedback(pidc_t *pidp,double value);

//double get_feedback(pidc_t *pidp);

//double filter_input(pidc_t *pidp);//
//double actuate_output(pidc_t *pidp,double *outputp);//

void initialize(pidc_t *pidp,double ckm1,double ckm2,double ekm1,double ekm2);		
double get_error(pidc_t *pidp);

double pid(pidc_t *pidp,double Kp,double Ki,double Kd,double N,double dt,char Method); 

//void forward(pidc_t *pidp); 		// forward euler approximation
//void backward(pidc_t *pidp);		// backward euler approximation
//void trapezoidal(pidc_t *pidp);		// trapezoidal
void pid_autotune(/*@unused@*/);
#endif 
