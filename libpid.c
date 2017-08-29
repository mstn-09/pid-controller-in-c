/*
 *         By : Yared Tadesse
 *
 *         12:56 PM Sunday,Aug 27,2017 GC
 *	   Misht 6:00, Kedamit sembet,Nehase 20,2009 EC
 *         Addis Ababa ,Ethiopia
 * Reference:
 * 1.	PID controller-Wikipedia, the free encyclopedia https://en.wikipedia.org/wik
 * 	i/PID_controller ..07/26/2016 02:06PM
 * 2.	Karl Johan Astrom and Richard M. Murray ,"Feedback systems : an introduction
 * 	for scientists and engineers ",2009,Princeton University Press,Version v2.10
 * 3.	I.Kar,"Digital Control,Module 1:Introduction to Digital Control Lecture Note
 * 4.	Xin-lan Li,Jong-Gyu Park,& Hwi-Beom Shin,"Comparison and Evaluation of
 * 	Anti-Windup PI Controllers",Journal of Power Electronics, Vol. 11, No. 1,
 * 	January 2011
 */

#include <math.h>
#include <float.h>
#include "libpid.h"

/* User access limited functions*/
static double get_feedback(pidc_t *pidp);
static double filter_input(pidc_t *pidp);//
static double actuate_output(pidc_t *pidp,double *outputp);//
static void forward(pidc_t *pidp); 		// forward Euler approximation
static void backward(pidc_t *pidp);		// backward Euler approximation
static void trapezoidal(pidc_t *pidp);		// trapezoidal


/*PID Controller implementation Codes*/
void set_reference(pidc_t *pidp,double reference){

	pidp->set_point=reference;
}//end of set point

void input_range(pidc_t *pidp,double minima,double maxima){

	pidp->min_input=minima;
	pidp->max_input=maxima;
}//end of input_range

void output_range(pidc_t *pidp,double minima,double maxima){

	pidp->min_output=minima;
	pidp->max_output=maxima;
}//end of output_range


static double filter_input(pidc_t *pidp){
	double err=0;

	if(pidp->set_point > pidp->max_input){

		err=(pidp->max_input - pidp->set_point);
		pidp->set_point=pidp->max_input;

	}else if (pidp->set_point < pidp->min_input){

		err=(pidp->min_input - pidp->set_point);
		pidp->set_point=pidp->min_input;

	}else err=0;
	return err;
}//end of filter_input

static double actuate_output(pidc_t *pidp,double *outputp){
	double err;

	if((*outputp) > pidp->max_output){

		err=  (pidp->max_output - (*outputp)); /* return Negative error */

	}else if ((*outputp) < pidp->min_output){

		err=(pidp->min_output - (*outputp)); /* return Positive error */
	}else{
		err=0;
	}

	return err;
}//end of output Actuator


void set_feedback(pidc_t *pidp,double value){

	pidp->feedback=value;
}//end of set_feedback

static double get_feedback(pidc_t *pidp){

	return pidp->feedback;
}//end of get_feedback

void initialize(pidc_t *pidp,double _ckm1,double _ckm2,double _ekm1,double _ekm2){

	pidp->ckm1=_ckm1;
	pidp->ckm2=_ckm2;

	pidp->ekm1=_ekm1;
	pidp->ekm2=_ekm2;
}

double get_error(pidc_t *pidp){

	return pidp->ek;
}//end of get_error


double pid(pidc_t *pidp,double kp,double ki,double kd,double n,double dt,char Method){

	pidp->kp=kp;
	pidp->ki=ki;
	pidp->kd=kd;
	pidp->dt=dt;
	pidp->n=n;

	if(fabs(pidp->kd) < DBL_EPSILON){

		pidp->td=pidp->kd/pidp->kp;

		pidp->kt=1/sqrt(fabs(pidp->kd/pidp->ki));
	}else{

		pidp->kt=1/sqrt(fabs(pidp->kp/pidp->ki));
	}

	/* setting input with in range*/
	(void)filter_input(pidp);

	pidp->ek=pidp->set_point - get_feedback(pidp);
	/*end_of error calculation */

	switch(Method){

		case(FORWARD_EULER):
			forward(pidp);
			break;
		case(BACKWARD_EULER):
			backward(pidp);
			break;
		case(TRAPEZOIDAL):
			trapezoidal(pidp);
			break;
	};

	/*Update_Controller_Variables*/
	pidp->ckm2=pidp->ckm1;
	pidp->ckm1=pidp->ck;
	pidp->ekm2=pidp->ekm1;
	pidp->ekm1=pidp->ek;
	/*End of Controller_Variables update*/

	return pidp->cka;
}//end of pid

static void forward(pidc_t *pidp){

	double pterm,iterm,dterm,sterm,nts_minus1,nts_minus2;

	nts_minus1=pidp->n*pidp->dt-1;
	nts_minus2=pidp->n*pidp->dt-2;

	pterm=pidp->kp*(pidp->ek + ((nts_minus2*pidp->ekm1)-(nts_minus1*(pidp->ekm2))));

	iterm =pidp->ki*(pidp->dt)*(pidp->ekm1-(nts_minus1*pidp->ekm2));

	sterm =pidp->kt*(pidp->dt)*(actuate_output(pidp,&pidp->ckm1)-(nts_minus1*actuate_output(pidp,&pidp->ckm2)));

	dterm=pidp->kd*(pidp->n)*(pidp->ek + pidp->ekm2 - (2*pidp->ekm1));

	pidp->ck=pterm + iterm + sterm + dterm + (nts_minus1*pidp->ckm2)-(nts_minus2*pidp->ckm1);

	/*Update_Controller_Variables*/
	pidp->ckm2=pidp->ckm1;
	pidp->ckm1=pidp->ck;
	pidp->ekm2=pidp->ekm1;
	pidp->ekm1=pidp->ek;
	pidp->cka = pidp->ck + actuate_output(pidp,&pidp->ck);
} //End of forward Euler


static void backward(pidc_t *pidp){

	double pterm,iterm,sterm,dterm,nt_plus1,nts_plus2;

	nt_plus1=pidp->n*pidp->dt+1;
	nts_plus2=pidp->n*pidp->dt+2;
	/*
	 * future work for complete implementation pid controller
	 * source signal weighting
	 * double bm1r=(pidp->b - 1)*pidp->set_point;
	 * double cm1r=(pidp->c - 1)*pidp->set_point;
	 */
	pterm = pidp->kp * ((nt_plus1*pidp->ek) + pidp->ekm2 - (nts_plus2*pidp->ekm1));

	iterm = pidp->ki*pidp->dt * ((nt_plus1*pidp->ek) + pidp->ekm1);

	sterm = pidp->kt*pidp->dt * ((nt_plus1*actuate_output(pidp,&pidp->ck)) + actuate_output(pidp,&pidp->ckm1));

	dterm = pidp->kd * pidp->n * (pidp->ek - (2*pidp->ekm1) + pidp->ekm2);

	pidp->ck = (pterm + iterm + sterm + dterm + (nts_plus2 * pidp->ckm1) - pidp->ckm2 )/nt_plus1;

	/*updating variable values of the controller*/
	pidp->ckm2=pidp->ckm1;
	pidp->ckm1=pidp->ck;
	pidp->ekm2=pidp->ekm1;
	pidp->ekm1=pidp->ek;
	pidp->cka = pidp->ck + actuate_output(pidp,&pidp->ck);
} //End of backward


static void trapezoidal(pidc_t *pidp){

	double pterm,iterm,sterm,dterm,ntsm2o2,ntsp2o2,nt;
	ntsm2o2=((pidp->n*pidp->dt)-2)/2;
	ntsp2o2=((pidp->n*pidp->dt)+2)/2;
	nt=pidp->n * pidp->dt;

	pterm = ((ntsp2o2*pidp->ek)+(2*pidp->ekm1)-(ntsm2o2*pidp->ekm2))*pidp->kp;

	iterm = ((ntsp2o2*pidp->ek)+(nt*pidp->ekm1)+(ntsm2o2*pidp->ekm2))*pidp->ki*pidp->dt/2;

	sterm=((ntsp2o2*actuate_output(pidp,&pidp->ck))+(nt*actuate_output(pidp,&pidp->ckm1))+(ntsm2o2*actuate_output(pidp,&pidp->ckm2)))*pidp->kt*pidp->dt/2;

	dterm = (pidp->ek - (2*pidp->ekm1) + pidp->ekm2)*pidp->kd*pidp->n;

	pidp->ck=(pterm + iterm + sterm +dterm)/ntsp2o2;

	//updating variable values of the controller
	pidp->ckm2=pidp->ckm1;
	pidp->ckm1=pidp->ck;
	pidp->ekm2=pidp->ekm1;
	pidp->ekm1=pidp->ek;
	pidp->cka = pidp->ck + actuate_output(pidp,&pidp->ck);
}//End of trapezoidal


void pid_autotune(/*@unused@*/){
	/**/

}
