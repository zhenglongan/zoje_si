/*******************************************************************************
*
* math library header file
*
* math.h -- Version 2.00.00
* $Date: 2005/10/24 10:23:38 $
* $Revision: 1.15 $
*
* Copyright(C) 1999(2000-2004). Renesas Technology Corp.
* and Renesas Solutions Corp., All rights reserved.
*
*
*******************************************************************************/
#if defined(NC30)		/* NCxx */
	/* dummy */
#elif defined(NC77)
	#error "NC77 not supported"
#elif defined(NC79)
	#error "NC79 not supported"
#else
	#error "NC30, NC77, NC79 not defined"
#endif				/* NCxx */

#ifdef NEED_SJMP_FOR_LIB
_asm("	.SJMP	OFF");
#endif

#ifndef __MATH_H
#define __MATH_H
/* definition of exception struct - this struct is passed to the matherr
 * routine when a floating point exception is detected
 */
/*#define HUGE_VAL 1.7976931348623157e+308
*/
extern	double __huge_val;
extern	float   __huge_valf;
#ifdef	__DOUBLE_32__
#define	HUGE_VAL	HUGE_VALF
#else
#define HUGE_VAL	(__huge_val)
#endif
#define HUGE_VALF	(__huge_valf)

double acos(double x);
double asin(double x);
double atan(double x);
double atan2(double y, double x);
double ceil(double x);
double cos(double x);
double cosh(double x);
double exp(double x);
double fabs(double x);
double floor(double x);
double fmod(double x, double y);
double frexp(double val, int _far *exp);
double ldexp(double x, int exp);
double log(double x);
double log10(double x);
double modf(double val, double *pd);
double pow(double x, double y);
double sin(double x);
double sinh(double x);
double sqrt(double x);
double tan(double x);
double tanh(double x);

float acosf(float x);
float asinf(float x);
float atanf(float x);
float atan2f(float y, float x);
float ceilf(float x);
float cosf(float x);
float coshf(float x);
float expf(float x);
float fabsf(float x);
float floorf(float x);
float fmodf(float x, float y);
float frexpf(float val, int _far *exp);
float ldexpf(float x, int exp);
float logf(float x);
float log10f(float x);
float modff(float val, float *pd);
float powf(float x, float y);
float sinf(float x);
float sinhf(float x);
float sqrtf(float x);
float tanf(float x);
float tanhf(float x);

/******************************
   NC MACRO FUNCTION
******************************/
#define	fabs(z)		((z >= (double)0) ? (z) : -(z))
#define	fabsf(z)	((z >= (float)0) ? (z) : -(z))
#define	fabsm(z)	((z >= 0) ? (z) : -(z))          // this macro function is writed by limeng	
/******************************
   NC LOCAL FUNCTION
******************************/
double _tan(double x, long *k);
float  _tanf(float x, long *k);
/******************************
   NC LOCAL DEFINITION
******************************/
#define	_LOG2	(double)0.693147180559945309417232121458
#define	_LOG10	(double)2.3025850929940459010936137929093
#define	_SQRT2	(double)1.41421356237309504880168872421
#define	_PI		(double)3.1415926535897932384626433832795   /* PI */
#define	_PIP2	(double)1.57079632679489661923132169163975  /* PI/2 */
#define	_PIP2x3	(double)4.71238898038468985769396507491925  /* PI*1.5 */
#define	_D		(double)4.454455103380768678308360248557901415300e-6
#define	_EPS	(double)0.001			/* DBL_EPSILON ** 1/5 */
#endif
/*******************************************************************************
*
* math.h -- Version 2.00.00
*
* Copyright(C) 1999(2000-2004). Renesas Technology Corp.
* and Renesas Solutions Corp., All rights reserved.
*
*
*******************************************************************************/
