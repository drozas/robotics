/** Header file generated with fdesign on Mon Jan 17 18:35:29 2005.**/

#ifndef FD_guixformsf_h_
#define FD_guixformsf_h_

/** Callbacks, globals and object handlers **/


/**** Forms and Objects ****/
typedef struct {
	FL_FORM *guixformsf;
	void *vdata;
	char *cdata;
	long  ldata;
	FL_OBJECT *exit;
	FL_OBJECT *escala;
	FL_OBJECT *center;
	FL_OBJECT *track_robot;
	FL_OBJECT *sonars;
	FL_OBJECT *robot;
	FL_OBJECT *joystick;
	FL_OBJECT *teleoperator;
	FL_OBJECT *ventana0;
	FL_OBJECT *micanvas;
	FL_OBJECT *visgreyimageA;
	FL_OBJECT *frame_rateA;
	FL_OBJECT *myschema;
	FL_OBJECT *back;
	FL_OBJECT *stop;
	FL_OBJECT *laser;
	FL_OBJECT *colorimageA;
	FL_OBJECT *pantilt_joystick;
	FL_OBJECT *pantilt_origin;
	FL_OBJECT *pantilt_enable;
	FL_OBJECT *pantilt_stop;
	FL_OBJECT *ventana1;
	FL_OBJECT *frame_rateB;
	FL_OBJECT *visgreyimageB;
	FL_OBJECT *colorimageB;
	FL_OBJECT *ptspeed;
	FL_OBJECT *pantilt_encoders;
	FL_OBJECT *guifps;
	FL_OBJECT *fpslaser;
	FL_OBJECT *fpssonars;
	FL_OBJECT *fpsencoders;
	FL_OBJECT *fpspantiltencoders;
	FL_OBJECT *motors;
	FL_OBJECT *pantiltmotors;
	FL_OBJECT *fpsmotors;
	FL_OBJECT *fpspantiltmotors;
	FL_OBJECT *vislaser;
	FL_OBJECT *visrobot;
	FL_OBJECT *vissonars;
	FL_OBJECT *vispantiltencoders;
	FL_OBJECT *viscolorimageA;
	FL_OBJECT *viscolorimageB;
} FD_guixformsf;

extern FD_guixformsf * create_form_guixformsf(void);

#endif /* FD_guixformsf_h_ */
