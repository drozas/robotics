extern float mouse_x, mouse_y;
extern int mouse_new;
extern float joystick_x, joystick_y;
extern float pt_joystick_x, pt_joystick_y;

extern unsigned long display_state;
#define DISPLAY_ROBOT 0x01UL
#define DISPLAY_SONARS 0x02UL
#define DISPLAY_LASER 0x04UL
#define DISPLAY_GREYIMAGEA 0x08UL
#define DISPLAY_COLORIMAGEA 0x10UL
#define DISPLAY_GREYIMAGEB 0x20UL
#define DISPLAY_COLORIMAGEB 0x40UL

extern int guixforms_cycle;
extern void guixforms_shutdown();
extern void guixforms_startup();
extern void guixforms_resume();
extern void guixforms_suspend();


