extern void myschema_startup();
extern void myschema_suspend();
extern void myschema_resume(int *brothers, arbitration fn);

extern int myschema_cycle; /* ms */
