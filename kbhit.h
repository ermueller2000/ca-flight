// Retrieved from http://linux-sxs.org/programming/kbhit.html on 11/25/14

#ifndef KBHITh
#define KBHITh

void   init_keyboard(void);
void   close_keyboard(void);
int      kbhit(void);
int     readch(void); 

#endif 