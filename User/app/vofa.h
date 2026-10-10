#ifndef USER_APP_VOFA_H
#define USER_APP_VOFA_H

#include <stdint.h>

void vofa_init(void);
void vofa_process(void);
void vofa_usb_host_request(void);
void vofa_usb_disconnected(void);

#endif /* USER_APP_VOFA_H */
