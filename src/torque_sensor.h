#ifndef _TORQUE_SENSOR_H_
#define _TORQUE_SENSOR_H_

int dy200_init(char *dev, int bud);
void dy200_shutdown(void);
int get_dy200_info(float *torque, float *speed, float *power);
int dy200_is_running(void);
int dy200_has_valid_data(void);

#endif
