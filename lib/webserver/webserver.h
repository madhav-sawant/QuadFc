/**
 * @file webserver.h
 * @brief WiFi AP + HTTP server for PID tuning and telemetry
 */

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdbool.h>

void webserver_init(void);
void webserver_set_rate_targets(float roll_rate, float pitch_rate);
void webserver_set_angle_targets(float roll_angle, float pitch_angle);
void webserver_set_error(const char *msg);

#endif // WEBSERVER_H
