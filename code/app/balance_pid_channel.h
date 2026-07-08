/*
 * balance_pid_channel.h
 *
 *  Shared balance PID channel identifiers
 *  Copyright: KimJong
 */

#ifndef CODE_APP_BALANCE_PID_CHANNEL_H_
#define CODE_APP_BALANCE_PID_CHANNEL_H_

typedef enum
{
    BALANCE_PID_STEERING = 0,
    BALANCE_PID_ANGLE,
    BALANCE_PID_GYRO,
} balance_pid_channel_enum;

#endif /* CODE_APP_BALANCE_PID_CHANNEL_H_ */
