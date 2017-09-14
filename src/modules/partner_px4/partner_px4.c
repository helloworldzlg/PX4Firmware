/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file partner_px4.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
// #include <v1.0/common/mavlink.h>
// #include <v1.0/protocol.h>
// #include <v1.0/mavlink_types.h>
// #include <v1.0/common/mavlink_msg_heartbeat.h>

__EXPORT int partner_px4_main(int argc, char *argv[]);

int partner_px4_main(int argc, char *argv[])
{
	PX4_INFO("partner_px4_main start!");

    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

    att.roll  = 1;
    att.pitch = 2;
    att.yaw   = 3;

    while (1)
    {
        att.yaw++;
        orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);

        sleep(1);
    }
    
#if 0
	while (1)
	{
        if (send_sync == 1)
        {
            send_sync = 0;
            mavlink_msg_heartbeat_send(MAVLINK_COMM_2,
                                       MAV_TYPE_GENERIC,
                                       MAV_AUTOPILOT_GENERIC, 0, 0, 0);
        }
        
        if (image_buffer_1_flag == BUFFER_NOBODY)
        {
            image_update = 1;
            image_buffer_1_flag = BUFFER_CPU;
#if 1
            mavlink_msg_data_transmission_handshake_send(MAVLINK_COMM_2,
            MAVLINK_DATA_STREAM_IMG_RAW8U,
            IMAGE_WIDTH_VAL*IMAGE_HEIGH_VAL,
            IMAGE_WIDTH_VAL, 
            IMAGE_HEIGH_VAL,
            IMAGE_WIDTH_VAL*IMAGE_HEIGH_VAL / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
            MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
            100);

            uint16_t frame = 0;
            for (frame = 0; frame < IMAGE_HEIGH_VAL*IMAGE_WIDTH_VAL / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1;
                    frame++)
            { 
                mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2,
                frame,
                &((uint8_t *) image_buffer_1)[frame * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN]);
            }
#else
            serial_write(g_uart_image_fd, image_buffer_8bit_1, sizeof(image_buffer_8bit_1));
#endif
            image_buffer_1_flag = BUFFER_USED;
        }

        if (image_buffer_2_flag == BUFFER_NOBODY)
        {
            image_update = 2;
            image_buffer_2_flag = BUFFER_CPU;
#if 1
            mavlink_msg_data_transmission_handshake_send(MAVLINK_COMM_2,
            MAVLINK_DATA_STREAM_IMG_RAW8U,
            IMAGE_WIDTH_VAL*IMAGE_HEIGH_VAL,
            IMAGE_WIDTH_VAL, 
            IMAGE_HEIGH_VAL,
            IMAGE_WIDTH_VAL*IMAGE_HEIGH_VAL / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1,
            MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN,
            100);

            uint16_t frame = 0;
            for (frame = 0; frame < IMAGE_HEIGH_VAL*IMAGE_WIDTH_VAL / MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN + 1;
                    frame++)
            { 
                mavlink_msg_encapsulated_data_send(MAVLINK_COMM_2,
                frame,
                &((uint8_t *) image_buffer_2)[frame * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN]);
            }
#else
            serial_write(g_uart_image_fd, image_buffer_8bit_1, sizeof(image_buffer_8bit_1));
#endif
            image_buffer_2_flag = BUFFER_USED;

        }   
#endif
	PX4_INFO("partner_px4_main end!");

	return 0;
}
