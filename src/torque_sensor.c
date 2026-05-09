#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "uart_op.h"

#define DY200_CRC_RECONNECT_LIMIT 3
#define DY200_RECONNECT_DELAY_US (1000 * 500)
#define DY200_RECONNECT_COOLDOWN_US (1000 * 1000)
#define DY200_LOG_INTERVAL_US (1000LL * 1000LL * 5LL)

static uint16_t MODBUS_CRC16(unsigned char *buf, unsigned int len )
{
	static const uint16_t table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

	uint8_t xor = 0;
	uint16_t crc = 0xFFFF;

	while( len-- )
	{
		xor = (*buf++) ^ crc;
		crc >>= 8;
		crc ^= table[xor];
	}

	return crc;
}

static pthread_mutex_t g_sensor_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t t_id;
static int g_started = 0;
static int g_valid = 0;
static int g_fd = -1;
static int g_reconnect_requested = 0;
static int g_stop_requested = 0;
static char g_dev[128] = {0};
static int g_baud = 115200;
static float g_torque = 0.0f;
static float g_speed = 0.0f;
static float g_power = 0.0f;

static int dy200_should_stop(void);

static long long monotonic_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}

static void log_rate_limited(long long *last_log_us, const char *fmt, ...)
{
    long long now = monotonic_us();
    va_list args;

    if (*last_log_us != 0 && now - *last_log_us < DY200_LOG_INTERVAL_US) {
        return;
    }

    *last_log_us = now;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

static void mark_sensor_invalid(void)
{
    pthread_mutex_lock(&g_sensor_lock);
    g_valid = 0;
    pthread_mutex_unlock(&g_sensor_lock);
}

static int read_exact(int fd, unsigned char *buf, int len)
{
    int index = 0;

    while (index < len) {
        if (dy200_should_stop()) {
            return -1;
        }

        int read_len = uart_recv(fd, (char *)buf + index, len - index);
        if (read_len > 0) {
            index += read_len;
            continue;
        }

        if (read_len == 0 || errno == EINTR || errno == EAGAIN) {
            usleep(1000);
            continue;
        }

        return -1;
    }

    return 0;
}

static void update_sensor_values(float torque, float speed, float power)
{
    pthread_mutex_lock(&g_sensor_lock);
    g_torque = torque;
    g_speed = speed;
    g_power = power;
    g_valid = 1;
    pthread_mutex_unlock(&g_sensor_lock);
}

static void set_active_fd(int fd)
{
    pthread_mutex_lock(&g_sensor_lock);
    g_fd = fd;
    pthread_mutex_unlock(&g_sensor_lock);
}

static int take_reconnect_request(void)
{
    int requested;

    pthread_mutex_lock(&g_sensor_lock);
    requested = g_reconnect_requested;
    g_reconnect_requested = 0;
    pthread_mutex_unlock(&g_sensor_lock);
    return requested;
}

static int dy200_should_stop(void)
{
    int requested;

    pthread_mutex_lock(&g_sensor_lock);
    requested = g_stop_requested;
    pthread_mutex_unlock(&g_sensor_lock);
    return requested;
}

static void finish_sensor_thread(int fd)
{
    if (fd > 0) {
        close(fd);
    }

    pthread_mutex_lock(&g_sensor_lock);
    g_started = 0;
    g_valid = 0;
    g_fd = -1;
    g_reconnect_requested = 0;
    g_stop_requested = 0;
    pthread_mutex_unlock(&g_sensor_lock);
}

static void flush_sensor_uart(int fd)
{
    uart_flush(fd);
    usleep(1000);
    uart_flush(fd);
    usleep(1000);
    uart_flush(fd);
}

static int open_configured_sensor(void)
{
    char dev[sizeof(g_dev)];
    int baud;

    pthread_mutex_lock(&g_sensor_lock);
    snprintf(dev, sizeof(dev), "%s", g_dev);
    baud = g_baud;
    pthread_mutex_unlock(&g_sensor_lock);

    return uart_open_normal(dev, baud, 0);
}

static int reconnect_sensor(int fd)
{
    static long long last_reconnect_log_us = 0;

    if (fd > 0) {
        close(fd);
    }

    mark_sensor_invalid();
    usleep(DY200_RECONNECT_COOLDOWN_US);

    while (!dy200_should_stop()) {
        int new_fd = open_configured_sensor();
        if (new_fd > 0) {
            log_rate_limited(&last_reconnect_log_us, "dy200 reconnected\n");
            set_active_fd(new_fd);
            flush_sensor_uart(new_fd);
            return new_fd;
        }

        log_rate_limited(&last_reconnect_log_us, "dy200 reconnect failed\n");
        usleep(DY200_RECONNECT_DELAY_US);
    }

    return -1;
}

void *dy200_thread(void *arg)
{
    int fd = (long)arg;
    int crc_error_count = 0;
    long long last_read_log_us = 0;
    long long last_crc_log_us = 0;
    unsigned char uart_raw_buf[6];

    flush_sensor_uart(fd);

    while (!dy200_should_stop()) {
        uint16_t received_crc;
        uint16_t calc_crc;
        uint16_t raw_torque;
        uint16_t raw_speed;
        float torque;
        float speed;
        float power;

        if (take_reconnect_request()) {
            fd = reconnect_sensor(fd);
            if (fd < 0) {
                break;
            }
            crc_error_count = 0;
            continue;
        }

        if (read_exact(fd, uart_raw_buf, sizeof(uart_raw_buf)) != 0) {
            if (dy200_should_stop()) {
                break;
            }
            log_rate_limited(&last_read_log_us, "dy200 read failed, reconnecting\n");
            fd = reconnect_sensor(fd);
            if (fd < 0) {
                break;
            }
            crc_error_count = 0;
            continue;
        }

        received_crc = (uint16_t)uart_raw_buf[4] | ((uint16_t)uart_raw_buf[5] << 8);
        calc_crc = MODBUS_CRC16(uart_raw_buf, 4);
        if (calc_crc != received_crc) {
            crc_error_count++;
            if (crc_error_count >= DY200_CRC_RECONNECT_LIMIT) {
                log_rate_limited(
                    &last_crc_log_us,
                    "dy200 crc failed %d consecutive times, reconnecting\n",
                    crc_error_count
                );
                fd = reconnect_sensor(fd);
                if (fd < 0) {
                    break;
                }
                crc_error_count = 0;
            } else {
                flush_sensor_uart(fd);
            }
            continue;
        }

        crc_error_count = 0;

        raw_torque = ((uint16_t)uart_raw_buf[0] << 8) | uart_raw_buf[1];
        raw_speed = ((uint16_t)uart_raw_buf[2] << 8) | uart_raw_buf[3];

        torque = raw_torque / 10.0f * ((raw_speed & 0x8000) ? -1.0f : 1.0f);
        speed = raw_speed & 0x7FFF;
        power = torque * speed * (M_PI / 30.0f);
        update_sensor_values(torque, speed, power);
    }

    finish_sensor_thread(fd);
    return NULL;
}

int dy200_init(char *dev, int bud)
{   
    int fd = 0;

    pthread_mutex_lock(&g_sensor_lock);
    if (g_started) {
        snprintf(g_dev, sizeof(g_dev), "%s", dev);
        g_baud = bud;
        g_valid = 0;
        g_reconnect_requested = 1;
        pthread_mutex_unlock(&g_sensor_lock);
        return 0;
    }
    snprintf(g_dev, sizeof(g_dev), "%s", dev);
    g_baud = bud;
    g_stop_requested = 0;
    pthread_mutex_unlock(&g_sensor_lock);

    fd = uart_open_normal(dev, bud, 0);
    if (fd <= 0 ) {
        printf("dev:%s, open failed\n", dev);
        pthread_mutex_lock(&g_sensor_lock);
        g_dev[0] = '\0';
        pthread_mutex_unlock(&g_sensor_lock);
        return  -1;
    }

    if (-1 == pthread_create(&t_id, NULL, dy200_thread, (void *)(long)fd))
    {
        printf("dy200 thread create failed.\n");
        close(fd);
        return -1;
    }  

    set_active_fd(fd);

    pthread_mutex_lock(&g_sensor_lock);
    g_started = 1;
    g_valid = 0;
    pthread_mutex_unlock(&g_sensor_lock);

    return 0;  
}

void dy200_shutdown(void)
{
    pthread_t thread;
    int should_join = 0;

    pthread_mutex_lock(&g_sensor_lock);
    if (g_started) {
        g_stop_requested = 1;
        g_reconnect_requested = 0;
        g_valid = 0;
        thread = t_id;
        should_join = 1;
    }
    pthread_mutex_unlock(&g_sensor_lock);

    if (should_join) {
        pthread_join(thread, NULL);
    }
}

int get_dy200_info(float *torque, float *speed, float *power)
{
    int valid;

    pthread_mutex_lock(&g_sensor_lock);
    if (torque)
        *torque = g_torque;
    if (speed)
        *speed = g_speed;
    if (power)
        *power = g_power;
    valid = g_valid;
    pthread_mutex_unlock(&g_sensor_lock);

    return valid ? 0 : -1;
}

int dy200_is_running(void)
{
    int started;

    pthread_mutex_lock(&g_sensor_lock);
    started = g_started;
    pthread_mutex_unlock(&g_sensor_lock);
    return started;
}

int dy200_has_valid_data(void)
{
    int valid;

    pthread_mutex_lock(&g_sensor_lock);
    valid = g_valid;
    pthread_mutex_unlock(&g_sensor_lock);
    return valid;
}
