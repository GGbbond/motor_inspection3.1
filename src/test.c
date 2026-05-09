#include "mit_protocol.h"
#include "pid_control.h"
#include "common.h"
#include "torque_sensor.h"
#include "can_thread.h"
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include "math_ops.h"
#include "finsh.h"
#include "math.h"
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <stdarg.h>

// TCP服务器相关定义
#define SERVER_PORT 9999
#define BUFFER_SIZE 1024
#define CAN_TEXT_LOG_SLOTS 8
#define CAN_TEXT_LOG_BUFFER_SIZE 2048
#define PI_F 3.1415926f
#define RAD_TO_DEG(rad) ((rad) / PI_F * 180.0f)
#define DEG_TO_RAD(deg) ((deg) / 180.0f * PI_F)

int server_fd = -1;
int client_fd = -1;
pthread_t server_thread;

// 电机相关定义
#define MOTOR_1_ID  1
#define MOTOR_2_ID  2

#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

typedef struct
{
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
	__u8    __pad;   /* padding */
	__u8    __res0;  /* reserved / padding */
	__u8    __res1;  /* reserved / padding */
	__u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
}can_frame  __attribute__((__aligned__(1)));

typedef struct 
{
    float max_vel;
    float max_pos;
    float max_torque;
}mit_param;


typedef struct 
{
    joint_control control;
    joint_state state;
    pid_control pid;
    mHandle can;
    mit_param param;
    can_frame can_tx;
    int motor_enable;
    pthread_spinlock_t lock;
    int protocol;
}motor_mit;

typedef struct
{
    unsigned int can_id;
    char text[CAN_TEXT_LOG_BUFFER_SIZE];
    size_t len;
    int active;
} can_text_log;

#define MOTOR_NUM   2
motor_mit g_motor[MOTOR_NUM];
static can_text_log g_can_text_logs[CAN_TEXT_LOG_SLOTS];

// 函数声明
int set_motor_tor(motor_mit *motor, float tor);
int set_motor_vel(motor_mit *motor, float vel);
int motor_enable(motor_mit *motor, int enable);
int position_with_velocity(int argc, char *argv[]);

static int tcp_send_text(int fd, const char *text)
{
    return send(fd, text, strlen(text), 0);
}

static int tcp_send_log(int fd, const char *fmt, ...)
{
    char body[256];
    char message[320];
    va_list args;

    va_start(args, fmt);
    vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);

    snprintf(message, sizeof(message), "LOG %s\n", body);
    return tcp_send_text(fd, message);
}

static int tcp_send_can_line_if_connected(unsigned int can_id, const char *text)
{
    char message[CAN_TEXT_LOG_BUFFER_SIZE + 64];

    if (client_fd <= 0) {
        return 0;
    }

    snprintf(message, sizeof(message), "LOG_CAN_LINE %u %s\n", can_id, text);
    return tcp_send_text(client_fd, message);
}

static void can_text_log_flush(can_text_log *log);

static can_text_log *can_text_log_get(unsigned int can_id)
{
    can_text_log *empty = NULL;

    for (int i = 0; i < CAN_TEXT_LOG_SLOTS; i++) {
        if (g_can_text_logs[i].active && g_can_text_logs[i].can_id == can_id) {
            return &g_can_text_logs[i];
        }
        if (!g_can_text_logs[i].active && !empty) {
            empty = &g_can_text_logs[i];
        }
    }

    if (!empty) {
        empty = &g_can_text_logs[0];
        can_text_log_flush(empty);
    }

    empty->active = 1;
    empty->can_id = can_id;
    empty->len = 0;
    empty->text[0] = '\0';
    return empty;
}

static void can_text_log_flush(can_text_log *log)
{
    if (!log || !log->active) {
        return;
    }

    while (log->len > 0 &&
           (log->text[log->len - 1] == ' ' || log->text[log->len - 1] == '\t')) {
        log->len--;
    }
    log->text[log->len] = '\0';

    if (log->len > 0) {
        printf("[%u]:%s\n", log->can_id, log->text);
        tcp_send_can_line_if_connected(log->can_id, log->text);
    }

    log->len = 0;
    log->text[0] = '\0';
}

static void can_text_log_append_frame(unsigned int can_id, const unsigned char *data, int len)
{
    can_text_log *log = can_text_log_get(can_id);

    for (int i = 0; i < len; i++) {
        unsigned char ch = data[i];

        if (ch == '\0') {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            can_text_log_flush(log);
            continue;
        }

        if (log->len >= sizeof(log->text) - 1) {
            can_text_log_flush(log);
        }

        log->text[log->len++] = (char)ch;
        log->text[log->len] = '\0';
    }
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float triangle_scale(float phase)
{
    phase = clamp_float(phase, 0.0f, 1.0f);
    if (phase <= 0.5f) {
        return phase * 2.0f;
    }
    return (1.0f - phase) * 2.0f;
}

static float bidirectional_load_torque(float move_dir, float peak_torque, float progress)
{
    float local_phase;
    float sign;

    progress = clamp_float(progress, 0.0f, 1.0f);
    if (progress < 0.5f) {
        local_phase = progress * 2.0f;
        sign = -move_dir;
    } else {
        local_phase = (progress - 0.5f) * 2.0f;
        sign = move_dir;
    }

    return sign * fabs(peak_torque) * triangle_scale(local_phase);
}

static void set_all_motors_enabled(int enable)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_enable(&g_motor[i], enable);
    }
}

static void enable_all_motors_without_motion(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        g_motor[i].protocol = 0;
        g_motor[i].control.p_des = g_motor[i].state.p;
        g_motor[i].control.v_des = 0.0f;
        g_motor[i].control.t_ff = 0.0f;
        g_motor[i].control.kp = (i == 0) ? 300.0f : 0.0f;
        g_motor[i].control.kd = (i == 0) ? 5.0f : 0.0f;
        motor_enable(&g_motor[i], 1);
    }
}

static void set_all_max_torque(float max_torque)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        g_motor[i].param.max_torque = max_torque;
    }
}

static void zero_all_motors(void)
{
    struct can_frame zero_msg;
    memset(&zero_msg, 0, sizeof(zero_msg));
    zero_msg.can_dlc = 8;
    Zero(&zero_msg);

    zero_msg.can_id = MOTOR_1_ID;
    can_commu_send(g_motor[0].can, (char *)&zero_msg, sizeof(zero_msg));

    zero_msg.can_id = MOTOR_2_ID;
    can_commu_send(g_motor[1].can, (char *)&zero_msg, sizeof(zero_msg));
}

static int run_position_with_velocity(float pos, float load_torque, float velocity)
{
    char pos_str[32];
    char torque_str[32];
    char velocity_str[32];
    char *argv[] = {"position_with_velocity", pos_str, torque_str, velocity_str};

    snprintf(pos_str, sizeof(pos_str), "%.2f", pos);
    snprintf(torque_str, sizeof(torque_str), "%.2f", load_torque);
    snprintf(velocity_str, sizeof(velocity_str), "%.2f", velocity);

    return position_with_velocity(4, argv);
}

// TCP服务器清理函数
void cleanup_tcp_server() {
    if (client_fd > 0) {
        close(client_fd);
        client_fd = -1;
    }
    if (server_fd > 0) {
        close(server_fd);
        server_fd = -1;
    }
    pthread_cancel(server_thread);
    pthread_join(server_thread, NULL);
}

static void handle_pos_with_velocity_command(int client_socket, char *command)
{
    char *token = strtok(command, " ");
    float pos;
    float second;
    float load_torque = 0.0f;
    float velocity;

    token = strtok(NULL, " ");
    if (!token) {
        tcp_send_text(client_socket, "ERROR invalid POS_WITH_VEL\n");
        tcp_send_log(client_socket, "ERROR invalid POS_WITH_VEL");
        return;
    }
    pos = atof(token);

    token = strtok(NULL, " ");
    if (!token) {
        tcp_send_text(client_socket, "ERROR invalid POS_WITH_VEL\n");
        tcp_send_log(client_socket, "ERROR invalid POS_WITH_VEL");
        return;
    }
    second = atof(token);

    token = strtok(NULL, " ");
    velocity = second;
    if (token) {
        load_torque = second;
        velocity = atof(token);
    }

    tcp_send_log(client_socket, "Drag Start: pos=%.2f deg load=%.2f Nm velocity=%.2f deg/s",
                 pos, load_torque, velocity);
    run_position_with_velocity(pos, load_torque, velocity);
    tcp_send_log(client_socket, "Drag Complete");
    tcp_send_text(client_socket, "POS_WITH_VEL_COMPLETE\n");
}

static void handle_dy200_connect_command(int client_socket, char *command)
{
    char dev[128] = {0};
    int baud = 115200;
    int parsed;

    parsed = sscanf(command, "DY200_CONNECT %127s %d", dev, &baud);
    if (parsed < 1) {
        tcp_send_text(client_socket, "DY200_CONNECT_FAILED\n");
        tcp_send_log(client_socket, "DYN200 connect failed: invalid command");
        return;
    }

    if (dy200_init(dev, baud) != 0) {
        tcp_send_text(client_socket, "DY200_CONNECT_FAILED\n");
        tcp_send_log(client_socket, "DYN200 connect failed: dev=%s baud=%d", dev, baud);
        return;
    }

    tcp_send_text(client_socket, "DY200_CONNECTED\n");
    tcp_send_log(client_socket, "DYN200 connected: dev=%s baud=%d", dev, baud);
}

static void handle_dy200_disconnect_command(int client_socket)
{
    dy200_shutdown();
    tcp_send_text(client_socket, "DY200_DISCONNECTED\n");
    tcp_send_log(client_socket, "DYN200 disconnected");
}

static void handle_tcp_command(int client_socket, char *command)
{
    if (strncmp(command, "SET_TORQUE", 10) == 0) {
        float torque = atof(command + 11);
        set_motor_tor(&g_motor[0], torque);
        motor_enable(&g_motor[0], 1);
        printf("Set torque: %.2f\n", torque);
        tcp_send_log(client_socket, "Set torque: %.2f", torque);
    }
    else if (strncmp(command, "SET_POS", 7) == 0) {
        float pos = atof(command + 8);
        g_motor[0].control.p_des = DEG_TO_RAD(pos);
        printf("Set position: %.2f\n", pos);
        tcp_send_log(client_socket, "Set position: %.2f", pos);
    }
    else if (strncmp(command, "POS_WITH_VEL", 12) == 0) {
        handle_pos_with_velocity_command(client_socket, command);
    }
    else if (strncmp(command, "ZERO", 4) == 0) {
        zero_all_motors();
        printf("g_motor[0].state.p: %.2f\n", RAD_TO_DEG(g_motor[0].state.p));
        printf("g_motor[1].state.p: %.2f\n", RAD_TO_DEG(g_motor[1].state.p));
        printf("Received ZERO command\n");
        tcp_send_log(client_socket, "g_motor[0].state.p: %.2f", RAD_TO_DEG(g_motor[0].state.p));
        tcp_send_log(client_socket, "g_motor[1].state.p: %.2f", RAD_TO_DEG(g_motor[1].state.p));
        tcp_send_log(client_socket, "Received ZERO command");
    }
    else if (strncmp(command, "DISABLE_MOTOR", 13) == 0) {
        set_all_motors_enabled(0);
        printf("Motor disabled\n");
        tcp_send_log(client_socket, "Motor disabled");
    }
    else if (strncmp(command, "ENABLE_MOTOR", 12) == 0) {
        enable_all_motors_without_motion();
        printf("Motor enabled for position check\n");
        tcp_send_log(client_socket, "Motor enabled for position check");
    }
    else if (strncmp(command, "SET_MAX_TORQUE", 14) == 0) {
        char result[64];
        float max_torque = atof(command + 15);
        set_all_max_torque(max_torque);
        snprintf(result, sizeof(result), "MAX_TORQUE_SET %.2f\n", max_torque);
        tcp_send_text(client_socket, result);
        printf("Set max torque: %.2f\n", max_torque);
        tcp_send_log(client_socket, "Set max torque: %.2f", max_torque);
    }
    else if (strncmp(command, "DY200_CONNECT", 13) == 0) {
        handle_dy200_connect_command(client_socket, command);
    }
    else if (strncmp(command, "DY200_DISCONNECT", 16) == 0) {
        handle_dy200_disconnect_command(client_socket);
    }
}

// TCP服务器线程函数
void* tcp_server_thread(void* arg) {
    int server_socket, client_socket;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE];
    
    // 创建socket
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket failed");
        return NULL;
    }
    server_fd = server_socket;
    
    // 设置socket选项
    int opt = 1;
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt");
        return NULL;
    }
    
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(SERVER_PORT);
    
    // 绑定socket
    if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        return NULL;
    }
    
    // 监听连接
    if (listen(server_socket, 3) < 0) {
        perror("Listen failed");
        return NULL;
    }
    
    printf("TCP Server listening on port %d\n", SERVER_PORT);
    
    while (1) {
        // 接受客户端连接
        if ((client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &addr_len)) < 0) {
            perror("Accept failed");
            continue;
        }
        
        printf("Client connected: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
        client_fd = client_socket;
        
        while (1) {
            memset(buffer, 0, BUFFER_SIZE);
            int read_size = recv(client_socket, buffer, BUFFER_SIZE, 0);
            
            if (read_size <= 0) {
                printf("Client disconnected\n");
                close(client_socket);
                client_fd = -1;
                break;
            }

            char *saveptr = NULL;
            char *command = strtok_r(buffer, "\n", &saveptr);
            while (command) {
                size_t len = strlen(command);
                if (len > 0 && command[len - 1] == '\r') {
                    command[len - 1] = '\0';
                }
                if (command[0] != '\0') {
                    handle_tcp_command(client_socket, command);
                }
                command = strtok_r(NULL, "\n", &saveptr);
            }

        }
    }
    
    return NULL;
}

int vsec_pack_current(uint8_t *data, float current)
{
    if (fabs(current)  >= 420)
    {
        printf("error current:%.3f\n", current);
        return -1;
    }
    

    int temp = current * 1000;

    uint8_t *p = (uint8_t *)&temp;

    data[3] = p[0];
    data[2] = p[1];
    data[1] = p[2];
    data[0] = p[3];


    return 0;
}

int motor_enable(motor_mit *motor, int enable)
{
    motor->motor_enable = enable;
 
    return 0;
}

int set_motor_tor(motor_mit *motor, float tor)
{
    motor->control.t_ff = tor;
    return 0;
}

int set_motor_vel(motor_mit *motor, float vel)
{
    motor->control.v_des = vel;

    return 0;
}

int motor_test_can_call(void *arg, void *arg2, char *buf, int len)
{
    (void)arg2;
    (void)len;

    can_frame *frame = (can_frame *)buf;
    motor_mit *tmp = arg;
    int p_int, v_int, i_int;
    float p, v, t;
    motor_mit *motor = NULL;

    if (frame->can_id == (MOTOR_1_ID | 0x10))
    {
        motor = &tmp[0];
    }
    else if(frame->can_id == (MOTOR_2_ID | 0x10))
    {
       motor = &tmp[1];
    }
    else
    {
        can_text_log_append_frame(frame->can_id, frame->data, frame->can_dlc);
        return 0;
    }

    p_int = (frame->data[1] << 8) | frame->data[2];
    v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
    i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];
    /// convert uints to floats ///
    p = uint_to_float(p_int, -motor->param.max_pos, motor->param.max_pos, 16);
    v = uint_to_float(v_int, -motor->param.max_vel, motor->param.max_vel, 12);
    t = uint_to_float(i_int, -motor->param.max_torque, motor->param.max_torque, 12);

    motor->state.p = p;
    motor->state.v = v;
    motor->state.t = t;

    return 0;

}

void *commu_thread(void *arg)
{
    int enable_last[MOTOR_NUM] = {0};
    motor_mit *tmp = arg;

    while (1)
    {
        delay_ms(5);

        for (size_t i = 0; i < MOTOR_NUM; i++)
        {
            motor_mit *motor = &tmp[i];
            if (motor->protocol == 0)
            {
                if(!enable_last[i] && motor->motor_enable)
                {
                    printf("enable\n");
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    printf("disenable\n");
                    ExitMotorMode(motor->can_tx.data);
                }
                else
                {
                    pack_cmd_new(motor->can_tx.data, &motor->control, 
                    motor->param.max_pos, -motor->param.max_pos, 
                    motor->param.max_vel, -motor->param.max_vel, 
                    motor->param.max_torque, -motor->param.max_torque);
                }
                
                can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
                
                enable_last[i] = motor->motor_enable;
                usleep(1000*1);
            }
            else if (motor->protocol == 1)
            {
                if(!enable_last[i] && motor->motor_enable)
                {
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    ExitMotorMode(motor->can_tx.data);
                }
                else
                {
                    pack_cmd_vel(motor->can_tx.data, &motor->control, 
                    motor->param.max_vel, -motor->param.max_vel);

                }
                
                can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
                
                enable_last[i] = motor->motor_enable;
                usleep(1000*1);
            }
            else if (motor->protocol == 2)
            {
                if(!enable_last[i] && motor->motor_enable)
                {
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    ExitMotorMode(motor->can_tx.data);
                }
                else
                {
                    if(motor->motor_enable){
                        g_motor[1].pid.input = motor->state.v;
                        pid_caculate(&g_motor[1].pid);

                        motor->control.t_ff = g_motor[1].pid.output;
                    }else{
                        motor->control.t_ff = 0;
                    }

                    pack_cmd_new(motor->can_tx.data, &motor->control, 
                    motor->param.max_pos, -motor->param.max_pos, 
                    motor->param.max_vel, -motor->param.max_vel, 
                    motor->param.max_torque, -motor->param.max_torque);

                }
                
                can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
                
                enable_last[i] = motor->motor_enable;
            }
        }
        
        // 发送状态数据到客户端
        if (client_fd > 0) {
            char msg[384];
            float sensor_torque = NAN;
            float sensor_speed = NAN;
            float sensor_power = NAN;
            if (get_dy200_info(&sensor_torque, &sensor_speed, &sensor_power) != 0) {
                sensor_torque = NAN;
                sensor_speed = NAN;
            }

            snprintf(msg, sizeof(msg),
                    "TORQUE1 %.2f\nPOS1 %.2f\nVEL1 %.2f\n"
                    "TORQUE2 %.2f\nPOS2 %.2f\nVEL2 %.2f\n"
                    "SENSOR_TORQUE %.3f\nSENSOR_SPEED %.3f\n",
                    g_motor[0].state.t, 
                    RAD_TO_DEG(g_motor[0].state.p),
                    RAD_TO_DEG(g_motor[0].state.v),
                    g_motor[1].state.t,
                    RAD_TO_DEG(g_motor[1].state.p),
                    RAD_TO_DEG(g_motor[1].state.v),
                    sensor_torque,
                    sensor_speed);
            send(client_fd, msg, strlen(msg), 0);
            usleep(1000*10);
        }
    }
    

    return NULL;
}

int motor_test_init()
{
    signal(SIGPIPE, SIG_IGN);

    memset(g_motor, 0, sizeof(g_motor));

    g_motor[0].param.max_pos = 12.5;
    g_motor[0].param.max_vel = 45;
    g_motor[0].param.max_torque = 80;  // 根据测试电机修改 
    g_motor[0].protocol = 0;

    g_motor[1].param.max_pos = 12.5;
    g_motor[1].param.max_vel = 45;
    g_motor[1].param.max_torque = 40;
    g_motor[1].protocol = 0;

    g_motor[0].can = can_commu_init("can0", "can0", 0, 0xffff, motor_test_can_call, (void *)g_motor, NULL, 0);
    if (!g_motor[0].can)
        return -1;

    g_motor[1].can = g_motor[0].can;

    pid_zero(&g_motor[1].pid);

    g_motor[1].control.kp = 0.0;
    g_motor[1].control.kd = 0.0;
    g_motor[1].pid.kp = 0.5;
    g_motor[1].pid.ki = 0.1;
    g_motor[1].pid.output_max = 40.0;
    g_motor[1].pid.error_all_max = g_motor[1].pid.output_max / g_motor[1].pid.ki;
    g_motor[1].pid.error_max = 4.0;

    pthread_spin_init(&g_motor[0].lock, 0);
    g_motor[0].can_tx.can_dlc = 8;
    g_motor[0].can_tx.can_id = MOTOR_1_ID;

    pthread_spin_init(&g_motor[1].lock, 0);
    g_motor[1].can_tx.can_dlc = 8;
    g_motor[1].can_tx.can_id = MOTOR_2_ID;

    pthread_t t_id2;
    if (-1 == pthread_create(&t_id2, NULL, commu_thread, (void *)g_motor))
    {
        printf("commu thread create failed.\n");
        return -1;
    }  

    // 启动TCP服务器线程
    if (pthread_create(&server_thread, NULL, tcp_server_thread, NULL) != 0) {
        printf("Failed to create TCP server thread\n");
        return -1;
    }

    return 0;
}

int s_torq(int argc, char *argv[])
{
    float tor =0;
    if (argc == 2)
    {
        tor = atof(argv[1]);
    }

    set_motor_tor(&g_motor[0], tor);

    return 0;
}
FINSH_FUNCTION_EXPORT(s_torq, s_torq);

int s2_torq(int argc, char *argv[])
{
    float tor =0;
    if (argc == 2)
    {
        tor = atof(argv[1]);
    }

    set_motor_tor(&g_motor[1], tor);

    return 0;
}
FINSH_FUNCTION_EXPORT(s2_torq, s2_torq);

int s_vel(int argc, char *argv[])
{
    float vel =0;
    if (argc == 2)
    {
        vel = atof(argv[1]);
    }

    set_motor_vel(&g_motor[1], vel);

    return 0;
}
FINSH_FUNCTION_EXPORT(s_vel, s_vel);

int s_pid_vel(int argc, char *argv[])
{
    float vel =0;
    if (argc == 2)
    {
        vel = atof(argv[1]);
    }

    g_motor[1].pid.des = vel;
    // g_motor[1].control.v_des = vel;

    return 0;
}
FINSH_FUNCTION_EXPORT(s_pid_vel, s_pid_vel);

int m_enable(int argc, char *argv[])
{
    int enable =0;
    if (argc != 2)
        return -1;

    enable = atoi(argv[1]);

    motor_enable( &g_motor[0], enable);

    return 0;
}
FINSH_FUNCTION_EXPORT(m_enable, m_enable);


int m_enable2(int argc, char *argv[])
{
    int enable =0;
    if (argc != 2)
        return -1;

    enable = atoi(argv[1]);

    motor_enable( &g_motor[1], enable);

    return 0;
}
FINSH_FUNCTION_EXPORT(m_enable2, m_enable2);


int p_info(int argc, char *argv[])
{
    float torque,speed,power;

    printf("print info\n");

    get_dy200_info(&torque, &speed, &power);
    printf("torque:%.3f, speed:%.3f, power:%.3f\n", torque, speed, power);

    printf("motor pos:%.5f, vel:%.5f, tor:%.5f\n", g_motor[0].state.p, g_motor[0].state.v, g_motor[0].state.t);
    printf("motor pos:%.5f, vel:%.5f, tor:%.5f\n", g_motor[1].state.p, g_motor[1].state.v, g_motor[1].state.t);

    return 0;
 
}
FINSH_FUNCTION_EXPORT(p_info, p_info);

int change_protocol(int argc, char *argv[])
{
    if (argc != 3) {
        return -1;
    }
    if (atoi(argv[1]) == 0){
        g_motor[0].protocol = atoi(argv[2]);
    }else if (atoi(argv[1]) == 1){
        g_motor[1].protocol = atoi(argv[2]);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT(change_protocol, change_protocol);

int tor_test(int argc, char *argv[])
{

    float torque,speed,power;
        
    

    motor_enable( &g_motor[0], 1);

    printf("current, torque\n");

    float current = 0;
    int times = 40 * 10 * 2;
    for (int i = 0; i < times; i++)
    {
        current += (0.1 * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[0], current);
        usleep(1000*10);
        get_dy200_info(&torque, &speed, &power);
    }

    motor_enable( &g_motor[0], 0);

    return 0;
}
FINSH_FUNCTION_EXPORT(tor_test, tor_test);

int ttor_test_new(int argc, char *argv[])
{
    float torque,speed,power;
    float target_current = -5.0f;

    if (argc == 2) {
        target_current = atof(argv[1]);
    }
    printf("target current: %.3f\n", target_current);

    system("mkdir -p ../data");
    
    FILE *file = fopen("../data/current_torque.csv", "w");

    if(!file)
    {
        printf("open file failed\n");
    }

    g_motor[1].protocol = 2;

    motor_enable( &g_motor[0], 1);
    motor_enable( &g_motor[1], 1);

    g_motor[1].pid.des = 5.0f;   // 根据电机相对方向修改，若反向则为正，同向则为负
    usleep(1000*200);
 
    printf("current, torque\n");

    float current = 0;
    int times = 1000 * 2;
    for (int i = 0; i < times; i++)
    {
        current += (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[0], current);
        usleep(1000*10);
        get_dy200_info(&torque, &speed, &power);
        if(i <= (times/2) && i >=50){
            printf("%.3f, %.3f\n", current, torque);  // 修改扭矩正负输出，前面的系数根据扭矩输出修改
            if(file){
                fprintf(file, "%.3f, %.3f\n", current, torque);
            }
        }
    }

    motor_enable( &g_motor[0], 0);
    motor_enable( &g_motor[1], 0);

    g_motor[1].pid.des = 0.0f;

    g_motor[1].protocol = 0;

    if(file){
        fclose(file);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT(ttor_test_new, ttor_test_new);

int vves_test(int argc, char *argv[])
{
    float target_current = 10.0f;

    if (argc >= 2) {
        target_current = atof(argv[1]);
    }
    printf("target current: %.3f\n", target_current);

    g_motor[1].protocol = 2;
    if (argc >= 4) {
        g_motor[1].pid.kp = atof(argv[2]);
        g_motor[1].pid.ki = atof(argv[3]);
    }

    printf("kp:%.3f", g_motor[1].pid.kp);
    printf("ki:%.3f", g_motor[1].pid.ki);

    motor_enable( &g_motor[1], 1);

    g_motor[1].pid.des = 5.0f;   // 根据电机相对方向修改，若反向则为正，同向则为负
    usleep(1000*200);
 
    printf("current, torque\n");

    float current = 0;
    int times = 1000 * 1;
    for (int i = 0; i < times; i++)
    {
        current += (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1));
        usleep(1000*10);
    }

    motor_enable( &g_motor[1], 0);

    g_motor[1].pid.des = 0.0f;

    g_motor[1].protocol = 0;

    return 0;
}
FINSH_FUNCTION_EXPORT(vves_test, vves_test);

int p2_v(int argc, char *argv[])
{
    int times = 1000 * 5;
    for (int i = 0; i < times; i++)
    {
        printf("m2_v:%.3f\n\r", g_motor[1].state.v);
        usleep(1000*10);
    }

    return 0;

}
FINSH_FUNCTION_EXPORT(p2_v, p2_v);

int a_max_speed_tor_test(int argc, char *argv[])
{
    float torque,speed,power;
    float target_current = 10.0f;

    if (argc == 2) {
        target_current = atof(argv[1]);
    }
    printf("target current: %.3f\n", target_current);

    system("mkdir -p ../data");

    FILE *file = fopen("../data/speed_torque.csv", "w");

    if(!file)
    {
        printf("open file failed\n");
    }

    motor_enable( &g_motor[0], 1);
    motor_enable( &g_motor[1], 1);

    printf("speed, torque\n");

    float current1 = 0;
    float current2 = 0;
    int times = 1000 * 2;

    for (int i = 0; i < times / 2; i++)
    {
        current1 += (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[0], current1);
        usleep(1000*10);
    }

    printf("%.3f\n", current1);

    for (int i = 0; i < times; i++)
    {
        current2 += (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1)); // 根据电机相对大小修改，85-85：1，70-85：1.3，50-85：1.8
        set_motor_tor(&g_motor[1], current2);
        usleep(1000*10);
        get_dy200_info(&torque, &speed, &power);
        if(i <= (times/2)){
            printf("%.3f, %.3f\n", g_motor[0].state.v * 30.0f / M_PI, 1 * torque);  // 修改扭矩正负输出，前面的系数根据扭矩输出修改
            if(file){
                fprintf(file, "%.3f, %.3f\n", g_motor[0].state.v * 30.0f / M_PI, 1 * torque);
            }
        }
    }

    for (int i = 0; i < times / 2; i++)
    {
        current1 -= (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1));
        set_motor_tor(&g_motor[0], current1);
        usleep(1000*10);
    }

    printf("%.3f\n", current1);

    motor_enable( &g_motor[0], 0);
    motor_enable( &g_motor[1], 0);

    if(file){
        fclose(file);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT(a_max_speed_tor_test, a_max_speed_tor_test);

int extreme_condition_testing(int argc, char *argv[])
{
    float target_current = 20.0f;

    if (argc == 2) {
        target_current = atof(argv[1]);
    }
    printf("target current: %.3f\n", target_current);

    motor_enable( &g_motor[0], 1);
    motor_enable( &g_motor[1], 1);

    usleep(1000*2000);

    float current_1 = 0;
    float current_2 = 0;
    int times = 200 * 2 * 18;
    for (int i = 0; i < times; i++)
    {
        current_1 = ((-1 * target_current) + 1.0 * (rand()%RAND_MAX) / RAND_MAX * (target_current * 2)) * 2;
        current_2 = ((-1 * target_current) + 1.0 * (rand()%RAND_MAX) / RAND_MAX * (target_current * 2)) * 2;

        if(current_1 > target_current)
            current_1 = target_current;
        if(current_1 < -target_current)
            current_1 = -target_current;

        if(current_2 > target_current)
            current_2 = target_current;
        if(current_2 < -target_current) 
            current_2 = -target_current;
        
        set_motor_tor(&g_motor[0], current_1);
        set_motor_tor(&g_motor[1], current_2);
        usleep(1000*50);

        printf("current:%.3f, %.3f\n", current_1, current_2);
    }

    motor_enable( &g_motor[0], 0);
    motor_enable( &g_motor[1], 0);

    return 0;
}
FINSH_FUNCTION_EXPORT(extreme_condition_testing, extreme_condition_testing);

int vesc_tor_test(int argc, char *argv[])
{
    float torque,speed,power;
        
    float current = 0;

    float max = 50;
    float sec = 2;

    if (argc >= 2)
    {
        max = atof(argv[1]);
        if (argc == 3)
        {
            sec = atof(argv[2]);
        }
    }

    if (max >= 420 || sec >= 30)
    {
        printf("wrong param, max:%.3f sec:%.3f\n", max, sec);
        return -1;
    }

    printf("test param, max:%.3f sec:%.3f\n", max, sec);

#define FILE_BUF 64
    FILE *file = fopen("./test.csv", "w");
    char bufer[FILE_BUF] = {0};
    if(!file)
    {
        printf("open file failed\n");
        return -1;
    }

    printf("\n*****************test start*************\n\n");

    sprintf(bufer, "current, torque\n");
    fwrite(bufer, 1, strlen(bufer), file);
    printf("%s", bufer);

    float interval = 0.01;

    int times = sec / interval;

    float step = max / (float)times;

    int loop_num = times * 2;

    for (int i = 0; i < loop_num; i++)
    {
        current += ( step * (i <= times ? 1 : -1) );

        set_motor_tor(&g_motor[0], current);

        delay_us(interval * 1000 * 1000);

        get_dy200_info(&torque, &speed, &power);

        memset(bufer, 0, FILE_BUF);
        sprintf(bufer, "%.3f, %.3f\n", current, torque);
        fwrite(bufer, 1, strlen(bufer), file);
        printf("%s", bufer);

    }

    fclose(file);

    return 0;
}
FINSH_FUNCTION_EXPORT(vesc_tor_test, vesc_tor_test);

static __inline int kbhit(void)
{

    fd_set rfds;
    struct timeval tv;
    int retval;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    retval = select(1, &rfds, NULL, NULL, &tv);

    if (retval == -1)
    {
        perror("select()");
        return 0;
    }
    else if (retval)
        return 1;
    else
        return 0;
    return 0;
}

int dir_test2(int argc, char *argv[])
{
    float vel = -1;
    float tor = 10;
    int time = 2;

    if (argc >= 3)
    {
        vel = atof(argv[1]);
        tor = atof(argv[2]);
    }
    if (argc >= 4)
    {
        time = atoi(argv[3]);
    }
    


    motor_enable( &g_motor[0], 1);
    motor_enable( &g_motor[1], 1);

    while (!kbhit())
    {
        set_motor_tor(&g_motor[0], tor);
        set_motor_vel(&g_motor[1], vel);

        sleep(time);
        if (kbhit())
            break;

        set_motor_tor(&g_motor[0], -tor*0.5);
        set_motor_vel(&g_motor[1], vel);     

        sleep(time);
        if (kbhit())
            break;  
    }
    

        



    motor_enable( &g_motor[0], 0);
    motor_enable( &g_motor[1], 0);

    return 0;
}
FINSH_FUNCTION_EXPORT(dir_test2, dir_test2);

int position_with_velocity(int argc, char *argv[])
{
    float target_pos = 0.0f;
    float load_torque = 0.0f;
    float velocity = 1.0f;

    if (argc >= 2) {
        float relative_pos = clamp_float(atof(argv[1]), -360.0f, 360.0f);
        target_pos = DEG_TO_RAD(relative_pos);
    }
    if (argc >= 4) {
        load_torque = atof(argv[2]);
        velocity = atof(argv[3]);
    } else if (argc >= 3) {
        velocity = atof(argv[2]);
    }
    if (fabs(velocity) < 0.001f) {
        printf("Invalid velocity: %.3f deg/s\n", velocity);
        return -1;
    }
    velocity = DEG_TO_RAD(fabs(velocity));

    g_motor[0].protocol = 0;
    g_motor[1].protocol = 0;

    g_motor[0].control.kp = 300.0;
    g_motor[0].control.kd = 5.0;
    g_motor[1].control.kp = 0.0;
    g_motor[1].control.kd = 0.0;
    g_motor[0].control.t_ff = 0.0f;
    g_motor[1].control.t_ff = 0.0f;

    motor_enable(&g_motor[0], 1);
    motor_enable(&g_motor[1], 1);

    float start_pos = g_motor[0].state.p;
    printf("Drag Start: %.3f deg, Target: %.3f deg, Velocity: %.3f deg/s, Load: %.3f Nm\n",
           RAD_TO_DEG(start_pos),
           RAD_TO_DEG(target_pos),
           RAD_TO_DEG(velocity),
           load_torque);

    float total_distance = target_pos;
    float total_time = fabs(total_distance) / velocity;
    float move_dir = total_distance >= 0.0f ? 1.0f : -1.0f;

    struct timeval start_time, current_time;
    gettimeofday(&start_time, NULL);

    while (!kbhit()) {
        gettimeofday(&current_time, NULL);
        float elapsed_time = (current_time.tv_sec - start_time.tv_sec) +
                            (current_time.tv_usec - start_time.tv_usec) / 1000000.0f;

        if (elapsed_time >= total_time) {
            g_motor[0].control.p_des = start_pos + total_distance;
            g_motor[1].control.t_ff = 0.0f;
            break;
        } else {
            float progress = elapsed_time / total_time;

            g_motor[0].control.p_des = start_pos + total_distance * progress;
            g_motor[1].control.t_ff = bidirectional_load_torque(move_dir, load_torque, progress);
        }

        usleep(1000 * 1);
    }

    g_motor[1].control.t_ff = 0.0f;

    return 0;
}
FINSH_FUNCTION_EXPORT(position_with_velocity, position_with_velocity);


int drag_test(int argc, char *argv[])
{
    if (argc < 4) {
        printf("usage: drag_test <pos_deg> <velocity_deg_s> <load_torque_nm>\n");
        return -1;
    }

    char *drag_argv[] = {"drag_test", argv[1], argv[3], argv[2]};
    return position_with_velocity(4, drag_argv);
}
FINSH_FUNCTION_EXPORT(drag_test, drag_test);
