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

// TCP服务器相关定义
#define SERVER_PORT 9999
#define BUFFER_SIZE 1024

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
    // unsigned int can_id;
    // unsigned int can_dlc;
    // unsigned char data[CAN_MAX_DLC];
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

#define MOTOR_NUM   2
motor_mit g_motor[MOTOR_NUM];

// 函数声明
int set_motor_tor(motor_mit *motor, float tor);
int set_motor_vel(motor_mit *motor, float vel);
int motor_enable(motor_mit *motor, int enable);

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
            
            // 解析命令
            if (strncmp(buffer, "SET_TORQUE", 10) == 0) {
                float torque = atof(buffer + 11);
                set_motor_tor(&g_motor[0], torque);
                motor_enable(&g_motor[0], 1);  // 使能电机
                printf("Set torque: %.2f\n", torque);
            }
            else if (strncmp(buffer, "SET_POS", 7) == 0) {
                float pos = atof(buffer + 8);
                g_motor[0].control.p_des = pos / 180.0f * 3.1415926f; // 角度转弧度
                printf("Set position: %.2f\n", pos);
            }
            else if (strncmp(buffer, "POS_WITH_VEL", 12) == 0) {
                // 解析参数：位置、扭矩、速度
                char* token = strtok(buffer, " ");
                token = strtok(NULL, " "); // 跳过命令
                float pos = atof(token);
                token = strtok(NULL, " ");
                float torque = atof(token);
                token = strtok(NULL, " ");
                float velocity = atof(token);

                // 创建字符串缓冲区来存储转换后的数值
                char pos_str[32], torque_str[32], velocity_str[32];

                // 将浮点数转换为字符串
                snprintf(pos_str, sizeof(pos_str), "%.2f", pos);
                snprintf(torque_str, sizeof(torque_str), "%.2f", torque);
                snprintf(velocity_str, sizeof(velocity_str), "%.2f", velocity);

                // 使用字符串指针创建argv数组
                char* argv[] = {"", pos_str, torque_str, velocity_str};

                // 调用position_with_velocity函数
                position_with_velocity(4, argv);
                
                // 发送结果
                char result[256];
                snprintf(result, sizeof(result), "POS_WITH_VEL_COMPLETE\n");
                send(client_socket, result, strlen(result), 0);
            }
            else if(strncmp(buffer, "ZERO", 4) == 0){
                can_frame zero_msg;
                memset(&zero_msg, 0, sizeof(zero_msg));
                zero_msg.can_id = MOTOR_1_ID;
                zero_msg.can_dlc = 8;
                Zero(&zero_msg);
                can_commu_send(g_motor[0].can, (char *)&zero_msg, sizeof(zero_msg));
                printf("g_motor[0].state.p: %.2f\n", g_motor[0].state.p / 3.1415926f * 180.0f); // 打印当前角度

                printf("Received ZERO command\n");
            }
            else if (strncmp(buffer, "DISABLE_MOTOR", 13) == 0) {
                motor_enable(&g_motor[0], 0);
                motor_enable(&g_motor[1], 0);
                printf("Motor disabled\n");
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
    // pthread_spin_lock(&motor->lock);
    motor->motor_enable = enable;
    // pthread_spin_unlock(&motor->lock);
 
    return 0;
}

int set_motor_tor(motor_mit *motor, float tor)
{
    // pthread_spin_lock(&motor->lock);
    motor->control.t_ff = tor;
    // pthread_spin_unlock(&motor->lock);
    return 0;
}

int set_vesc_tor(motor_mit *motor, float tor)
{
    // pthread_spin_lock(&motor->lock);
    motor->control.t_ff = tor;
    // pthread_spin_unlock(&motor->lock);

    return 0;
}

int set_motor_vel(motor_mit *motor, float vel)
{
    // pthread_spin_lock(&motor->lock);
    motor->control.v_des = vel;
    // pthread_spin_unlock(&motor->lock);

    return 0;
}

int motor_test_can_call(void *arg, void *arg2, char *buf, int len)
{
    // return 0;
    // printf("can recv\n");

    can_frame *frame = (can_frame *)buf;
    motor_mit *tmp = arg;
    int p_int, v_int, i_int;
    float p, v, t;
    motor_mit *motor = NULL;

    // printf("can id:%d, dlc:%d, %d\n", frame->can_id, frame->can_dlc, frame->data[0]);

    // if (frame->data[0] == (MOTOR_1_ID & 0x10))
    if (frame->can_id == (MOTOR_1_ID | 0x10))
    {
        motor = &tmp[0];
    }
    // else if(frame->data[0] == (MOTOR_2_ID & 0x10))
    else if(frame->can_id == (MOTOR_2_ID | 0x10))
    {
       motor = &tmp[1];
    }
    else
    {
        printf("[%d]:", frame->can_id);
        for(int i = 0; i < frame->can_dlc; i++){
            printf("%c", frame->data[i]);
        }
        printf("\n\r");
        return 0;
    }

    // printf("motor msg\n");
    p_int = (frame->data[1] << 8) | frame->data[2];
    v_int = (frame->data[3] << 4) | (frame->data[4] >> 4);
    i_int = ((frame->data[4] & 0xF) << 8) | frame->data[5];
    /// convert uints to floats ///
    p = uint_to_float(p_int, -motor->param.max_pos, motor->param.max_pos, 16);
    v = uint_to_float(v_int, -motor->param.max_vel, motor->param.max_vel, 12);
    t = uint_to_float(i_int, -motor->param.max_torque, motor->param.max_torque, 12);

// pthread_spin_lock(&motor->lock);
    motor->state.p = p;
    motor->state.v = v;
    motor->state.t = t;

// pthread_spin_unlock(&motor->lock);

    return 0;

}

void *commu_thread(void *arg)
{
    int enable_last[MOTOR_NUM] = {0};
    motor_mit *tmp = arg;

    while (1)
    {
        
        //usleep(1000*10);
        delay_ms(5);

        // motor_mit *motor = &tmp[0];

        // pthread_spin_lock(&motor->lock);

        // vsec_pack_current(motor->can_tx.data, motor->control.t_ff);

        // can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));

        // pthread_spin_unlock(&motor->lock);

        // continue;

        for (size_t i = 0; i < MOTOR_NUM; i++)
        {
            motor_mit *motor = &tmp[i];

            if (motor->protocol == 0)
            {
                // pthread_spin_lock(&motor->lock);

                if(!enable_last[i] && motor->motor_enable)
                {
                    //enter;
                    printf("enable\n");
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    //exit
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
                // pthread_spin_unlock(&motor->lock);
                
                enable_last[i] = motor->motor_enable;
                usleep(1000*1);
            }
            else if (motor->protocol == 1)
            {
                // pthread_spin_lock(&motor->lock);

                if(!enable_last[i] && motor->motor_enable)
                {
                    //enter;
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    //exit
                    ExitMotorMode(motor->can_tx.data);
                }
                else
                {
                    pack_cmd_vel(motor->can_tx.data, &motor->control, 
                    motor->param.max_vel, -motor->param.max_vel);

                }
                
                can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
                // printf("protocol 1\n");
                
                // pthread_spin_unlock(&motor->lock);
                
                enable_last[i] = motor->motor_enable;
                usleep(1000*1);
            }
            else if (motor->protocol == 2)
            {
                // pthread_spin_lock(&motor->lock);

                if(!enable_last[i] && motor->motor_enable)
                {
                    //enter;
                    EnterMotorMode(motor->can_tx.data);
                }
                else if(enable_last[i] && !motor->motor_enable)
                {
                    //exitv
                    ExitMotorMode(motor->can_tx.data);
                }
                else
                {
                    if(motor->motor_enable){
                        float KT = 0.07f;
                        float GR = (7056.f/361.f);

                        g_motor[1].pid.input = motor->state.v;
                        pid_caculate(&g_motor[1].pid);
                        // printf("%.2f, %.2f, %.2f, %.2f\n", g_motor[1].pid.error, g_motor[1].pid.output, g_motor[1].pid.input, g_motor[1].pid.des);
                        // printf("%.2f\n", g_motor[1].pid.output);

                        motor->control.t_ff = g_motor[1].pid.output;
                    }else{
                        motor->control.t_ff = 0;

                        // g_motor[1].pid.input = motor->state.v;
                        // pid_caculate(&g_motor[1].pid);
                        // printf("%.2f\n", g_motor[1].pid.output);
                    }

                    pack_cmd_new(motor->can_tx.data, &motor->control, 
                    motor->param.max_pos, -motor->param.max_pos, 
                    motor->param.max_vel, -motor->param.max_vel, 
                    motor->param.max_torque, -motor->param.max_torque);

                }
                
                can_commu_send(motor->can, (char *)&motor->can_tx, sizeof(can_frame));
                // printf("protocol 2\n");
                // pthread_spin_unlock(&motor->lock);
                
                enable_last[i] = motor->motor_enable;
                // usleep(1000*1);
            }
            
            
            // usleep(1000*10);
        }
        
        // 发送状态数据到客户端
        if (client_fd > 0) {
            char msg[256];
            snprintf(msg, sizeof(msg), "TORQUE %.2f\nPOS %.2f\n", 
                    g_motor[0].state.t, 
                    g_motor[0].state.p / 3.1415926f * 180.0f); // 弧度转角度
            send(client_fd, msg, strlen(msg), 0);
            usleep(1000*10);
        }
    }
    

    return NULL;
}

int motor_test_init()
{
    memset(g_motor, 0, sizeof(g_motor));

    // if(dy200_init("/dev/ttyUSB0", 115200))
    //     return -1;

    g_motor[0].param.max_pos = 12.5;
    g_motor[0].param.max_vel = 45;
    g_motor[0].param.max_torque = 80;  // 根据测试电机修改 
    g_motor[0].protocol = 0;

    g_motor[1].param.max_pos = 12.5;
    g_motor[1].param.max_vel = 45;
    g_motor[1].param.max_torque = 80;
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

    // // 使能电机以便实时获取 CAN 数据
    // motor_enable(&g_motor[0], 1);

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

// pthread_spin_lock(&g_motor[0].lock);
//     g_motor[0].control.t_ff = tor;
set_motor_tor(&g_motor[0], tor);
// pthread_spin_unlock(&g_motor[0].lock);

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

// pthread_spin_lock(&g_motor[0].lock);
//     g_motor[0].control.t_ff = tor;
set_motor_tor(&g_motor[1], tor);
// pthread_spin_unlock(&g_motor[0].lock);

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

// pthread_spin_lock(&g_motor[0].lock);
//     g_motor[0].control.t_ff = tor;
set_motor_vel(&g_motor[1], vel);
// pthread_spin_unlock(&g_motor[0].lock);

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


// pthread_spin_lock(&g_motor[0].lock);
    motor_enable( &g_motor[0], enable);
// pthread_spin_unlock(&g_motor[0].lock);

return 0;
}
FINSH_FUNCTION_EXPORT(m_enable, m_enable);


int m_enable2(int argc, char *argv[])
{
    int enable =0;
    if (argc != 2)
        return -1;

    enable = atoi(argv[1]);


// pthread_spin_lock(&g_motor[1].lock);
    motor_enable( &g_motor[1], enable);
// pthread_spin_unlock(&g_motor[1].lock);

return 0;
}
FINSH_FUNCTION_EXPORT(m_enable2, m_enable2);


int p_info(int argc, char *argv[])
{
    float torque,speed,power;

    printf("print info\n");

    get_dy200_info(&torque, &speed, &power);
    printf("torque:%.3f, speed:%.3f, power:%.3f\n", torque, speed, power);

// pthread_spin_lock(&g_motor[0].lock);
    printf("motor pos:%.5f, vel:%.5f, tor:%.5f\n", g_motor[0].state.p, g_motor[0].state.v, g_motor[0].state.t);
// pthread_spin_unlock(&g_motor[0].lock);

// pthread_spin_lock(&g_motor[1].lock);
    printf("motor pos:%.5f, vel:%.5f, tor:%.5f\n", g_motor[1].state.p, g_motor[1].state.v, g_motor[1].state.t);
// pthread_spin_unlock(&g_motor[1].lock);

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
        // printf("current: %.3f, state.t: %.3f, torque: %.3f\n", current, g_motor[0].state.t, torque*-1);
        // printf("p:%.3f, v:%.3f, t:%.3f\n", g_motor[0].state.p, g_motor[0].state.v, g_motor[0].state.t);
    }

    motor_enable( &g_motor[0], 0);

    return 0;
}
FINSH_FUNCTION_EXPORT(tor_test, tor_test);

int ttor_test_new(int argc, char *argv[])
{
    float torque,speed,power;
    float KT = 0.07f;
    float GR = (7056.f/361.f);
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

    fclose(file);

    return 0;
}
FINSH_FUNCTION_EXPORT(ttor_test_new, ttor_test_new);

int vves_test(int argc, char *argv[])
{
    float target_current = 10.0f;

    if (argc == 2) {
        target_current = atof(argv[1]);
    }
    printf("target current: %.3f\n", target_current);

    g_motor[1].protocol = 2;
    g_motor[1].pid.kp = atof(argv[2]);
    g_motor[1].pid.ki = atof(argv[3]);

    printf("kp:%.3f", g_motor[1].pid.kp);
    printf("ki:%.3f", g_motor[1].pid.ki);

    motor_enable( &g_motor[1], 1);

    g_motor[1].pid.des = 3.0f;   // 根据电机相对方向修改，若反向则为正，同向则为负
    usleep(1000*200);
 
    printf("current, torque\n");

    float current = 0;
    int times = 1000 * 1;
    for (int i = 0; i < times; i++)
    {
        current += (target_current * 2.0f / times * (i >= (times/2) ? -1 : 1));
        // set_motor_tor(&g_motor[0], current);
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
    float ves = g_motor[1].state.v;

    int times = 1000 * 5;
    for (int i = 0; i < times; i++)
    {
        printf("m2_v:%.3f\n\r", ves);
        usleep(1000*10);
    }

    return 0;

}
FINSH_FUNCTION_EXPORT(p2_v, p2_v);

int a_max_speed_tor_test(int argc, char *argv[])
{
    float torque,speed,power;
    float KT = 0.07;
    float GR = (7056.f/361.f);
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

        // if(i <= (times/2)){
        //     printf("current2%.3f\n", current2);
        // }
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

    fclose(file);

    return 0;
}
FINSH_FUNCTION_EXPORT(a_max_speed_tor_test, a_max_speed_tor_test);

int extreme_condition_testing(int argc, char *argv[])
{
    float torque,speed,power;
    float KT = 0.07;
    float GR = (7056.f/361.f);
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
    // int times = 100 * 4 * 5;
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
        set_motor_tor(&g_motor[0], current_2);
        usleep(1000*50);

        // printf("time:%.3f\n\r", times / 10 / 60.0f);
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
    }

    printf("test param, max:%.3f sec:%.3f\n", max, sec);

#define FILE_BUF 64
    FILE *file = fopen("./test.csv", "w");
    char bufer[FILE_BUF] = {0};

    printf("\n*****************test start*************\n\n");

    //memset(bufer, 0, FILE_BUF);
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

        //usleep(interval * 1000 * 1000);
        delay_us(interval * 1000 * 1000);

        get_dy200_info(&torque, &speed, &power);

        memset(bufer, 0, FILE_BUF);
        sprintf(bufer, "%.3f, %.3f\n", current, torque);
        fwrite(bufer, 1, strlen(bufer), file);
        printf("%s", bufer);

    }

    // motor_enable( &g_motor[0], 0);
    fclose(file);

    return 0;
}
FINSH_FUNCTION_EXPORT(vesc_tor_test, vesc_tor_test);

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/sysinfo.h>
static __inline int kbhit(void)
{

    fd_set rfds;
    struct timeval tv;
    int retval;

    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    /* Wait up to five seconds. */
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    retval = select(1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */

    if (retval == -1)
    {
        perror("select()");
        return 0;
    }
    else if (retval)
        return 1;
    /* FD_ISSET(0, &rfds) will be true. */
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


// int position_with_torque(int argc, char *argv[])
// {
//     float target_pos = 0.0f;  // 目标位置（角度）
//     float torque = 0.0f;      // 输出力矩（Nm）
    
//     // 从命令行获取参数
//     if (argc >= 2) target_pos = atof(argv[1]);
//     if (argc >= 3) torque = atof(argv[2]);
//     //角度转弧度
//     target_pos = target_pos / 180.0f * 3.1415926f;
    
//     // 设置协议0模式
//     g_motor[0].protocol = 0;
    
//     // 配置MIT协议控制参数
//     g_motor[0].control.kp = 200.0;  // 位置比例增益
//     g_motor[0].control.kd = 0.0;  // 速度阻尼增益
    
//     // 使能电机
//     motor_enable(&g_motor[0], 1);
    
    
//     // // 设置输出力矩
//     g_motor[0].control.t_ff = torque;
    
//     // 设置目标位置
//     g_motor[0].control.p_des = target_pos;

    

//     printf("Position: %.3f rad, Torque: %.3f Nm\n", target_pos, torque);
    
//     //如果电机力矩改变，打印力矩值
//     float last_pos = 0.0f;
//     while (!kbhit()) {
//         if (kbhit())
//             break;  
//         if (fabs(g_motor[0].state.p - last_pos) > 0.01) { // 位置变化超过0.01rad时打印
//             printf("p_des: %.3f, Current Position: %.3f, Torque: %.3f, Curent_Torque: %.3f\n", 
//                    g_motor[0].control.p_des / 3.1415926f * 180.0f, g_motor[0].state.p / 3.1415926f * 180.0f, g_motor[0].control.t_ff, g_motor[0].state.t);
//             last_pos = g_motor[0].state.p;
//         }
//         usleep(1000 * 100); // 每100ms检查一次
//     }
//     motor_enable( &g_motor[0], 0);

//     return 0;
// }
// FINSH_FUNCTION_EXPORT(position_with_torque, position_with_torque);

// int pid(int argc, char *argv[])
// {
//     g_motor[0].pid.kp = atof(argv[2]);  // 动态设置Kp
//     g_motor[0].pid.ki = atof(argv[3]);  // 动态设置Ki
//     // ...
// }
// FINSH_FUNCTION_EXPORT(pid, pid);

int position_with_velocity(int argc, char *argv[])
{
    float target_pos = 0.0f;  // 目标位置（角度）
    float torque = 0.0f;      // 输出力矩（Nm）
    float velocity = 1.0f;   // 转动速度（度/秒）
    
    // 从命令行获取参数
    if (argc >= 2) {
        float relative_pos = atof(argv[1]);
        // 限制在-180到180度范围内
        if (relative_pos > 360.0f) relative_pos = 360.0f;
        if (relative_pos < -360.0f) relative_pos = -360.0f;
        // 将相对角度转换为弧度
        target_pos = relative_pos / 180.0f * 3.1415926f;
    }
    if (argc >= 3) torque = atof(argv[2]);
    if (argc >= 4) velocity = atof(argv[3]);
    // 速度转为弧度/秒
    velocity = velocity / 180.0f * 3.1415926f;

    
    // 设置协议0模式
    g_motor[0].protocol = 0;
    
    // 配置MIT协议控制参数
    g_motor[0].control.kp = 300.0;  // 位置比例增益
    g_motor[0].control.kd = 5.0;    // 速度阻尼增益，增加阻尼以减少震荡
    
    // 使能电机
    motor_enable(&g_motor[0], 1);
    
    // 设置输出力矩
    g_motor[0].control.t_ff = torque;
    
    // 设置起始位置为当前位置
    float start_pos = g_motor[0].state.p;
    float current_des = start_pos;
    
    printf("Start Position: %.3f deg, Target Position: %.3f deg, Velocity: %.3f deg/s, Torque: %.3f Nm\n", 
           start_pos / 3.1415926f * 180.0f, target_pos / 3.1415926f * 180.0f, velocity / 3.1415926f * 180.0f, torque);//显示起始位置角度、目标位置、速度和力矩
    
    // 计算需要移动的总距离（相对于当前位置）
    float total_distance = target_pos;  // 直接使用相对距离
    float total_time = fabs(total_distance) / velocity;

    
    // 记录开始时间
    struct timeval start_time, current_time;
    gettimeofday(&start_time, NULL);
    
    // 记录上次打印位置
    float last_pos = start_pos;
    
    while (!kbhit()) {
        // 获取当前时间
        gettimeofday(&current_time, NULL);
        
        // 计算已经过的时间（秒）
        float elapsed_time = (current_time.tv_sec - start_time.tv_sec) + 
                            (current_time.tv_usec - start_time.tv_usec) / 1000000.0f;
        
        // 如果已经超过总时间，退出循环
        if (elapsed_time >= total_time) {
            break; 
        } else {
            // 使用线性插值计算当前位置
            float progress = elapsed_time / total_time;
            g_motor[0].control.p_des = start_pos + total_distance * progress;
        }

        
        // 打印位置信息（当位置变化超过0.01rad时）
        // if (fabs(g_motor[0].state.p - last_pos) > 0.01) {
        //     printf("p_des: %.3f, Current Position: %.3f, Torque: %.3f, Current_Torque: %.3f\n", 
        //            g_motor[0].control.p_des / 3.1415926f * 180.0f, 
        //            g_motor[0].state.p / 3.1415926f * 180.0f, 
        //            g_motor[0].control.t_ff, 
        //            g_motor[0].state.t);
        //     last_pos = g_motor[0].state.p;
        // }
        

        // 检查是否已经到达目标位置（允许一定误差）
        if (fabs(g_motor[0].state.p - (start_pos + target_pos)) < 0.01 && elapsed_time >= total_time) {
            printf("Target position reached!\n");
            break;
        }
        
        usleep(1000 * 1); // 1ms更新一次
    }

    return 0;
}
FINSH_FUNCTION_EXPORT(position_with_velocity, position_with_velocity);
