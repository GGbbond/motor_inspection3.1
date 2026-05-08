#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib")
#define sleep_ms(ms) Sleep(ms)
#define close_socket closesocket
typedef int socklen_t;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
#define sleep_ms(ms) usleep((ms) * 1000)
#define close_socket close
typedef int SOCKET;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

#ifndef SERVER_PORT
#define SERVER_PORT 9999
#endif

#define SERVER_ADDRESS "127.0.0.1"
#define MOTOR_COUNT 2
#define DRIVE_MOTOR 0
#define LOAD_MOTOR 1
#define UPDATE_DT_SEC 0.02
#define DEFAULT_MOVE_VEL_DEG_S 90.0
#define DEFAULT_MAX_TORQUE_NM 80.0
#define TORQUE_SMOOTHING 0.25
#define RECV_BUFFER_SIZE 1024
#define COMMAND_BUFFER_SIZE 2048
#define TELEMETRY_BUFFER_SIZE 256

typedef struct {
    double target_torque;
    double current_torque;
    double target_pos;
    double current_pos;
    double velocity;
    double move_velocity;
    double max_torque;
    int enabled;
    int moving;
    int completion_pending;
} MotorSimulator;

typedef struct {
    int active;
    double move_dir;
    double peak_torque;
    double total_time;
    double elapsed_time;
} DragLoadProfile;

static MotorSimulator g_motors[MOTOR_COUNT];
static DragLoadProfile g_load_profile;
static int g_running = 1;
static SOCKET g_client_sock = INVALID_SOCKET;

#ifdef _WIN32
static HANDLE g_update_thread;
static CRITICAL_SECTION g_mutex;
#else
static pthread_t g_update_thread;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif

static double clamp_double(double value, double min_value, double max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static double triangle_scale(double phase)
{
    phase = clamp_double(phase, 0.0, 1.0);
    if (phase <= 0.5) {
        return phase * 2.0;
    }
    return (1.0 - phase) * 2.0;
}

static double bidirectional_load_torque(double move_dir, double peak_torque, double progress)
{
    double local_phase;
    double sign;

    progress = clamp_double(progress, 0.0, 1.0);
    if (progress < 0.5) {
        local_phase = progress * 2.0;
        sign = -move_dir;
    } else {
        local_phase = (progress - 0.5) * 2.0;
        sign = move_dir;
    }

    return sign * fabs(peak_torque) * triangle_scale(local_phase);
}

static void mutex_lock(void)
{
#ifdef _WIN32
    EnterCriticalSection(&g_mutex);
#else
    pthread_mutex_lock(&g_mutex);
#endif
}

static void mutex_unlock(void)
{
#ifdef _WIN32
    LeaveCriticalSection(&g_mutex);
#else
    pthread_mutex_unlock(&g_mutex);
#endif
}

static void motor_init(MotorSimulator *motor)
{
    memset(motor, 0, sizeof(*motor));
    motor->move_velocity = DEFAULT_MOVE_VEL_DEG_S;
    motor->max_torque = DEFAULT_MAX_TORQUE_NM;
}

static void motor_disable(MotorSimulator *motor)
{
    motor->enabled = 0;
    motor->moving = 0;
    motor->target_torque = 0.0;
    motor->current_torque = 0.0;
    motor->velocity = 0.0;
}

static void motor_zero(MotorSimulator *motor)
{
    motor->current_pos = 0.0;
    motor->target_pos = 0.0;
    motor->velocity = 0.0;
    motor->moving = 0;
    motor->completion_pending = 0;
}

static void motor_set_torque(MotorSimulator *motor, double torque)
{
    motor->enabled = 1;
    motor->moving = 0;
    motor->target_torque = clamp_double(torque, -motor->max_torque, motor->max_torque);
}

static void motor_set_position(MotorSimulator *motor, double pos)
{
    motor->enabled = 1;
    motor->target_pos = pos;
    motor->move_velocity = DEFAULT_MOVE_VEL_DEG_S;
    motor->moving = 1;
    motor->completion_pending = 0;
}

static void motor_move_relative(MotorSimulator *motor, double pos_delta,
                                double torque, double velocity)
{
    motor->enabled = 1;
    motor->target_torque = clamp_double(torque, -motor->max_torque, motor->max_torque);
    motor->target_pos = motor->current_pos + pos_delta;
    motor->move_velocity = fabs(velocity);

    if (motor->move_velocity < 0.001) {
        motor->current_pos = motor->target_pos;
        motor->velocity = 0.0;
        motor->moving = 0;
        motor->completion_pending = 1;
        return;
    }

    motor->moving = 1;
    motor->completion_pending = 0;
}

static void motor_update(MotorSimulator *motor, double dt)
{
    if (!motor->enabled) {
        motor->velocity = 0.0;
        motor->current_torque = 0.0;
        return;
    }

    motor->current_torque += (motor->target_torque - motor->current_torque) * TORQUE_SMOOTHING;

    if (motor->moving) {
        double error = motor->target_pos - motor->current_pos;
        double direction = (error >= 0.0) ? 1.0 : -1.0;
        double step = motor->move_velocity * dt;

        if (fabs(error) <= step) {
            motor->current_pos = motor->target_pos;
            motor->velocity = 0.0;
            motor->moving = 0;
            motor->completion_pending = 1;
        } else {
            motor->current_pos += direction * step;
            motor->velocity = direction * motor->move_velocity;
        }
        return;
    }

    if (fabs(motor->target_torque) > 0.001) {
        motor->velocity += motor->target_torque * dt;
        motor->current_pos += motor->velocity * dt;
    } else {
        motor->velocity = 0.0;
    }
}

static void stop_load_profile(void)
{
    g_load_profile.active = 0;
    g_load_profile.elapsed_time = 0.0;
    g_motors[LOAD_MOTOR].target_torque = 0.0;
}

static void start_load_profile(double pos_delta, double load_torque, double velocity)
{
    double abs_velocity = fabs(velocity);

    stop_load_profile();
    if (fabs(pos_delta) < 0.001 || fabs(load_torque) < 0.001 || abs_velocity < 0.001) {
        return;
    }

    g_load_profile.active = 1;
    g_load_profile.move_dir = pos_delta >= 0.0 ? 1.0 : -1.0;
    g_load_profile.peak_torque = fabs(load_torque);
    g_load_profile.total_time = fabs(pos_delta) / abs_velocity;
    g_load_profile.elapsed_time = 0.0;
}

static void update_load_profile(double dt)
{
    double progress;
    double target_torque;

    if (!g_load_profile.active) {
        return;
    }

    if (g_load_profile.total_time <= 0.0 ||
        g_load_profile.elapsed_time >= g_load_profile.total_time) {
        stop_load_profile();
        return;
    }

    progress = g_load_profile.elapsed_time / g_load_profile.total_time;
    target_torque = bidirectional_load_torque(g_load_profile.move_dir,
                                              g_load_profile.peak_torque,
                                              progress);
    g_motors[LOAD_MOTOR].target_torque = clamp_double(target_torque,
                                                      -g_motors[LOAD_MOTOR].max_torque,
                                                      g_motors[LOAD_MOTOR].max_torque);
    g_load_profile.elapsed_time += dt;
}

static void start_drag_test(double pos_delta, double load_torque, double velocity)
{
    motor_move_relative(&g_motors[DRIVE_MOTOR], pos_delta, 0.0, velocity);
    motor_move_relative(&g_motors[LOAD_MOTOR], -pos_delta, 0.0, velocity);
    start_load_profile(pos_delta, load_torque, velocity);
}

static int socket_send_all(SOCKET client, const char *message, size_t len)
{
    size_t sent = 0;

    while (sent < len) {
        int result;
        size_t remaining = len - sent;
        int chunk = remaining > 16384 ? 16384 : (int)remaining;

#ifdef _WIN32
        result = send(client, message + sent, chunk, 0);
#else
        result = send(client, message + sent, chunk, MSG_NOSIGNAL);
#endif
        if (result == SOCKET_ERROR || result == 0) {
            return -1;
        }
        sent += (size_t)result;
    }

    return 0;
}

static int send_client_message(const char *message)
{
    SOCKET client;

    mutex_lock();
    client = g_client_sock;
    mutex_unlock();

    if (client == INVALID_SOCKET) {
        return 0;
    }

    if (socket_send_all(client, message, strlen(message)) != 0) {
        mutex_lock();
        if (g_client_sock == client) {
            g_client_sock = INVALID_SOCKET;
        }
        mutex_unlock();
        return -1;
    }

    return 0;
}

static int send_client_log(const char *fmt, ...)
{
    char body[256];
    char message[320];
    va_list args;

    va_start(args, fmt);
    vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);

    snprintf(message, sizeof(message), "LOG %s\n", body);
    return send_client_message(message);
}

static void handle_set_torque(double torque)
{
    mutex_lock();
    stop_load_profile();
    motor_set_torque(&g_motors[DRIVE_MOTOR], torque);
    mutex_unlock();
    printf("Set motor 1 torque: %.2f\n", torque);
    send_client_log("Set motor 1 torque: %.2f", torque);
}

static void handle_set_position(double pos)
{
    mutex_lock();
    stop_load_profile();
    motor_set_position(&g_motors[DRIVE_MOTOR], pos);
    mutex_unlock();
    printf("Set motor 1 position: %.2f\n", pos);
    send_client_log("Set motor 1 position: %.2f", pos);
}

static void handle_drag_command(double pos, double load_torque, double velocity)
{
    mutex_lock();
    start_drag_test(pos, load_torque, velocity);
    mutex_unlock();
    printf("Drag test: pos=%.2f load_torque=%.2f velocity=%.2f\n", pos, load_torque, velocity);
    send_client_log("Drag test: pos=%.2f load_torque=%.2f velocity=%.2f",
                    pos, load_torque, velocity);
}

static void handle_zero(void)
{
    mutex_lock();
    stop_load_profile();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_zero(&g_motors[i]);
    }
    mutex_unlock();
    printf("Zeroed simulated motors\n");
    send_client_log("Zeroed simulated motors");
}

static void handle_disable(void)
{
    mutex_lock();
    stop_load_profile();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_disable(&g_motors[i]);
    }
    mutex_unlock();
    printf("Disabled simulated motors\n");
    send_client_log("Disabled simulated motors");
}

static void handle_set_max_torque(double max_torque)
{
    char reply[64];
    double limit = fabs(max_torque);

    mutex_lock();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_motors[i].max_torque = limit;
        g_motors[i].target_torque = clamp_double(g_motors[i].target_torque, -limit, limit);
    }
    mutex_unlock();

    snprintf(reply, sizeof(reply), "MAX_TORQUE_SET %.2f\n", limit);
    send_client_message(reply);
    printf("Set max torque: %.2f\n", limit);
    send_client_log("Set max torque: %.2f", limit);
}

static void process_command(const char *line)
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    int parsed;

    if (sscanf(line, "SET_TORQUE %lf", &a) == 1) {
        handle_set_torque(a);
        return;
    }

    if (sscanf(line, "SET_POS %lf", &a) == 1) {
        handle_set_position(a);
        return;
    }

    parsed = sscanf(line, "POS_WITH_VEL %lf %lf %lf", &a, &b, &c);
    if (parsed == 2 || parsed == 3) {
        double load_torque = (parsed == 3) ? b : 0.0;
        double velocity = (parsed == 3) ? c : b;
        handle_drag_command(a, load_torque, velocity);
        return;
    }

    if (strncmp(line, "ZERO", 4) == 0) {
        handle_zero();
        return;
    }

    if (strncmp(line, "DISABLE_MOTOR", 13) == 0) {
        handle_disable();
        return;
    }

    if (sscanf(line, "SET_MAX_TORQUE %lf", &a) == 1) {
        handle_set_max_torque(a);
        return;
    }

    printf("Unknown command: %s\n", line);
    send_client_log("Unknown command: %s", line);
}

static void process_command_buffer(char *buffer, size_t *buffer_len,
                                   const char *recv_buf, int recv_size)
{
    size_t append_len = (size_t)recv_size;

    if (*buffer_len + append_len >= COMMAND_BUFFER_SIZE) {
        *buffer_len = 0;
    }

    memcpy(buffer + *buffer_len, recv_buf, append_len);
    *buffer_len += append_len;
    buffer[*buffer_len] = '\0';

    char *line_start = buffer;
    char *newline = strchr(line_start, '\n');
    while (newline) {
        *newline = '\0';
        if (newline > line_start && newline[-1] == '\r') {
            newline[-1] = '\0';
        }
        if (*line_start != '\0') {
            process_command(line_start);
        }
        line_start = newline + 1;
        newline = strchr(line_start, '\n');
    }

    *buffer_len = strlen(line_start);
    memmove(buffer, line_start, *buffer_len);
    buffer[*buffer_len] = '\0';
}

static void build_telemetry(char *buffer, size_t size)
{
    int complete;

    mutex_lock();
    update_load_profile(UPDATE_DT_SEC);
    motor_update(&g_motors[DRIVE_MOTOR], UPDATE_DT_SEC);
    motor_update(&g_motors[LOAD_MOTOR], UPDATE_DT_SEC);

    complete = g_motors[DRIVE_MOTOR].completion_pending ||
               g_motors[LOAD_MOTOR].completion_pending;
    g_motors[DRIVE_MOTOR].completion_pending = 0;
    g_motors[LOAD_MOTOR].completion_pending = 0;

    snprintf(buffer, size,
             "TORQUE1 %.2f\nPOS1 %.2f\nVEL1 %.2f\n"
             "TORQUE2 %.2f\nPOS2 %.2f\nVEL2 %.2f\n%s",
             g_motors[DRIVE_MOTOR].current_torque,
             g_motors[DRIVE_MOTOR].current_pos,
             g_motors[DRIVE_MOTOR].velocity,
             g_motors[LOAD_MOTOR].current_torque,
             g_motors[LOAD_MOTOR].current_pos,
             g_motors[LOAD_MOTOR].velocity,
             complete ? "POS_WITH_VEL_COMPLETE\n" : "");
    mutex_unlock();
}

#ifdef _WIN32
static DWORD WINAPI update_thread_func(LPVOID arg)
#else
static void *update_thread_func(void *arg)
#endif
{
    (void)arg;

    while (g_running) {
        char telemetry[TELEMETRY_BUFFER_SIZE];
        build_telemetry(telemetry, sizeof(telemetry));
        send_client_message(telemetry);
        sleep_ms((int)(UPDATE_DT_SEC * 1000));
    }

    return 0;
}

static void init_motors(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_init(&g_motors[i]);
    }
}

static int init_platform(void)
{
#ifndef _WIN32
    signal(SIGPIPE, SIG_IGN);
#else
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        printf("WSAStartup failed.\n");
        return -1;
    }
    InitializeCriticalSection(&g_mutex);
#endif
    return 0;
}

static void cleanup_platform(void)
{
#ifdef _WIN32
    DeleteCriticalSection(&g_mutex);
    WSACleanup();
#endif
}

static int start_update_thread(void)
{
#ifdef _WIN32
    g_update_thread = CreateThread(NULL, 0, update_thread_func, NULL, 0, NULL);
    return g_update_thread ? 0 : -1;
#else
    return pthread_create(&g_update_thread, NULL, update_thread_func, NULL);
#endif
}

static void join_update_thread(void)
{
#ifdef _WIN32
    WaitForSingleObject(g_update_thread, INFINITE);
    CloseHandle(g_update_thread);
#else
    pthread_join(g_update_thread, NULL);
#endif
}

static SOCKET create_server_socket(void)
{
    SOCKET server_sock;
    struct sockaddr_in server;
    int opt = 1;

    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock == INVALID_SOCKET) {
        printf("Could not create socket.\n");
        return INVALID_SOCKET;
    }

    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));

    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    server.sin_port = htons(SERVER_PORT);

    if (bind(server_sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
        printf("Bind failed.\n");
        close_socket(server_sock);
        return INVALID_SOCKET;
    }

    if (listen(server_sock, 3) < 0) {
        printf("Listen failed.\n");
        close_socket(server_sock);
        return INVALID_SOCKET;
    }

    return server_sock;
}

static void set_active_client(SOCKET client_sock)
{
    mutex_lock();
    g_client_sock = client_sock;
    mutex_unlock();
}

static void clear_active_client(SOCKET client_sock)
{
    mutex_lock();
    if (g_client_sock == client_sock) {
        g_client_sock = INVALID_SOCKET;
    }
    mutex_unlock();
}

static void handle_client(SOCKET client_sock)
{
    char recv_buf[RECV_BUFFER_SIZE];
    char command_buffer[COMMAND_BUFFER_SIZE];
    size_t command_len = 0;

    command_buffer[0] = '\0';

    while (1) {
        int recv_size = recv(client_sock, recv_buf, sizeof(recv_buf) - 1, 0);
        if (recv_size <= 0) {
            printf("Client disconnected.\n");
            break;
        }

        recv_buf[recv_size] = '\0';
        process_command_buffer(command_buffer, &command_len, recv_buf, recv_size);
    }
}

int main(void)
{
    SOCKET server_sock;

    init_motors();
    if (init_platform() != 0) {
        return 1;
    }

    server_sock = create_server_socket();
    if (server_sock == INVALID_SOCKET) {
        cleanup_platform();
        return 1;
    }

    printf("Motor simulator listening on %s:%d...\n", SERVER_ADDRESS, SERVER_PORT);

    if (start_update_thread() != 0) {
        printf("Update thread create failed.\n");
        close_socket(server_sock);
        cleanup_platform();
        return 1;
    }

    while (g_running) {
        SOCKET client_sock;
        struct sockaddr_in client;
        socklen_t client_len = sizeof(client);

        client_sock = accept(server_sock, (struct sockaddr *)&client, &client_len);
        if (client_sock == INVALID_SOCKET) {
            printf("Accept failed.\n");
            break;
        }

        set_active_client(client_sock);
        printf("Client connected.\n");
        handle_client(client_sock);
        clear_active_client(client_sock);
        close_socket(client_sock);
    }

    g_running = 0;
    join_update_thread();
    close_socket(server_sock);
    cleanup_platform();

    return 0;
}
