#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <iso646.h>
#include "string.h"
#include "esp_adc_cal.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_timer.h"

#include "time.h"
#include "sys/time.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_task_wdt.h"

//Motor
#define MAX_SPEED 200
#define L_PWM 5
#define L_FORW 4
#define L_BACK 16
#define R_PWM 17
#define R_FORW 18
#define R_BACK 19
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<L_FORW) | (1ULL<<L_BACK) | (1ULL<<R_FORW) | (1ULL<<R_BACK))
#define PWM_CHANNEL_LEFT LEDC_CHANNEL_0
#define PWM_CHANNEL_RIGHT LEDC_CHANNEL_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0
#define PWM_FREQ_HZ 5000 // Frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution

//Bluetooth
#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "Trouble Maker"

// IMU
#define I2C_MASTER_SCL_IO            22  // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO            23  // GPIO number for I2C master data
#define I2C_MASTER_NUM               I2C_NUM_0  // I2C port number
#define I2C_MASTER_FREQ_HZ           400000  // I2C master clock frequency
#define LSM6DSO_ADDRESS              0x6B  // Device address of LSM6DSO
#define ACK_CHECK_EN                 0x1  // I2C master will check ack from slave
#define ACK_VAL                      0x0  // I2C ack value
#define NACK_VAL                     0x1  // I2C nack value
#define ACCEL_SENSITIVITY            0.061  // mg/LSB for ±2g range
#define GRAVITY                      9.81  // m/s^2
#define GYRO_SENSITIVITY 8.75  // mdps/LSB for ±250 dps range
static const char *TAG = "LSM6DSO";

//Encoder
#define LED GPIO_NUM_4
#define ENCODER GPIO_NUM_13

// p control
float V0 = 0.0;
float current_velocity = 0.0;
const float max_speed = 0.5; //m/s
const int max_PWM = 255;
float desired_speed[4] = {(190.0/max_PWM)*max_speed, max_speed*(255.0/max_PWM), max_speed*(255.0/max_PWM), max_speed*(190/max_PWM)};
//float desired_speed[4] = {(220.0/max_PWM)*max_speed, max_speed*(250.0/max_PWM), max_speed*(250.0/max_PWM), max_speed*(250/max_PWM)};
float kp = 10.0;
float saturation = 1.0; 
float u [4] = {0.0, 0.0, 0.0, 0.0};
int output_PWM [4] = {0, 0, 0, 0};

//Astar
#define map_size_rows 10
#define map_size_cols 10
volatile char *dir_arr;
float stepsize;

//Sensor
TaskHandle_t Task2_Handle;
const float distanceThreshold = 10.0; //cm

static TaskHandle_t spp_server_handle = NULL;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;
static struct timeval time_new, time_old;
static long data_num = 0;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;
esp_spp_cb_param_t copy_param;

void pin_def();
void move_forward(int left_output, int right_output);
void move_backward(int left_output, int right_output);
void stop();
void turn_left();
void turn_right();
void turn_90_degrees();

// Function prototypes
esp_err_t i2c_write_register(uint8_t reg, uint8_t data);
esp_err_t i2c_read_register(uint8_t reg, uint8_t *data, size_t len);

void pin_def() {

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // 8-bit resolution
        .freq_hz = 5000, // Frequency in Hz
        .speed_mode = PWM_MODE, 
        .timer_num = PWM_TIMER
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_left = {
        .channel    = PWM_CHANNEL_LEFT,
        .duty       = 0,
        .gpio_num   = L_PWM,
        .speed_mode = PWM_MODE,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&ledc_channel_left);

    ledc_channel_config_t ledc_channel_right = {
        .channel    = PWM_CHANNEL_RIGHT,
        .duty       = 0,
        .gpio_num   = R_PWM,
        .speed_mode = PWM_MODE,
        .timer_sel  = PWM_TIMER
    };
    ledc_channel_config(&ledc_channel_right);
}

void move_forward(int left_output, int right_output) {
    printf("Moving Forward\n");
    gpio_set_level(L_FORW, 0);
    gpio_set_level(L_BACK, 1);
    gpio_set_level(R_FORW, 0);
    gpio_set_level(R_BACK, 1);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_LEFT, left_output);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_RIGHT, right_output);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_LEFT);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_RIGHT);
}

void move_backward(int left_output, int right_output) {
    printf("Moving Backward\n");
    gpio_set_level(L_FORW, 1);
    gpio_set_level(L_BACK, 0);
    gpio_set_level(R_FORW, 1);
    gpio_set_level(R_BACK, 0);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_LEFT, left_output);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_RIGHT, right_output);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_LEFT);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_RIGHT);
}

void turn_left() {
    printf("Moving left\n");
    int speed_left = 0;
    int speed_right = 200;

    gpio_set_level(L_FORW, 1);
    gpio_set_level(L_BACK, 0);
    gpio_set_level(R_FORW, 1);
    gpio_set_level(R_BACK, 0);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_LEFT, speed_left);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_RIGHT, speed_right);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_LEFT);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_RIGHT);
}

void turn_right() {
    printf("Moving right\n");
    int speed_left = 0;
    int speed_right = 200;

    gpio_set_level(L_FORW, 1);
    gpio_set_level(L_BACK, 0);
    gpio_set_level(R_FORW, 0);
    gpio_set_level(R_BACK, 1);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_LEFT, speed_left);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_RIGHT, speed_right);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_LEFT);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_RIGHT);
}

void stop() {
    printf("Stop\n");
    gpio_set_level(L_FORW, 0);
    gpio_set_level(L_BACK, 0);
    gpio_set_level(R_FORW, 0);
    gpio_set_level(R_BACK, 0);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_LEFT, 0);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_RIGHT, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_LEFT);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_RIGHT);
}

esp_err_t i2c_write_register(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_register(uint8_t reg, uint8_t *data, size_t len) {
    if (len == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSO_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void sensor_setup() {
    ESP_LOGI(TAG, "Initializing LSM6DSO...");
    i2c_master_init();

    // Example of writing to a register
    //ESP_ERROR_CHECK(i2c_write_register(0x10, 0x20)); // Hypothetical setup value to a control register
    ESP_ERROR_CHECK(i2c_write_register(0x10, 0x60)); // CTRL1_XL
    ESP_ERROR_CHECK(i2c_write_register(0x11, 0x40)); 

    ESP_LOGI(TAG, "Ready.");
    ESP_LOGI(TAG, "Loaded Settings.");
}

struct state {
    int step;
    float initial_angle;
};

int facing_direction[2] = {1,0};
float current_xy[2] = {0.0,0.0};
float current_xy_labview[2] = {0.0,0.0};
float absdiff2;
float prev_distance;
int s=0;

struct state turn(int step, float initial_angle, float current_angle, char direction) {
    float target_angle=initial_angle;
    struct state return_state;

    if (direction == 'L') {
        target_angle = initial_angle + 90.0;
        if (target_angle > 180.0) {
            target_angle -= 360.0;
        }
    } else if (direction == 'R') {
        target_angle = initial_angle - 90.0;
        if (target_angle < -180.0) {
            target_angle += 360.0;
        }
    }

    float ANGLE_TOLERANCE = 3; //degree

    if (abs(current_angle - target_angle) <= ANGLE_TOLERANCE) {
        if (direction == 'L') {
            int temp = facing_direction[0];
            facing_direction[0]=facing_direction[1];
            facing_direction[1]=-temp;
        } else {
            int temp = facing_direction[0];
            facing_direction[0]=-facing_direction[1];
            facing_direction[1]=temp;
        }
        return_state.step=step+1;
        return_state.initial_angle=current_angle;
        stop();
        vTaskDelay(pdMS_TO_TICKS(500));
        return return_state;
    }

    if (direction == 'L') {
        turn_left();
    } else if (direction == 'R') {
        turn_right(); 
    }

    return_state.step=step;
    return_state.initial_angle=initial_angle;
    return return_state;
}

struct state straight(int step, float initial_distance, float current_distance, float distance) {
    float DISTANCE_TOLERANCE = 0.01; 
    float target_distance=initial_distance+distance;
    struct state return_state;
    float absdiff = fabs(target_distance - current_distance);
    absdiff2 = fabs(current_distance - initial_distance);

    if (absdiff <= DISTANCE_TOLERANCE) {
        current_xy[0]+=distance * facing_direction[0] /stepsize;
        current_xy[1]+=distance * facing_direction[1] /stepsize;
        return_state.step=step+1;
        return_state.initial_angle=current_distance;
        return return_state;
    }

   const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback,
        .name = "My Timer"};
    esp_timer_handle_t timer_handler;
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 20));

    for (int j = 0; j < 4; j++){
        if (output_PWM [j] < 180){
            output_PWM [j] = 180;
        }
    else if (output_PWM [j] > 255){
            output_PWM [j] = 255;
        }
    }

    //printf("output_PWM[2] : %d\n",  output_PWM[2]);

    if (dir_arr[step-1]=='R') {
        move_forward(output_PWM[0],output_PWM[1]);
    } else {
        move_forward(output_PWM[2],output_PWM[3]);
    }

    current_xy_labview[0]+=(current_distance-prev_distance) * facing_direction[0];
    current_xy_labview[1]+=(current_distance-prev_distance) * facing_direction[1];
    prev_distance=current_distance;

    return_state.step=step;
    return_state.initial_angle=initial_distance;
    return return_state;
}

//Astar
/* description of graph node */
struct stop {
    double col, row;
    /* array of indexes of routes from this stop to neighbours in array of all routes */
    int * n;
    int n_len;
    double f, g, h;
    int from;
};

/* description of route between two nodes */
struct route {
    /* route has only one direction! */
    int x; /* index of stop in array of all stops of src of this route */
    int y; /* intex of stop in array of all stops od dst of this route */
    double d;
};

// The map that the car will operate //
char map[map_size_rows][map_size_cols] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1, 1, 1, 0, 1},
    {1, 0, 0, 1, 0, 0, 0, 1, 0, 1},
    {1, 0, 0, 1, 0, 0, 0, 1, 0, 1},
    {1, 0, 0, 1, 1, 1, 0, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

// Initialize the route map //
int ind[map_size_rows][map_size_cols] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};

volatile float *distance_arr;
size_t size;

// Run Astar with the map //
void runastar(void)
{
    // Define the necessary variables
    int i, j, k, l, b;
    int p_len = 0;
    int * path = NULL;
    int closed_len = 0;
    int * closed = NULL;
    int open_len = 1;
    int * open = (int*)calloc(open_len, sizeof(int));
    double min, current_g;
    int end_number;
    int current = 0;
    int source_len = 0;
    struct steploc * step_point = NULL;
    int path_len = 0;
    struct p * path = NULL;
    for (i = 1; i < map_size_rows - 1; i++) {
        for (j = 1; j < map_size_cols - 1; j++) {
            if (!map[i][j]) {
                ++source_len;
                step_point = (struct steploc *)realloc(step_point, source_len * sizeof(struct steploc));
                int t = source_len - 1;
                step_point[t].col = j;
                step_point[t].row = i;
                step_point[t].from = -1;
                step_point[t].g = DBL_MAX;
                step_point[t].n_len = 0;
                step_point[t].n = NULL;
                point_number[i][j] = t;
            }
        }
    }
    // /* point_numberex of start step */
    // source_point = 0;
    /* point_numberex of finish step */
    end_number = source_len - 1;
    // Caluculate the heuristic with the Euclidean distance
    for (i = 0; i < source_len; i++) {
        step_point[i].h = sqrt(pow(step_point[end_number].row - step_point[i].row, 2) + pow(step_point[end_number].col - step_point[i].col, 2));
    }
    for (i = 1; i < map_size_rows - 1; i++) {
        for (j = 1; j < map_size_cols - 1; j++) {
            if (point_number[i][j] >= 0) {
                for (k = i - 1; k <= i + 1; k++) {
                    for (l = j - 1; l <= j + 1; l++) {
                        if ((k == i) and (l == j)) {
                            continue;
                        }
                        if (point_number[k][l] >= 0) {
                            ++path_len;
                            path = (struct p *)realloc(path, path_len * sizeof(struct p));
                            int t = path_len - 1;
                            path[t].x = point_number[i][j];
                            path[t].y = point_number[k][l];
                            path[t].d = sqrt(pow(step_point[path[t].y].row - step_point[path[t].x].row, 2) + pow(step_point[path[t].y].col - step_point[path[t].x].col, 2));
                            ++step_point[path[t].x].n_len;
                            step_point[path[t].x].n = (int*)realloc(step_point[path[t].x].n, step_point[path[t].x].n_len * sizeof(int));
                            step_point[path[t].x].n[step_point[path[t].x].n_len - 1] = t;
                        }
                    }
                }
            }
        }
    }
    open[0] = source;
    step_point[source].g = 0;
    step_point[source].f = step_point[source].g + step_point[source].h;
    int found = 0;
    while (open_len and not found) {
        min = DBL_MAX;
        // Search for the point with minimum f
        for (i = 0; i < open_len; i++) {
            if (step_point[open[i]].f < min) {
                current = open[i];
                min = step_point[open[i]].f;
            }
        }
        // Save the path if current point equals the terminal point
        if (current == end_number) {
            found = 1; // The path is found
            ++p_len;
            path = (int*)realloc(path, p_len * sizeof(int));
            path[p_len - 1] = current;
            while (step_point[current].from >= 0) {
                current = step_point[current].from;
                ++p_len;
                path = (int*)realloc(path, p_len * sizeof(int));
                path[p_len - 1] = current;
            }
        }
        // Remove the current point from the open list
        for (i = 0; i < open_len; i++) {
            if (open[i] == current) {
                if (i not_eq (open_len - 1)) {
                    for (j = i; j < (open_len - 1); j++) {
                        open[j] = open[j + 1];
                    }
                }
                --open_len;
                open = (int*)realloc(open, open_len * sizeof(int));
                break;
            }
        }
        // Add the current point to the closed
        ++closed_len;
        closed = (int*)realloc(closed, closed_len * sizeof(int));
        closed[closed_len - 1] = current;
        // Update the values with the points
        for (i = 0; i < step_point[current].n_len; i++) {
            b = 0;
            for (j = 0; j < closed_len; j++) {
                if (path[step_point[current].n[i]].y == closed[j]) {
                    b = 1;
                }
            }
            if (b) {
                continue;
            }
            // Compute the current g value of the route
            current_g = step_point[current].g + path[step_point[current].n[i]].d;
            b = 1;
            // Determine if the point is in the open list
            if (open_len > 0) {
                for (j = 0; j < open_len; j++) {
                    if (path[step_point[current].n[i]].y == open[j]) {
                        b = 0;
                    }
                }
            }
            // Update the f g h values of the point
            if (b or (current_g < step_point[path[step_point[current].n[i]].y].g)) {
                step_point[path[step_point[current].n[i]].y].from = current;
                step_point[path[step_point[current].n[i]].y].g = current_g;
                step_point[path[step_point[current].n[i]].y].f = step_point[path[step_point[current].n[i]].y].g + step_point[path[step_point[current].n[i]].y].h;
                if (b) {
                    ++open_len;
                    open = (int*)realloc(open, open_len * sizeof(int));
                    open[open_len - 1] = path[step_point[current].n[i]].y;
                }
            }
        }
    }
    if (not found) {
        puts("Path NOT found");
    } else {
        printf("The optimal path is:\n");
        for (i = p_len - 1; i >= 0; i--) {
            printf("(%1.0f, %1.0f)\n", step_point[path[i]].col, step_point[path[i]].row);
        }
    }

    // Get the command lines for the control algorithm
    distance_arr=(float*)malloc((p_len-2)*5*sizeof(float)); // Set the distance array
    dir_arr=(char*)malloc((p_len-2)*5*sizeof(char)); // Set the next command where 1 = F, 2 = L, 3 = R
    int step = 0;
    stepsize = 0.5; // Set 0.3m
    int orient = 3; // Orientation of the car: 1 = up, 2 = right, 3 = down, 4 = left
    int prev_row = stops[path[p_len-1]].row;
    int prev_col = stops[path[p_len-1]].col;
    for (i = p_len - 2; i >= 0; i--){
        // Move towards right
        if (prev_col > stops[path[i]].col) {
            // Need facing right: orient = 2
            if (orient!=2){
                if (orient==1){
                    dir_arr[step] = 'R';
                    distance_arr[step] = 0.0;
                    step++;
                }else if (orient==3){
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;
                }else{
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;                        
                }
            }
            if (prev_row > stops[path[i]].row){
                // Move upper right
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                dir_arr[step] = 'L';
                distance_arr[step] = 0.0;
                step++;
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                orient = 1;
            }else if (prev_row < stops[path[i]].row){
                // Move lower right
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                dir_arr[step] = 'R';
                distance_arr[step] = 0.0;
                step++;
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                orient = 3;
            }else{
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                orient = 2;
            }              
        // Move towards left
        }else if (prev_col < stops[path[i]].col){
            // Need facing left: orient = 4
            if (orient!=4){
                if (orient==3){
                    dir_arr[step] = 'R';
                    distance_arr[step] = 0.0;
                    step++;
                }else if (orient==1)
                {
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;
                }else{
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;                        
                }
            }
            
            if (prev_row > stops[path[i]].row){
                // Move upper left
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                dir_arr[step] = 'R';
                distance_arr[step] = 0.0;
                step++;
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                orient = 1;
            }else if (prev_row < stops[path[i]].row){
                // Move lower left
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                dir_arr[step] = 'L';
                distance_arr[step] = 0.0;
                step++;
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                orient = 3;
            }else{
                // Move toward left
                dir_arr[step] = 'F';
                distance_arr[step] = stepsize;
                step++;
                orient = 4;
            }
        }else if (prev_row < stops[path[i]].row){
            // Need facing down: orient = 3
            if (orient!=3){
                if (orient==2){
                    dir_arr[step] = 'R';
                    distance_arr[step] = 0.0;
                    step++;
                }else if (orient==4){
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;
                }else{
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;                        
                }
            }
            orient = 3;
            dir_arr[step] = 'F';
            distance_arr[step] = stepsize;
            step++;
        }else if (prev_row > stops[path[i]].row){
            // Need facing up: orient = 1
            if (orient!=1){
                if (orient==4){
                    dir_arr[step] = 'R';
                    distance_arr[step] = 0.0;
                    step++;
                }else if (orient==2){
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;
                }else{
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;
                    dir_arr[step] = 'L';
                    distance_arr[step] = 0.0;
                    step++;                        
                }
            }
            orient = 1;
            dir_arr[step] = 'F';
            distance_arr[step] = stepsize;
            step++;
        }
        prev_row = stops[path[i]].row;
        prev_col = stops[path[i]].col;
    }

    // Print all arrays
    int index;
    printf( "Command line:");
    for(int index = 0; index < step; index++){
        printf( "%c ", dir_arr[index]);

    }
    printf( "\n ");
    printf( "distance line:");
    for(int index = 0; index < step; index++){
        printf( "%.3f ", distance_arr[index]);
    }
    size=step;
    printf( "\n ");
}

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

//Sensor
float adc_voltage_to_distance(float voltage) {
   
    const float A = 22.896; 
    const float B = 0.4208; 
    const float C = 36.6;
    const float D = -0.035998;
    const float max_distance_cm = 60.0; 
    const float min_distance_cm = 0.0;
    const float min_voltage = 0.4; 

    float inverse_distance;
    float distance;

    if (voltage > min_voltage) {
        if (voltage >= 1.184){

            inverse_distance = (voltage - B) / A;

            if (inverse_distance > 0) {
                distance = 1 / inverse_distance;
                return distance <= min_distance_cm ? min_distance_cm : distance;
            } else {
                return min_distance_cm;   
            }
        }
        else{
           inverse_distance = (voltage - D) / C;

            if (inverse_distance > 0) {
                distance = 1 / inverse_distance;
                return distance >= max_distance_cm ? max_distance_cm : distance;
            } else {
                return max_distance_cm;
            }  
        }
    } else {

        return min_distance_cm;
    }
}


void core1Task(void *pvParameters) {
    uint64_t last_notification_time = 0;
    const uint64_t notification_interval = 5e6;
    float voltage;
    float distance;
    adc1_config_width(ADC_WIDTH_BIT_12);       
    adc1_config_channel_atten(ADC_CHANNEL_6, ADC_ATTEN_DB_11);
    while (1) {
        if (copy_param.start.status == ESP_SPP_SUCCESS) 
        {
            char float_str[40];
            snprintf(float_str, sizeof(float_str), "%f,%f\n", current_xy_labview[0],current_xy_labview[1]);
            esp_spp_write(copy_param.srv_open.handle, strlen(float_str), (uint8_t *)float_str);
        }
        int adc2_Val = 0;
        int adc1_Val = adc1_get_raw(ADC_CHANNEL_6);
        adc2_get_raw(ADC_CHANNEL_0,ADC_WIDTH_BIT_12,&adc2_Val);
        
        voltage = (adc1_Val * 3) / 4096.0;
        distance = adc_voltage_to_distance(voltage); 
        uint64_t current_time = esp_timer_get_time();
        printf("ADC Reading: %d, Voltage: %.2f V, Distance: %.2f cm\n", adc1_Val, voltage, distance);

        if (distance < distanceThreshold && (current_time - last_notification_time) >= notification_interval) {
            xTaskNotifyGive(Task2_Handle);
            printf("Core 1 Task: Distance below threshold, sent notification\n");
            last_notification_time = current_time; // Update the timestamp
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int64_t last_time;

void core0Task(void *param) {
    int interr_count=0;
    uint8_t buffer[12]; // Buffer to hold accelerometer and gyroscope data
    float angle_z = 0.0;
    int count = 0;
    int count2 = 0;
    int step = 0;
    float initial_angle = 0;
    float initial_distance = 0;
    // char *dir_arr[] = {'F','S'};
    // float distance_arr[5] = {1.0, 0.0};
    // size_t size = sizeof(dir_arr) / sizeof(char);

    int readenc, lastread = 0;
    float Distance = 0;
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ENCODER);
    gpio_set_direction(ENCODER, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER, GPIO_PULLUP_ONLY);
    while (1) {
        if (ulTaskNotifyTake(pdTRUE,0) == 0) {
            interr_count=0;
            // Reading accelerometer data
            ESP_ERROR_CHECK(i2c_read_register(0x28, buffer, 6)); // Starting register address for accel data
            float accel_x_raw = (float)((int16_t)((buffer[1] << 8) | buffer[0]));
            float accel_y_raw = (float)((int16_t)((buffer[3] << 8) | buffer[2]));

            ESP_ERROR_CHECK(i2c_read_register(0x22, buffer, 6)); // Starting register address for gyro data
            float gyro_z_raw = (float)((int16_t)((buffer[5] << 8) | buffer[4]));

            // Convert raw data to m/s^2
            float accel_x = accel_x_raw * ACCEL_SENSITIVITY * 0.001 * GRAVITY;
            float accel_y = accel_y_raw * ACCEL_SENSITIVITY * 0.001 * GRAVITY;

            // Convert raw data to dps
            float gyro_z = gyro_z_raw * GYRO_SENSITIVITY * 0.001;

            // Compute delta t
            int64_t current_time = esp_timer_get_time();
            float dt = (current_time - last_time)*1e-6;
            last_time = current_time;

            float sumvelocity =0;
            float meanvelocity;
            int sumcount = 0;

            if (sumcount <=99){
                sumcount ++;
                sumvelocity += (accel_y-0.3);
                meanvelocity = sumvelocity/sumcount;
            }
            current_velocity = meanvelocity * dt + V0;
            V0 = current_velocity;
           // printf("dt: %f\n , accel_y: %f , vo: %f\n",dt,accel_y-0.3,V0);
            
            float error[4] = {0, 0, 0, 0};
            //printf("current velocity: %f\n", current_velocity);
            for (int i = 0; i < 4; i++) {
                error[i] = desired_speed[i] - current_velocity;
                //printf("desired speed 2: %f\n", desired_speed[2]);
                //printf("error2: %f\n", error[2]);
                u[i] = kp * error[i];
        
                if (u[i] > saturation) {
                    u[i] = saturation;
                }
            }

            if (abs(gyro_z)>=1e-5){
                angle_z += gyro_z * dt;
            }

            if (step<size){
                if ((dir_arr[step]=='R') | (dir_arr[step]=='L')) {
                    struct state current_state = turn(step,initial_angle,angle_z,dir_arr[step]);
                    step = current_state.step;
                    initial_angle = current_state.initial_angle;
                } else if (dir_arr[step]=='F') {
                    // int64_t start = esp_timer_get_time();
                    // while(esp_timer_get_time()-start<1e6){
                    //     move_forward(200,200);
                    // }
                    // step+=1;
                    readenc = gpio_get_level(ENCODER);
                    if (lastread != readenc){
                        // gpio_set_level(LED, true);
                        count2++;
                        printf("count:");
                        printf("%d \n",count2);
                        Distance = (count2 * 66.74 * 3.1415) / 40.0 /1000; //m
                        printf("Distance:");
                        printf("%f \n",Distance);
                    }
                    lastread = readenc;
                    struct state current_state = straight(step,initial_distance,Distance,distance_arr[step]);
                    step = current_state.step;
                    initial_distance = current_state.initial_angle;
                } else {
                    stop();
                    step += 1;
                }
            }

            if (step==size) {
                stop();
                step += 1;
            }

            if (count%200 == 0){
                ESP_LOGI(TAG, "\nangle");
                ESP_LOGI(TAG, " Z = %f degree", angle_z);
            }
            count++;

            vTaskDelay(pdMS_TO_TICKS(10));  // Delay for 1 second
        } else {
            printf("Notification received from Task1!\n");
            stop();
            if (dir_arr[step]=='F') {
                if (interr_count!=1) {
                    current_xy[0]+=absdiff2 * facing_direction[0] /stepsize;
                    current_xy[1]+=absdiff2 * facing_direction[1] /stepsize;
                }
            }
            if (interr_count!=1) {
                s = round(current_xy[1]) * 8 + round(current_xy[0]);
                int obstacle_x = current_xy[0] + facing_direction[0];
                int obstacle_y = current_xy[1] + facing_direction[1];
                if ((obstacle_x>=0)&(obstacle_x<=9)&(obstacle_y>=0)&(obstacle_y<=9)) {
                    map[obstacle_y][obstacle_x]=1;
                }
                memset(ind, -1, sizeof(ind));
                runastar();
                step = 0;
                while (step<1) {
                    ESP_ERROR_CHECK(i2c_read_register(0x22, buffer, 6)); // Starting register address for gyro data
                    float gyro_z_raw = (float)((int16_t)((buffer[5] << 8) | buffer[4]));
                    float gyro_z = gyro_z_raw * GYRO_SENSITIVITY * 0.001;
                    // Compute delta t
                    int64_t current_time = esp_timer_get_time();
                    float dt = (current_time - last_time)*1e-6;
                    last_time = current_time;
                    if (abs(gyro_z)>=1e-5){
                        angle_z += gyro_z * dt;
                    }
                    struct state current_state;
                    if (fabs(facing_direction[0]-1)>1e-6) {
                        if (fabs(facing_direction[1]-1)<1e-6) {
                            current_state = turn(step,initial_angle,90,'L');
                        } else if (fabs(facing_direction[1]+1)<1e-6) {
                            current_state = turn(step,initial_angle,90,'R');
                        } else {
                            current_state = turn(step,initial_angle,180,'R');
                        }
                        step = current_state.step;
                        initial_angle = current_state.initial_angle;
                    } else {
                        step += 1;
                    }
                }
            }
            interr_count=1;
            step=0;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%"PRIu32,
                 param->data_ind.len, param->data_ind.handle);
        // if (param->data_ind.len < 128) {
        //     esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
        // }
        printf("Value received: ");
        for (size_t i=0; i<param->data_ind.len; i++)
        {
            char value = param->data_ind.data[i];
            printf("%c",value);
        }
        printf("\n");
        // esp_task_wdt_init(10);
        memcpy(&copy_param, param, sizeof(esp_spp_cb_param_t));
        if (spp_server_handle == NULL) {
            pin_def();
            sensor_setup();
            runastar(); // Generate astar graph
            last_time = esp_timer_get_time();
            xTaskCreatePinnedToCore(core1Task, "Task 1", 4096, NULL, 1, NULL, 1);
            xTaskCreatePinnedToCore(core0Task, "Task 0", 4096, NULL, 1, &Task2_Handle, 0);
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32", rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void app_main(void)
{
    char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}

void timer_callback(void *param){
    output_PWM[0] = (desired_speed[0]/max_speed)*max_PWM * u[0];
    output_PWM[1] = (desired_speed[1]/max_speed)*max_PWM * u[1];
    output_PWM[2] = (desired_speed[2]/max_speed)*max_PWM * u[2];
    output_PWM[3] = (desired_speed[3]/max_speed)*max_PWM * u[3];
}
