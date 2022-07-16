#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <unistd.h>
#include <math.h>
#include <sys/_stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/portmacro.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

static const char *TAG = "robot";

#define ESP32C3_UART_PORT_NUM (1)
#define ESP32C3_UART_RXD (5)
#define ESP32C3_UART_TXD (4)
#define UART_RTS (-1)
#define UART_CTS (-1)

#define UART_BAUD_RATE (115200)
#define TASK_STACK_SIZE (2048)

#define BUF_SIZE (1024)

float Left_Velocity_KP = 170;
float Left_Velocity_KI = 10;

float Right_Velocity_KP = 170;
float Right_Velocity_KI = 10;

static int Left_Target = 0;
static int Right_Target = 0;

int Incremental_PI_Left(int encoder,int target){
  static float left_bias,left_pwm,left_last_bias;
  left_bias = target - encoder;
  left_pwm += Left_Velocity_KP * (left_bias - left_last_bias) + Left_Velocity_KI * left_bias;
  if (left_pwm > 8191) left_pwm = 8191;
  if (left_pwm < 0) left_pwm = 0;
  left_last_bias = left_bias;
  return (int)left_pwm;
}

int Incremental_PI_Right(int encoder,int target){
  static float right_bias,right_pwm,right_last_bias;
  right_bias = target - encoder;
  right_pwm += Right_Velocity_KP * (right_bias - right_last_bias) + Right_Velocity_KI * right_bias;
  if (right_pwm > 8191) right_pwm = 8191;
  if (right_pwm < 0) right_pwm = 0;
  right_last_bias = right_bias;
  return (int)right_pwm;
}

//encoder pulse io
#define ENCODER_LEFT_A (2)
#define ENCODER_LEFT_B (3)
#define ENCODER_RIGHT_A (10)
#define ENCODER_RIGHT_B (1)

#define ENCODER_A_INPUT_PIN_SEL ((1ULL<<ENCODER_LEFT_A) | (1ULL<<ENCODER_RIGHT_A))
#define ENCODER_B_INPUT_PIN_SEL ((1ULL<<ENCODER_LEFT_B) | (1ULL<<ENCODER_RIGHT_B))

#define ESP_INTR_FLAG_DEFAULT 0

//two motor pwm ctl io
#define MOTOR_ENA (8)
#define MOTOR_ENB (19)

//pwm configuration
#define PWM_TIMER_LEFT  LEDC_TIMER_0
#define PWM_TIMER_RIGHT LEDC_TIMER_1
#define PWM_CHANNEL_LEFT LEDC_CHANNEL_0
#define PWM_CHANNEL_RIGHT LEDC_CHANNEL_1

#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_DUTY_RES LEDC_TIMER_13_BIT
#define PWM_FREQUENCY (5000) //Frequentcy in Hertz. Set frequency at 5 kHz
                             //
#define PWM_DEFAULT_DUTY (4095) // set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define PWM_EMPTY_DUTY (0)
#define PWM_FULL_DUTY (8191) //set duty to 100%. ((2 ** 13) - 1)

//left motor direction ctl io
#define MOTOR_IN1 (6)
#define MOTOR_IN2 (7)
//right motor direction ctl io
#define MOTOR_IN3 (9)
#define MOTOR_IN4 (18)

//pin select
#define MOTOR_CTL_PIN_SEL ((1ULL<<MOTOR_IN1) | (1ULL<<MOTOR_IN2) | (1ULL<<MOTOR_IN3) | (1ULL<<MOTOR_IN4))

//motor left or right
typedef enum{
  MOTOR_LEFT = 0,
  MOTOR_RIGHT
}motor_t;

//motor control state
typedef enum{
  MOTOR_STOP = 0,
  MOTOR_BRAKE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD
}motor_ctl_state_t;

volatile static int16_t ec_left_count = 0;
volatile static int16_t ec_right_count = 0;

volatile static int8_t motor_left_state = -1;
volatile static int8_t motor_right_state = -1;

volatile static motor_ctl_state_t Left_Motor_State = MOTOR_STOP;
volatile static motor_ctl_state_t Right_Motor_State = MOTOR_STOP;

//A phase interrupt handler
static void IRAM_ATTR ec_left_isr_handler(void *arg){
  uint32_t gpio_num =(uint32_t)arg;
  if(gpio_num == ENCODER_LEFT_A){
    ec_left_count++;
    int b_phase_level = gpio_get_level(ENCODER_LEFT_B);
    if(b_phase_level == 1){
      motor_left_state = -1;
    }else{
      motor_left_state = 1;
    }
  }
}

static void IRAM_ATTR ec_right_isr_handler(void *arg){
  uint32_t gpio_num =(uint32_t)arg;
  if(gpio_num == ENCODER_RIGHT_A){
    ec_right_count++;
    int b_phase_level = gpio_get_level(ENCODER_RIGHT_B);
    if(b_phase_level == 1){
      motor_right_state = 1;
    }else{
      motor_right_state = -1;
    }
  }
}

void encoder_init(gpio_config_t *io_conf){
  //interrupt of rising edge
  io_conf->intr_type = GPIO_INTR_POSEDGE;
  //bit mask of the pins
  io_conf->pin_bit_mask = ENCODER_A_INPUT_PIN_SEL;
  //set as input mode
  io_conf->mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf->pull_up_en = 1;
  gpio_config(io_conf);
  //disable interrupt
  io_conf->intr_type = GPIO_INTR_DISABLE;
  //bit mask of the pins
  io_conf->pin_bit_mask = ENCODER_B_INPUT_PIN_SEL;
  //set as input mode
  io_conf->mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf->pull_up_en = 1;
  gpio_config(io_conf);

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(ENCODER_LEFT_A, ec_left_isr_handler, (void*)ENCODER_LEFT_A);
  gpio_isr_handler_add(ENCODER_RIGHT_A, ec_right_isr_handler, (void*)ENCODER_RIGHT_A);
}

int16_t get_encoder_value(motor_t motor){
  if (motor == MOTOR_LEFT){
    int16_t count = ec_left_count;
    ec_left_count = 0;
    return count;
  }else if (motor == MOTOR_RIGHT){
    int16_t count = ec_right_count;
    ec_right_count = 0;
    return count;
  }else return -1;
}


void motor_ctl_init(gpio_config_t *io_conf){
  //disable interrupt
  io_conf->intr_type =GPIO_INTR_DISABLE;
  //set as output mode
  io_conf->mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set
  io_conf->pin_bit_mask = MOTOR_CTL_PIN_SEL;
  //disable pull-down mode
  io_conf->pull_down_en = 1;
  //disable pull up mode
  io_conf->pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(io_conf);

  //pwm configuration
  ledc_timer_config_t pwm_timer_left = {
    .speed_mode = PWM_MODE,
    .timer_num = PWM_TIMER_LEFT,
    .duty_resolution = PWM_DUTY_RES,
    .freq_hz = PWM_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config_t pwm_timer_right = {
    .speed_mode = PWM_MODE,
    .timer_num = PWM_TIMER_RIGHT,
    .duty_resolution = PWM_DUTY_RES,
    .freq_hz = PWM_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };

  ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer_left));
  ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer_right));
  
  ledc_channel_config_t pwm_channel_left = {
    .speed_mode = PWM_MODE,
    .channel = PWM_CHANNEL_LEFT,
    .timer_sel = PWM_TIMER_LEFT,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = MOTOR_ENA,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config_t pwm_channel_right = {
    .speed_mode = PWM_MODE,
    .channel = PWM_CHANNEL_RIGHT,
    .timer_sel = PWM_TIMER_RIGHT,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = MOTOR_ENB,
    .duty = 0,
    .hpoint = 0
  };

  ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel_left));
  ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel_right));
}

void motor_ctl(motor_t motor,motor_ctl_state_t state,int pwm){
  if(pwm > 8191) {
    pwm = 8191;
  }

  gpio_num_t motor_ctl_io1,motor_ctl_io2; 
  ledc_channel_t motor_pwm_ctl_channel;

  if (motor) {
    motor_pwm_ctl_channel =  PWM_CHANNEL_RIGHT;
    motor_ctl_io1 = MOTOR_IN3;
    motor_ctl_io2 = MOTOR_IN4;
  }else{
    motor_pwm_ctl_channel =  PWM_CHANNEL_LEFT;
    motor_ctl_io1 = MOTOR_IN1;
    motor_ctl_io2 = MOTOR_IN2;
  }
  switch (state) {
    case MOTOR_STOP:
      {
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, motor_pwm_ctl_channel, PWM_EMPTY_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, motor_pwm_ctl_channel));
        break;
      }
    case MOTOR_BRAKE:
      {
        gpio_set_level(motor_ctl_io1, 0);
        gpio_set_level(motor_ctl_io2, 0);
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, motor_pwm_ctl_channel, PWM_FULL_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, motor_pwm_ctl_channel));
        break;
      }
    case MOTOR_FORWARD:
      {
        gpio_set_level(motor_ctl_io1, 0);
        gpio_set_level(motor_ctl_io2, 1);
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, motor_pwm_ctl_channel, pwm));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, motor_pwm_ctl_channel));
        break;
      }
    case MOTOR_BACKWARD:
      {
        gpio_set_level(motor_ctl_io1, 1);
        gpio_set_level(motor_ctl_io2, 0);
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, motor_pwm_ctl_channel, pwm));
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, motor_pwm_ctl_channel));
        break;
      }
    default:
      {
        break;
      }
  }
}

static int ticks = 0;

static QueueHandle_t encoder_evt_queue = NULL;

static void encoder_task(void *arg){
  int cnt;
  while(1){
    if(xQueueReceive(encoder_evt_queue,&cnt,portMAX_DELAY)){
      int left_encoder = get_encoder_value(MOTOR_LEFT);
      int right_encoder = get_encoder_value(MOTOR_RIGHT);
      int left_pwm = Incremental_PI_Left(left_encoder,Left_Target);
      int right_pwm = Incremental_PI_Right(right_encoder,Right_Target);
      motor_ctl(MOTOR_LEFT, Left_Motor_State, left_pwm);
      motor_ctl(MOTOR_RIGHT, Right_Motor_State, right_pwm);
      if (cnt % 50 == 0){
        if (left_encoder == 0){
              motor_left_state = 0;
        }
        if (right_encoder == 0){
              motor_right_state = 0;
        }
        ESP_LOGI(TAG, "Ticks:%d",cnt); 
        ESP_LOGI(TAG, "Left Motor Target:%d, Right Motor Target:%d", Left_Target,Right_Target);
        ESP_LOGI(TAG, "Left Motor Encoder:%d, Right Motor Encoder:%d", left_encoder,right_encoder);
        ESP_LOGI(TAG, "Left Motor State:%d, Right Motor State:%d", motor_left_state,motor_right_state);
        ESP_LOGI(TAG, "Left Motor PWM:%d, Right Motor PWM:%d",left_pwm,right_pwm);
        ESP_LOGI(TAG, "-----------------------------");
      }
    }
  }
}

static bool IRAM_ATTR encoder_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
      void *userdata){
    BaseType_t high_task_awoken = pdFALSE;
    xQueueSend(encoder_evt_queue, &ticks, portMAX_DELAY);
    ticks++;
    return (high_task_awoken == pdTRUE);
}

void encoder_timer_init(){
  ESP_LOGI(TAG,"encoder timer initialization");
  gptimer_handle_t encoder_timer = NULL;
  gptimer_config_t encoder_timer_cfg = {
    .clk_src = GPTIMER_CLK_SRC_APB,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, // 1MHz,1 tick = 1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&encoder_timer_cfg,&encoder_timer));
  gptimer_event_callbacks_t cbs = {
    .on_alarm = encoder_timer_isr,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(encoder_timer, &cbs,NULL));

  ESP_LOGI(TAG,"start encoder timer,auto reload at alarm event");
  gptimer_alarm_config_t alarm_cfg = {
    .reload_count = 0,
    .alarm_count = 20000,
    .flags.auto_reload_on_alarm = true,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(encoder_timer,&alarm_cfg));
  ESP_ERROR_CHECK(gptimer_start(encoder_timer));
}

typedef struct{
  uint8_t header;
  uint8_t command;
  uint8_t payload_len;
  uint8_t payload[16];
}control_t;

int recv_control(uart_port_t port,control_t *c){
  uint8_t header,command;
  uint8_t payload_len;
  uint8_t checksum;

  if(1 == uart_read_bytes(port,&header,1,20/portTICK_PERIOD_MS)){
        /* ESP_LOGI(TAG,"header:%x",header); */
        if(header == 0xA1){
          if (1 == uart_read_bytes(port, &command, 1, 20/portTICK_PERIOD_MS)){
              /* ESP_LOGI(TAG,"command:%x",command); */
              c->command = command;
              int len = uart_read_bytes(port, &payload_len,1, 20/portTICK_PERIOD_MS);
              if(len == 1){
                /* ESP_LOGI(TAG,"payload len:%d",payload_len); */
                if(payload_len > 0){
                  ESP_ERROR_CHECK(uart_get_buffered_data_len(port, (size_t *)&len));
                  /* ESP_LOGI(TAG,"buffer len:%d",len); */
                  if(len>=payload_len){
                    len = uart_read_bytes(port,c->payload,payload_len,20/portTICK_PERIOD_MS);
                    for(int i=0;i<payload_len;i++){
                        /* ESP_LOGI(TAG, "payload: %x", c->payload[i]); */
                      }
                    if(len == payload_len){
                      c->payload_len = payload_len;
                    }
                  }
                }else if( payload_len == 0){
                  c->payload_len = 0;
                }
               if(1 == uart_read_bytes(port,&checksum,1,20/portTICK_PERIOD_MS)){
                 /* ESP_LOGI(TAG,"checksum:%x",checksum); */
                 uint8_t sum = 0 ^ 0xA1 ^ c->command ^ c->payload_len; 
                 for(int i=0; i < c->payload_len;i++){
                   sum = sum ^ c->payload[i];
                 }
                 if(checksum == sum){
                   c->header = 0xA1;
                   /* ESP_LOGI(TAG,"check sum success"); */
                   return 1;
                 }
                 /* ESP_LOGI(TAG,"check sum fail"); */
               }
            }
       }
    } 
  }
  return  -1;
}

void getIntPayloads(int *data,uint8_t *payload,uint8_t len){
  int count = 0;
  int tmp = 0;
  for(int i=0;i<len;i++){
   count +=1; 
   if (count==1)tmp = 0xff & payload[i];
   else {
     int index = i/2;
     data[index] = tmp | (payload[i] << 8);
     tmp = 0;
     count=0;
   }
  }
}

typedef enum{
 CMD_STOP = 0x00,
 CMD_FORWARD = 0x01,
 CMD_BACKWARD = 0x02,
 CMD_BRAKE = 0x03,
 CMD_SPEED = 0x04,
}command_t;

void control_handler(control_t *c){
  switch (c->command) {
    case CMD_STOP:{
      Left_Target = 0;
      Left_Motor_State = MOTOR_STOP;
      Right_Target = 0;
      Right_Motor_State = MOTOR_STOP;
      break;
    }
    case CMD_FORWARD:
    {
      if (c->payload_len != 0){                 
        int data[2];
        getIntPayloads(data,c->payload,c->payload_len);
        Left_Target = data[0];
        Left_Motor_State = MOTOR_FORWARD;
        if (c->payload_len >= 4){
          Right_Target = data[1];
          Right_Motor_State = MOTOR_FORWARD;
        }
      }
      break;
    }
    case CMD_BACKWARD:
    {
      if (c->payload_len != 0){                   
        int data[2];
        getIntPayloads(data,c->payload,c->payload_len);
        Left_Target = data[0];
        Left_Motor_State = MOTOR_BACKWARD;
        if (c->payload_len >= 4){
          Right_Target = data[1];
          Right_Motor_State = MOTOR_BACKWARD;
        }
      }
      break;
    }
    case CMD_BRAKE:
    {
      Left_Target = 0;
      Left_Motor_State = MOTOR_BRAKE;
      Right_Target = 0;
      Right_Motor_State = MOTOR_BRAKE;
      break;
    }
    case CMD_SPEED:
    {
      if (c->payload_len != 0){
        int data[2];
        getIntPayloads(data,c->payload,c->payload_len);
        if (Left_Motor_State == MOTOR_FORWARD || Left_Motor_State == MOTOR_BACKWARD){
          Left_Target = data[0];
        }
        if (c->payload_len >= 4){
        if (Right_Motor_State == MOTOR_FORWARD || Right_Motor_State == MOTOR_BACKWARD){
          Right_Target = data[1];
        }
        }
      }
      break;
    }
    default:{
      Left_Target = 0;
      Left_Motor_State = MOTOR_STOP;
      Right_Target = 0;
      Right_Motor_State = MOTOR_STOP;
      break;
    }
  }
}

static void uart_task(void *arg){
  uart_config_t uart_config = {
    .baud_rate = UART_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
  };
  int intr_alloc_flags = 0;

  ESP_ERROR_CHECK(uart_driver_install(ESP32C3_UART_PORT_NUM,BUF_SIZE*2,0,0,NULL,intr_alloc_flags));
  ESP_ERROR_CHECK(uart_param_config(ESP32C3_UART_PORT_NUM,&uart_config));
  ESP_ERROR_CHECK(uart_set_pin(ESP32C3_UART_PORT_NUM,ESP32C3_UART_TXD,ESP32C3_UART_RXD, UART_RTS,UART_CTS));

  while(1){
    control_t ctl = {};
    recv_control(ESP32C3_UART_PORT_NUM, &ctl);
    if(ctl.header == 0xA1){
      ESP_LOGI(TAG,"Receive Command:0x%x",ctl.command);
      ESP_LOGI(TAG,"--------------------------------");
      control_handler(&ctl);
    }
  }
}

void app_main(void){
  //zero initialize io configure structure
  gpio_config_t io_conf = {};
  ESP_LOGI(TAG,"encoder initializing...");
  encoder_init(&io_conf);
  ESP_LOGI(TAG, "motor ctl io initializing");
  motor_ctl_init(&io_conf);
  encoder_evt_queue = xQueueCreate(10,sizeof(uint32_t));
  xTaskCreate(encoder_task,"encoder_counter",2048,NULL,10,NULL);
  xTaskCreate(uart_task,"uart_task",TASK_STACK_SIZE,NULL,10,NULL);
  encoder_timer_init();
  while(1){
    vTaskDelay(1);
  }
}
