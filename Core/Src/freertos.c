/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <example_interfaces/srv/add_two_ints.h>

#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//osThreadId_t publisherTaskHandle;
//uint32_t publisherTaskBuffer[ 500 ];
//osStaticThreadDef_t publisherTaskControlBlock;
//const osThreadAttr_t publisherTask_attributes = {
//  .name = "publisherTask",
//  .cb_mem = &publisherTaskControlBlock,
//  .cb_size = sizeof(publisherTaskControlBlock),
//  .stack_mem = &publisherTaskBuffer[0],
//  .stack_size = sizeof(publisherTaskBuffer),
//  .priority = (osPriority_t) osPriorityNormal,
//};
//
//osThreadId_t another_publisherTaskHandle;
//uint32_t another_publisherTaskBuffer[ 500 ];
//osStaticThreadDef_t another_publisherTaskControlBlock;
//const osThreadAttr_t another_publisherTask_attributes = {
//  .name = "another_publisherTask",
//  .cb_mem = &another_publisherTaskControlBlock,
//  .cb_size = sizeof(another_publisherTaskControlBlock),
//  .stack_mem = &another_publisherTaskBuffer[0],
//  .stack_size = sizeof(another_publisherTaskBuffer),
//  .priority = (osPriority_t) osPriorityNormal,
//};

//BaseType_t xReturned;
//TaskHandle_t xHandle = NULL;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 500 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void StartPublisherTask(void *argument);
void StartAnotherPublisherTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  publisherTaskHandle = osThreadNew(StartPublisherTask, NULL, &publisherTask_attributes);
//  another_publisherTaskHandle = osThreadNew(StartAnotherPublisherTask, NULL, &another_publisherTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */


  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart3,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // micro-ROS app

  std_msgs__msg__Int32 msg;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rcl_init_options_t init_options; // object for custom settings(DOMAIN ID = 1)
  rclc_executor_t executor;  //executor for service
  rcl_service_t service;  //custom service

  // Service message objects
  example_interfaces__srv__AddTwoInts_Response response_msg;
  example_interfaces__srv__AddTwoInts_Request request_msg;


  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 1);

  //create init_options with custom settings
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // create node
  rclc_node_init_default(&node, "cubemx_node_service", "", &support);


  executor = rclc_executor_get_zero_initialized_executor();
  unsigned int num_handles = 1;  //handle only one service
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);

  rclc_service_init_default(
    &service,
	&node,
	ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts),
	"/addtwoints");

  // Function prototype:
  void (* rclc_service_callback_t)(const void *, void *);

  // Implementation service callback:
  void service_callback(const void * request_msg, void * response_msg){
    // Cast messages to expected types
    example_interfaces__srv__AddTwoInts_Request * req_in =
      (example_interfaces__srv__AddTwoInts_Request *) request_msg;
    example_interfaces__srv__AddTwoInts_Response * res_in =
      (example_interfaces__srv__AddTwoInts_Response *) response_msg;

    // Handle request message and set the response message values
    printf("Client requested sum of %d and %d.\n", (int) req_in->a, (int) req_in->b);
    res_in->sum = req_in->a + req_in->b;
  }

  // Add server callback to the executor
  rclc_executor_add_service(
    &executor,
	&service,
	&request_msg,
    &response_msg,
	service_callback);


//  //starting executor
  rclc_executor_spin(&executor);

  while(1){

  }

  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//void StartAnotherPublisherTask(void *argument)
//{
//	  // micro-ROS app
//
//	  std_msgs__msg__Int32 msg2;
//	  rclc_support_t support2;
//	  rcl_allocator_t allocator2;
//	  rcl_node_t node2;
//	  rcl_init_options_t init_options2; // object for custom settings(DOMAIN ID = 1)
//	  rcl_publisher_t publisher2;
//
//
//	  allocator2 = rcl_get_default_allocator();
//
//	  init_options2 = rcl_get_zero_initialized_init_options();
//	  rcl_init_options_init(&init_options2, allocator2);
//	  rcl_init_options_set_domain_id(&init_options2, 1);
//
//	  //create init_options with custom settings
//	  rclc_support_init_with_options(&support2, 0, NULL, &init_options2, &allocator2);
//
//	  // create node
//	  rclc_node_init_default(&node2, "cubemx_node_another_publisher", "", &support2);
//
//	  // create publisher
//	  rclc_publisher_init_default(
//	    &publisher2,
//	    &node2,
//	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//	    "cubemx_another_publisher");
//
//	msg2.data = 0;
//	while(1)
//	  {
//	    rcl_ret_t ret;
//
//	    ret = rcl_publish(&publisher2, &msg2, NULL);
//	    if (ret != RCL_RET_OK)
//	    {
//	      printf("Error publishing (line %d)\n", __LINE__);
//	    }
//
//	    msg2.data++;
//	    osDelay(10);
//	  }
//}
//
//void StartPublisherTask(void *argument)
//{
//	  // micro-ROS app
//
//	  std_msgs__msg__Int32 msg;
////	  std_msgs__msg__Int32 msg2;
//	  rclc_support_t support;
//	  rcl_allocator_t allocator;
//	  rcl_node_t node1;
////	  rcl_node_t node2;
//	  rcl_init_options_t init_options; // object for custom settings(DOMAIN ID = 1)
//	  rcl_publisher_t publisher1;
////	  rcl_publisher_t publisher2;
//
//
//	  allocator = rcl_get_default_allocator();
//
//	  init_options = rcl_get_zero_initialized_init_options();
//	  rcl_init_options_init(&init_options, allocator);
//	  rcl_init_options_set_domain_id(&init_options, 1);
//
//	  //create init_options with custom settings
//	  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
//
//	  // create node
//	  rclc_node_init_default(&node1, "cubemx_node_publisher", "", &support);
//
//	  // create publisher
//	  rclc_publisher_init_default(
//	    &publisher1,
//	    &node1,
//	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//	    "cubemx_publisher");
////
////
////	  // create node
////	  rclc_node_init_default(&node2, "cubemx_node_another_publisher", "", &support);
////
////	  // create publisher
////	  rclc_publisher_init_default(
////	    &publisher2,
////	    &node2,
////	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
////	    "cubemx_another_publisher");
//
//
//	msg.data = 0;
//	while(1)
//	  {
//	    rcl_ret_t ret;
////	    msg2.data = 2;
//
//	    ret = rcl_publish(&publisher1, &msg, NULL);
////	    ret = rcl_publish(&publisher2, &msg2, NULL);
//	    if (ret != RCL_RET_OK)
//	    {
//	      printf("Error publishing (line %d)\n", __LINE__);
//	    }
//
//	    msg.data++;
//	    osDelay(10);
//	  }
//}
/* USER CODE END Application */
