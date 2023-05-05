// Copyright (c) 2020 - for handle_countersrmation on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/ros2/rclc.
// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclc/executor_handle.h"

#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>

/**
 * @brief 初始化 handle_counters 对象 (Initialize the handle_counters object)
 *
 * @param[out] handle_counters 指向 rclc_executor_handle_counters_t 结构体的指针 (Pointer to
 * rclc_executor_handle_counters_t structure)
 * @return 返回 RCL_RET_OK 或者 RCL_RET_INVALID_ARGUMENT (Returns RCL_RET_OK or
 * RCL_RET_INVALID_ARGUMENT)
 */
rcl_ret_t rclc_executor_handle_counters_zero_init(
    rclc_executor_handle_counters_t* handle_counters) {
  // 检查 handle_counters 是否为空指针 (Check if handle_counters is a NULL pointer)
  RCL_CHECK_ARGUMENT_FOR_NULL(handle_counters, RCL_RET_INVALID_ARGUMENT);

  // 使用 memset 函数将 handle_counters 结构体的内存设置为零 (Use memset function to set the memory
  // of handle_counters structure to zero)
  memset(handle_counters, 0, sizeof(rclc_executor_handle_counters_t));

  // 返回 RCL_RET_OK 表示成功 (Return RCL_RET_OK for success)
  return RCL_RET_OK;
}

/**
 * @brief 初始化 handle 对象 (Initialize the handle object)
 *
 * @param[out] handle 指向 rclc_executor_handle_t 结构体的指针 (Pointer to rclc_executor_handle_t
 * structure)
 * @param[in] max_handles 最大句柄数 (Maximum number of handles)
 * @return 返回 RCL_RET_OK 或者 RCL_RET_INVALID_ARGUMENT (Returns RCL_RET_OK or
 * RCL_RET_INVALID_ARGUMENT)
 */
rcl_ret_t rclc_executor_handle_init(rclc_executor_handle_t* handle, size_t max_handles) {
  // 检查 handle 是否为空指针 (Check if handle is a NULL pointer)
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);

  // 初始化 handle 的类型为 RCLC_NONE (Initialize the type of handle to RCLC_NONE)
  handle->type = RCLC_NONE;

  // 初始化 handle 的调用方式为 ON_NEW_DATA (Initialize the invocation of handle to ON_NEW_DATA)
  handle->invocation = ON_NEW_DATA;

  // 注意，指向 subscription, timer, service, client, gc 的指针是一个联合体
  // 单个 NULL 赋值应该就足够了。
  // (Note, the pointer to subscription, timer, service, client, gc is a union
  // and a single NULL assignment should be sufficient.)
  handle->subscription = NULL;
  handle->timer = NULL;
  handle->service = NULL;
  handle->client = NULL;
  handle->gc = NULL;

  // 初始化其他成员变量为 NULL 或者 false (Initialize other member variables to NULL or false)
  handle->data = NULL;
  handle->data_response_msg = NULL;
  handle->callback_context = NULL;

  handle->subscription_callback = NULL;

  // 因为联合结构:
  //   handle->service_callback == NULL;
  //   handle->client_callback == NULL;
  //   handle->gc_callback == NULL
  //   ...
  // (because of union structure)

  // 设置 handle 的索引和初始化标志 (Set the index and initialization flag of handle)
  handle->index = max_handles;
  handle->initialized = false;
  handle->data_available = false;

  // 返回 RCL_RET_OK 表示成功 (Return RCL_RET_OK for success)
  return RCL_RET_OK;
}

/**
 * @brief 清除 handle 对象 (Clear the handle object)
 *
 * @param[out] handle 指向 rclc_executor_handle_t 结构体的指针 (Pointer to rclc_executor_handle_t
 * structure)
 * @param[in] max_handles 最大句柄数 (Maximum number of handles)
 * @return 返回 RCL_RET_OK 或者 RCL_RET_INVALID_ARGUMENT (Returns RCL_RET_OK or
 * RCL_RET_INVALID_ARGUMENT)
 */
rcl_ret_t rclc_executor_handle_clear(rclc_executor_handle_t* handle, size_t max_handles) {
  // 检查 handle 是否为空指针 (Check if handle is a NULL pointer)
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);

  // 设置 handle 的索引和初始化标志 (Set the index and initialization flag of handle)
  handle->index = max_handles;
  handle->initialized = false;

  // 返回 RCL_RET_OK 表示成功 (Return RCL_RET_OK for success)
  return RCL_RET_OK;
}

/**
 * @brief 打印 rclc_executor_handle_t 结构体的类型名称 (Print the type name of the
 * rclc_executor_handle_t structure)
 *
 * @param handle 指向 rclc_executor_handle_t 结构体的指针 (Pointer to the rclc_executor_handle_t
 * structure)
 * @return rcl_ret_t 返回 RCL_RET_OK 或 RCL_RET_INVALID_ARGUMENT (Returns RCL_RET_OK or
 * RCL_RET_INVALID_ARGUMENT)
 */
rcl_ret_t rclc_executor_handle_print(rclc_executor_handle_t* handle) {
  // 检查 handle 参数是否为 NULL，如果是，则返回 RCL_RET_INVALID_ARGUMENT 错误
  // (Check if the handle parameter is NULL, and if so, return the RCL_RET_INVALID_ARGUMENT error)
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);

  // 定义一个 char 类型的指针 typeName，用于存储类型名称
  // (Define a char pointer typeName to store the type name)
  char* typeName;

  // 使用 switch 语句根据 handle 的类型设置 typeName
  // (Use a switch statement to set typeName based on the type of handle)
  switch (handle->type) {
    case RCLC_NONE:
      typeName = "None";
      break;
    case RCLC_SUBSCRIPTION:
    case RCLC_SUBSCRIPTION_WITH_CONTEXT:
      typeName = "Sub";
      break;
    case RCLC_TIMER:
      // case RCLC_TIMER_WITH_CONTEXT:
      typeName = "Timer";
      break;
    case RCLC_CLIENT:
    case RCLC_CLIENT_WITH_REQUEST_ID:
      // case RCLC_CLIENT_WITH_CONTEXT:
      typeName = "Client";
      break;
    case RCLC_SERVICE:
    case RCLC_SERVICE_WITH_REQUEST_ID:
    case RCLC_SERVICE_WITH_CONTEXT:
      typeName = "Service";
      break;
    case RCLC_GUARD_CONDITION:
      // case RCLC_GUARD_CONDITION_WITH_CONTEXT:
      typeName = "GuardCondition";
      break;
    default:
      typeName = "Unknown";
  }
  // 使用 RCUTILS_LOG_DEBUG_NAMED 函数打印 typeName
  // (Print typeName using the RCUTILS_LOG_DEBUG_NAMED function)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "  %s\n", typeName);
  // 返回 RCL_RET_OK
  // (Return RCL_RET_OK)
  return RCL_RET_OK;
}

/**
 * @brief 获取 rclc_executor_handle_t 结构体中的指针 (Get the pointer in the rclc_executor_handle_t
 * structure)
 *
 * @param[in] handle rclc_executor_handle_t 结构体指针 (Pointer to rclc_executor_handle_t structure)
 * @return 返回指向不同类型实体的指针，如果传入的 handle 为空或者类型未知，则返回 NULL (Returns a
 * pointer to different types of entities, or NULL if the input handle is empty or the type is
 * unknown)
 */
void* rclc_executor_handle_get_ptr(rclc_executor_handle_t* handle) {
  // 不能使用 RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);
  // 因为它会创建一个 "return" 语句，而这里的返回类型是 (void *)
  // (Cannot be used because it creates a "return" statement and here the return type is (void *))
  if (handle == NULL) {
    return NULL;
  }

  void* ptr;
  switch (handle->type) {
    case RCLC_SUBSCRIPTION:
    case RCLC_SUBSCRIPTION_WITH_CONTEXT:
      // 如果类型是订阅器，将指针设置为订阅器 (If the type is subscription, set the pointer to the
      // subscription)
      ptr = handle->subscription;
      break;
    case RCLC_TIMER:
      // 如果类型是定时器，将指针设置为定时器 (If the type is timer, set the pointer to the timer)
      ptr = handle->timer;
      break;
    case RCLC_CLIENT:
    case RCLC_CLIENT_WITH_REQUEST_ID:
      // 如果类型是客户端，将指针设置为客户端 (If the type is client, set the pointer to the client)
      ptr = handle->client;
      break;
    case RCLC_SERVICE:
    case RCLC_SERVICE_WITH_REQUEST_ID:
    case RCLC_SERVICE_WITH_CONTEXT:
      // 如果类型是服务，将指针设置为服务 (If the type is service, set the pointer to the service)
      ptr = handle->service;
      break;
    case RCLC_GUARD_CONDITION:
      // 如果类型是 guard condition，将指针设置为 guard condition (If the type is guard condition,
      // set the pointer to the guard condition)
      ptr = handle->gc;
      break;
    case RCLC_NONE:
    default:
      // 如果类型未知，将指针设置为 NULL (If the type is unknown, set the pointer to NULL)
      ptr = NULL;
  }

  return ptr;
}
