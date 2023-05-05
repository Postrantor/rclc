// Copyright (c) 2020 - for information on the respective copyright owner
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

#include "rclc/executor.h"

#include <rcutils/time.h>

#include "./action_client_internal.h"
#include "./action_generic_types.h"
#include "./action_goal_handle_internal.h"
#include "./action_server_internal.h"

/**
 * @file
 * @brief 包含了在 Foxy 版本中引入的 'rcl_wait_set_is_valid' 函数的回溯版本，用于支持 Dashing 和
 * Eloquent 版本的构建。 同时定义了默认的 rcl_wait() 超时时间。 Includes the backport of the
 * 'rcl_wait_set_is_valid' function introduced in Foxy, for supporting builds in Dashing and
 * Eloquent versions. Also defines the default timeout for rcl_wait().
 */
#if defined(USE_RCL_WAIT_SET_IS_VALID_BACKPORT)
#include "rclc/rcl_wait_set_is_valid_backport.h"
#endif

/**
 * @def DEFAULT_WAIT_TIMEOUT_NS
 * @brief 定义 rcl_wait() 函数的默认超时时间，单位为纳秒。
 *        Defines the default timeout for rcl_wait() function, in nanoseconds.
 */
#define DEFAULT_WAIT_TIMEOUT_NS 1000000000

// 声明辅助函数
// declarations of helper functions
/*
/// 从 DDS 队列获取句柄 i 的新数据
/// get new data from DDS queue for handle i
static
rcl_ret_t
_rclc_check_for_new_data(rclc_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);

/// 从 DDS 队列获取句柄 i 的新数据
/// get new data from DDS queue for handle i
static
rcl_ret_t
_rclc_take_new_data(rclc_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);

/// 执行句柄 i 的回调
/// execute callback of handle i
static
rcl_ret_t
_rclc_execute(rclc_executor_t * executor, rcl_wait_set_t * wait_set, size_t i);

static
rcl_ret_t
_rclc_let_scheduling(rclc_executor_t * executor, rcl_wait_set_t * wait_set);
*/

// 基本原理：用户必须使用以下方法创建一个执行器：
// executor = rclc_executor_get_zero_initialized_executor();
// 然后 handles==NULL 或者已经正确初始化
// rationale: user must create an executor with:
// executor = rclc_executor_get_zero_initialized_executor();
// then handles==NULL or not (e.g. properly initialized)
static bool _rclc_executor_is_valid(rclc_executor_t *executor) {
  // 检查执行器指针是否为空，如果为空，则返回错误消息
  // Check if the executor pointer is null, and if so, return an error message
  RCL_CHECK_FOR_NULL_WITH_MSG(executor, "executor pointer is invalid", return false);

  // 检查句柄指针是否为空，如果为空，则返回错误消息
  // Check if the handle pointer is null, and if so, return an error message
  RCL_CHECK_FOR_NULL_WITH_MSG(executor->handles, "handle pointer is invalid", return false);

  // 检查分配器指针是否为空，如果为空，则返回错误消息
  // Check if the allocator pointer is null, and if so, return an error message
  RCL_CHECK_FOR_NULL_WITH_MSG(executor->allocator, "allocator pointer is invalid", return false);

  // 如果执行器的最大句柄数为0，则返回 false
  // If the executor's max_handles is 0, return false
  if (executor->max_handles == 0) {
    return false;
  }

  // 如果以上条件都满足，则返回 true，表示执行器有效
  // If all the above conditions are met, return true, indicating that the executor is valid
  return true;
}

/**
 * @brief 获取一个零初始化的执行器 (Get a zero-initialized executor)
 *
 * 这个函数返回一个零初始化的 rclc_executor_t 结构体实例。
 * 该实例的所有字段都被初始化为零或空指针。
 * (This function returns a zero-initialized instance of the rclc_executor_t struct.
 * All fields of the instance are initialized to zero or NULL pointers.)
 *
 * @return 返回一个零初始化的执行器 (A zero-initialized executor)
 */
rclc_executor_t rclc_executor_get_zero_initialized_executor() {
  // 定义一个静态的 null_executor 变量，用于存储零初始化的执行器结构体
  // (Define a static null_executor variable to store the zero-initialized executor struct)
  static rclc_executor_t null_executor = {
      .context = NULL,  // 上下文字段设置为空指针 (Set the context field to a NULL pointer)
      .handles = NULL,   // 句柄字段设置为空指针 (Set the handles field to a NULL pointer)
      .max_handles = 0,  // 最大句柄数字段设置为 0 (Set the max_handles field to 0)
      .index = 0,        // 索引字段设置为 0 (Set the index field to 0)
      .allocator = NULL,  // 分配器字段设置为空指针 (Set the allocator field to a NULL pointer)
      .timeout_ns = 0,  // 超时时间（纳秒）字段设置为 0 (Set the timeout_ns field to 0)
      .invocation_time = 0,  // 调用时间字段设置为 0 (Set the invocation_time field to 0)
      .trigger_function =
          NULL,  // 触发函数字段设置为空指针 (Set the trigger_function field to a NULL pointer)
      .trigger_object =
          NULL  // 触发对象字段设置为空指针 (Set the trigger_object field to a NULL pointer)
  };

  // 返回零初始化的执行器结构体实例
  // (Return the zero-initialized executor struct instance)
  return null_executor;
}

/**
 * @brief 初始化执行器 (Initialize the executor)
 *
 * @param[in] executor          执行器指针 (Pointer to the executor)
 * @param[in] context           RCL上下文 (RCL context)
 * @param[in] number_of_handles 句柄数量 (Number of handles)
 * @param[in] allocator         分配器 (Allocator)
 * @return rcl_ret_t            返回操作结果 (Return operation result)
 */
rcl_ret_t rclc_executor_init(rclc_executor_t *executor,
                             rcl_context_t *context,
                             const size_t number_of_handles,
                             const rcl_allocator_t *allocator) {
  // 检查执行器是否为空 (Check if executor is NULL)
  RCL_CHECK_FOR_NULL_WITH_MSG(executor, "executor is NULL", return RCL_RET_INVALID_ARGUMENT);
  // 检查上下文是否为空 (Check if context is NULL)
  RCL_CHECK_FOR_NULL_WITH_MSG(context, "context is NULL", return RCL_RET_INVALID_ARGUMENT);
  // 检查分配器是否为空 (Check if allocator is NULL)
  RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "allocator is NULL", return RCL_RET_INVALID_ARGUMENT);

  // 如果句柄数量为0，返回无效参数错误 (If the number of handles is 0, return an invalid argument
  // error)
  if (number_of_handles == 0) {
    RCL_SET_ERROR_MSG("number_of_handles is 0. Must be larger or equal to 1");
    return RCL_RET_INVALID_ARGUMENT;
  }

  rcl_ret_t ret = RCL_RET_OK;
  // 获取零初始化的执行器 (Get a zero-initialized executor)
  (*executor) = rclc_executor_get_zero_initialized_executor();
  // 设置执行器的上下文 (Set the executor's context)
  executor->context = context;
  // 设置执行器的最大句柄数 (Set the maximum number of handles for the executor)
  executor->max_handles = number_of_handles;
  // 初始化执行器的索引 (Initialize the executor's index)
  executor->index = 0;
  // 获取零初始化的等待集 (Get a zero-initialized wait set)
  executor->wait_set = rcl_get_zero_initialized_wait_set();
  // 设置执行器的分配器 (Set the executor's allocator)
  executor->allocator = allocator;
  // 设置执行器的超时时间 (Set the executor's timeout)
  executor->timeout_ns = DEFAULT_WAIT_TIMEOUT_NS;
  // 为数组分配内存 (Allocate memory for the array)
  executor->handles = executor->allocator->allocate(
      (number_of_handles * sizeof(rclc_executor_handle_t)), executor->allocator->state);
  // 如果句柄为空，返回内存分配错误 (If handles is NULL, return memory allocation error)
  if (NULL == executor->handles) {
    RCL_SET_ERROR_MSG("Could not allocate memory for 'handles'.");
    return RCL_RET_BAD_ALLOC;
  }

  // 初始化句柄 (Initialize handle)
  for (size_t i = 0; i < number_of_handles; i++) {
    rclc_executor_handle_init(&executor->handles[i], number_of_handles);
  }

  // 初始化句柄类型计数 (Initialize counts for handle types)
  rclc_executor_handle_counters_zero_init(&executor->info);

  // 默认触发器：trigger_any，对应于ROS2 rclcpp Executor语义 (Default trigger: trigger_any,
  // corresponding to ROS2 rclcpp Executor semantics) 开始处理任何具有新数据/或准备好的句柄 (Start
  // processing any handle with new data/or is ready)
  rclc_executor_set_trigger(executor, rclc_executor_trigger_any, NULL);

  // 默认语义 (Default semantics)
  rclc_executor_set_semantics(executor, RCLCPP_EXECUTOR);

  // 返回操作结果 (Return operation result)
  return ret;
}

/**
 * @brief 设置执行器的超时时间（Set the timeout for the executor）
 *
 * @param[in,out] executor 执行器指针（Pointer to the executor）
 * @param[in] timeout_ns 超时时间，单位为纳秒（Timeout value in nanoseconds）
 * @return rcl_ret_t 返回 RCL_RET_OK 或错误代码（Returns RCL_RET_OK or an error code）
 */
rcl_ret_t rclc_executor_set_timeout(rclc_executor_t *executor, const uint64_t timeout_ns) {
  // 检查执行器是否为空指针，如果是则返回 RCL_RET_INVALID_ARGUMENT 错误
  // (Check if the executor is a null pointer, and return RCL_RET_INVALID_ARGUMENT error if it is)
  RCL_CHECK_FOR_NULL_WITH_MSG(executor, "executor is null pointer",
                              return RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // 检查执行器是否有效（Check if the executor is valid）
  if (_rclc_executor_is_valid(executor)) {
    // 设置执行器的超时时间（Set the timeout for the executor）
    executor->timeout_ns = timeout_ns;
  } else {
    // 如果执行器未初始化，则设置错误消息并返回 RCL_RET_ERROR 错误
    // (If the executor is not initialized, set the error message and return RCL_RET_ERROR error)
    RCL_SET_ERROR_MSG("executor not initialized.");
    return RCL_RET_ERROR;
  }
  return ret;
}

/**
 * @brief 设置执行器的通信语义（Set the communication semantics for the executor）
 *
 * @param[in,out] executor 执行器指针（Pointer to the executor）
 * @param[in] semantics 通信语义（Communication semantics）
 * @return rcl_ret_t 返回 RCL_RET_OK 或错误代码（Returns RCL_RET_OK or an error code）
 */
rcl_ret_t rclc_executor_set_semantics(rclc_executor_t *executor,
                                      rclc_executor_semantics_t semantics) {
  // 检查执行器是否为空指针，如果是则返回 RCL_RET_INVALID_ARGUMENT 错误
  // (Check if the executor is a null pointer, and return RCL_RET_INVALID_ARGUMENT error if it is)
  RCL_CHECK_FOR_NULL_WITH_MSG(executor, "executor is null pointer",
                              return RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // 检查执行器是否有效（Check if the executor is valid）
  if (_rclc_executor_is_valid(executor)) {
    // 设置执行器的通信语义（Set the communication semantics for the executor）
    executor->data_comm_semantics = semantics;
  } else {
    // 如果执行器未初始化，则设置错误消息并返回 RCL_RET_ERROR 错误
    // (If the executor is not initialized, set the error message and return RCL_RET_ERROR error)
    RCL_SET_ERROR_MSG("executor not initialized.");
    return RCL_RET_ERROR;
  }
  return ret;
}

/**
 * @brief 终止执行器，释放资源（Terminate the executor and release resources）
 *
 * @param[in,out] executor 执行器指针（Pointer to the executor）
 * @return rcl_ret_t 返回 RCL_RET_OK 或错误代码（Returns RCL_RET_OK or an error code）
 */
rcl_ret_t rclc_executor_fini(rclc_executor_t *executor) {
  // 检查执行器是否有效（Check if the executor is valid）
  if (_rclc_executor_is_valid(executor)) {
    // 释放执行器句柄的内存（Release memory of the executor handles）
    executor->allocator->deallocate(executor->handles, executor->allocator->state);
    executor->handles = NULL;
    executor->max_handles = 0;
    executor->index = 0;
    // 将执行器信息计数器初始化为零（Initialize the executor information counters to zero）
    rclc_executor_handle_counters_zero_init(&executor->info);

    // 如果 wait_set 已初始化，则释放其内存
    // (Release memory of the wait_set if it has been initialized)
    // 对未初始化的 wait_set 调用此函数将失败
    // (Calling this function with an uninitialized wait_set will fail)
    if (rcl_wait_set_is_valid(&executor->wait_set)) {
      rcl_ret_t rc = rcl_wait_set_fini(&executor->wait_set);
      if (rc != RCL_RET_OK) {
        PRINT_RCLC_ERROR(rclc_executor_fini, rcl_wait_set_fini);
      }
    }
    // 设置执行器的默认超时时间（Set the default timeout for the executor）
    executor->timeout_ns = DEFAULT_WAIT_TIMEOUT_NS;
  } else {
    // 对已终止或零初始化的执行器重复调用 fini 是可以的
    // (Repeated calls to fini or calling fini on a zero-initialized executor is ok)
  }
  return RCL_RET_OK;
}

/**
 * @brief 添加订阅到执行器 (Add a subscription to the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] subscription 订阅指针 (Pointer to the subscription)
 * @param[in] msg 消息指针 (Pointer to the message)
 * @param[in] callback 回调函数 (Callback function)
 * @param[in] invocation 调用方式 (Invocation method)
 * @return rcl_ret_t 返回状态 (Return status)
 */
rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *executor,
                                         rcl_subscription_t *subscription,
                                         void *msg,
                                         rclc_subscription_callback_t callback,
                                         rclc_executor_handle_invocation_t invocation) {
  // 检查参数是否为空 (Check if arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(subscription, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 检查数组边界 (Check array bounds)
  if (executor->index >= executor->max_handles) {
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return RCL_RET_ERROR;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_SUBSCRIPTION;
  executor->handles[executor->index].subscription = subscription;
  executor->handles[executor->index].data = msg;
  executor->handles[executor->index].subscription_callback = callback;
  executor->handles[executor->index].invocation = invocation;
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;
  executor->handles[executor->index].data_available = false;

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使wait_set无效，以便在下一次spin_some()调用中更新'executor->wait_set'
  // (Invalidate wait_set so that in the next spin_some() call, the 'executor->wait_set' is updated
  // accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_subscription.");
      return ret;
    }
  }

  // 增加订阅数量 (Increase number of subscriptions)
  executor->info.number_of_subscriptions++;

  // 记录添加订阅的日志 (Log the addition of a subscription)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a subscription.");

  return ret;
}

/**
 * @brief 添加一个带有上下文的订阅器到执行器中 (Add a subscription with context to the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] subscription 订阅器指针 (Pointer to the subscription)
 * @param[out] msg 存储接收到的消息的指针 (Pointer to store the received message)
 * @param[in] callback 带有上下文的回调函数 (Callback function with context)
 * @param[in] context 回调函数的上下文 (Context of the callback function)
 * @param[in] invocation 调用方式，同步或异步 (Invocation mode, synchronous or asynchronous)
 * @return 返回操作结果 (Return the operation result)
 */
rcl_ret_t rclc_executor_add_subscription_with_context(
    rclc_executor_t *executor,
    rcl_subscription_t *subscription,
    void *msg,
    rclc_subscription_callback_with_context_t callback,
    void *context,
    rclc_executor_handle_invocation_t invocation) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(subscription, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 检查数组边界 (Check array bounds)
  if (executor->index >= executor->max_handles) {
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return RCL_RET_ERROR;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_SUBSCRIPTION_WITH_CONTEXT;
  executor->handles[executor->index].subscription = subscription;
  executor->handles[executor->index].data = msg;
  executor->handles[executor->index].subscription_callback_with_context = callback;
  executor->handles[executor->index].invocation = invocation;
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = context;

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使 wait_set 无效，以便在下一次 spin_some() 调用中
  // 根据需要更新 'executor->wait_set' (Invalidate wait_set so that in next spin_some() call the
  // 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_subscription_with_context.");
      return ret;
    }
  }

  // 增加订阅器数量 (Increase number of subscriptions)
  executor->info.number_of_subscriptions++;

  // 记录调试信息 (Log debug information)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a subscription.");

  return ret;
}

/**
 * @brief 添加定时器到执行器 (Add a timer to the executor)
 *
 * @param[in,out] executor 执行器指针 (Pointer to the executor)
 * @param[in] timer 定时器指针 (Pointer to the timer)
 * @return rcl_ret_t 返回 RCL_RET_OK 或错误代码 (Return RCL_RET_OK or error code)
 */
rcl_ret_t rclc_executor_add_timer(rclc_executor_t *executor, rcl_timer_t *timer) {
  // 初始化返回值为 RCL_RET_OK (Initialize return value as RCL_RET_OK)
  rcl_ret_t ret = RCL_RET_OK;

  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(timer, RCL_RET_INVALID_ARGUMENT);

  // 数组边界检查 (Array bound check)
  if (executor->index >= executor->max_handles) {
    // 如果超出边界，设置返回值为 RCL_RET_ERROR (If out of bounds, set return value to
    // RCL_RET_ERROR)
    rcl_ret_t ret = RCL_RET_ERROR;  // TODO(jst3si) better name : rclc_RET_BUFFER_OVERFLOW
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_TIMER;
  executor->handles[executor->index].timer = timer;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // i.e. when timer elapsed
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;
  executor->handles[executor->index].data_available = false;

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使 wait_set 无效，以便在下一个 spin_some() 调用中更新 'executor->wait_set' (Invalidate wait_set
  // so that in next spin_some() call the 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_timer function.");
      return ret;
    }
  }
  // 增加定时器数量 (Increase number of timers)
  executor->info.number_of_timers++;
  // 记录调试信息 (Log debug information)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a timer.");
  return ret;
}

/**
 * @brief 添加一个客户端到执行器中 (Add a client to the executor)
 *
 * @param[in,out] executor 执行器指针 (Pointer to the executor)
 * @param[in] client 客户端指针 (Pointer to the client)
 * @param[out] response_msg 响应消息指针 (Pointer to the response message)
 * @param[in] callback 客户端回调函数 (Client callback function)
 * @return rcl_ret_t 返回 RCL_RET_OK 或错误代码 (Return RCL_RET_OK or an error code)
 */
rcl_ret_t rclc_executor_add_client(rclc_executor_t *executor,
                                   rcl_client_t *client,
                                   void *response_msg,
                                   rclc_client_callback_t callback) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(client, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 数组边界检查 (Array bound check)
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_CLIENT;
  executor->handles[executor->index].client = client;
  executor->handles[executor->index].data = response_msg;
  executor->handles[executor->index].client_callback = callback;
  executor->handles[executor->index].invocation =
      ON_NEW_DATA;  // 即当请求进入时 (i.e. when request came in)
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使 wait_set 无效，以便在下一个 spin_some() 调用中更新 'executor->wait_set' (Invalidate wait_set
  // so that in next spin_some() call the 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_client function.");
      return ret;
    }
  }

  // 增加客户端数量 (Increase number of clients)
  executor->info.number_of_clients++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a client.");

  return ret;
}

/**
 * @brief 添加带有请求ID的客户端到执行器 (Add a client with request ID to the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] client 客户端指针 (Pointer to the client)
 * @param[out] response_msg 响应消息指针 (Pointer to the response message)
 * @param[in] callback 带有请求ID的回调函数 (Callback function with request ID)
 * @return rcl_ret_t 返回RCL_RET_OK，如果成功添加客户端；否则返回相应的错误代码 (Return RCL_RET_OK
 * if the client is added successfully, otherwise return the corresponding error code)
 */
rcl_ret_t rclc_executor_add_client_with_request_id(
    rclc_executor_t *executor,
    rcl_client_t *client,
    void *response_msg,
    rclc_client_callback_with_request_id_t callback) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(client, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 检查数组边界 (Check array boundary)
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_CLIENT_WITH_REQUEST_ID;
  executor->handles[executor->index].client = client;
  executor->handles[executor->index].data = response_msg;
  executor->handles[executor->index].client_callback_with_reqid = callback;
  executor->handles[executor->index].invocation =
      ON_NEW_DATA;  // 即请求进入时 (i.e. when request came in)
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使wait_set无效，以便在下一个spin_some()调用中更新'executor->wait_set' (Invalidate wait_set so
  // that in next spin_some() call the 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_client function.");
      return ret;
    }
  }

  // 增加客户端数量 (Increase number of clients)
  executor->info.number_of_clients++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a client.");
  return ret;
}

/**
 * @brief 添加服务到执行器中 (Add a service to the executor)
 *
 * @param[in,out] executor 执行器指针 (Pointer to the executor)
 * @param[in] service 服务指针 (Pointer to the service)
 * @param[in] request_msg 请求消息指针 (Pointer to the request message)
 * @param[in] response_msg 响应消息指针 (Pointer to the response message)
 * @param[in] callback 服务回调函数 (Service callback function)
 * @return rcl_ret_t 返回RCL_RET_OK表示成功，其他值表示失败 (Returns RCL_RET_OK if successful, other
 * values indicate failure)
 */
rcl_ret_t rclc_executor_add_service(rclc_executor_t *executor,
                                    rcl_service_t *service,
                                    void *request_msg,
                                    void *response_msg,
                                    rclc_service_callback_t callback) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(request_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 检查数组边界 (Check array bounds)
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_SERVICE;
  executor->handles[executor->index].service = service;
  executor->handles[executor->index].data = request_msg;
  // TODO(jst3si) new type with req and resp message in data field.
  executor->handles[executor->index].data_response_msg = response_msg;
  executor->handles[executor->index].service_callback = callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // invoce when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // 增加处理数组的索引 (Increase index of handle array)
  executor->index++;

  // 使wait_set无效，以便在下一个spin_some()调用中更新'executor->wait_set' (Invalidate wait_set so
  // that in next spin_some() call the 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_service function.");
      return ret;
    }
  }

  // 增加服务数量 (Increase number of services)
  executor->info.number_of_services++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a service.");
  return ret;
}

/**
 * @brief 添加带有上下文的服务到执行器 (Add a service with context to the executor)
 *
 * @param[in,out] executor 执行器指针 (Pointer to the executor)
 * @param[in] service 服务指针 (Pointer to the service)
 * @param[out] request_msg 请求消息指针 (Pointer to the request message)
 * @param[out] response_msg 响应消息指针 (Pointer to the response message)
 * @param[in] callback 回调函数指针，带有上下文 (Callback function pointer with context)
 * @param[in] context 上下文指针 (Pointer to the context)
 * @return rcl_ret_t 返回 RCL_RET_OK 或错误代码 (Return RCL_RET_OK or error code)
 */
rcl_ret_t rclc_executor_add_service_with_context(rclc_executor_t *executor,
                                                 rcl_service_t *service,
                                                 void *request_msg,
                                                 void *response_msg,
                                                 rclc_service_callback_with_context_t callback,
                                                 void *context) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(request_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 数组边界检查 (Array bound check)
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_SERVICE_WITH_CONTEXT;
  executor->handles[executor->index].service = service;
  executor->handles[executor->index].data = request_msg;
  // TODO(jst3si) new type with req and resp message in data field.
  executor->handles[executor->index].data_response_msg = response_msg;
  executor->handles[executor->index].service_callback_with_context = callback;
  executor->handles[executor->index].invocation =
      ON_NEW_DATA;  // 当请求到来时调用 (Invoke when request comes in)
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = context;

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使 wait_set 无效，以便在下一次 spin_some() 调用中更新 'executor->wait_set' (Invalidate wait_set
  // so that in next spin_some() call the 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_service function.");
      return ret;
    }
  }

  // 增加服务数量 (Increase number of services)
  executor->info.number_of_services++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a service.");
  return ret;
}

/**
 * @brief 添加带请求ID的服务到执行器 (Add a service with request ID to the executor)
 *
 * @param[in,out] executor 执行器指针 (Pointer to the executor)
 * @param[in] service 服务指针 (Pointer to the service)
 * @param[in] request_msg 请求消息指针 (Pointer to the request message)
 * @param[in] response_msg 响应消息指针 (Pointer to the response message)
 * @param[in] callback 回调函数指针，带有请求ID (Callback function pointer with request ID)
 * @return rcl_ret_t 返回RCL_RET_OK表示成功，其他值表示失败 (Return RCL_RET_OK for success, other
 * values for failure)
 */
rcl_ret_t rclc_executor_add_service_with_request_id(
    rclc_executor_t *executor,
    rcl_service_t *service,
    void *request_msg,
    void *response_msg,
    rclc_service_callback_with_request_id_t callback) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(service, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(request_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(response_msg, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 数组边界检查 (Array bound check)
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_SERVICE_WITH_REQUEST_ID;
  executor->handles[executor->index].service = service;
  executor->handles[executor->index].data = request_msg;
  // TODO(jst3si) new type with req and resp message in data field.
  executor->handles[executor->index].data_response_msg = response_msg;
  executor->handles[executor->index].service_callback_with_reqid = callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // invoce when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = NULL;

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使wait_set无效，以便在下一个spin_some()调用中更新'executor->wait_set'
  // (Invalidate wait_set so that in next spin_some() call the 'executor->wait_set' is updated
  // accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_service function.");
      return ret;
    }
  }

  // 增加服务数量 (Increase number of services)
  executor->info.number_of_services++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a service.");

  return ret;
}

/**
 * @brief 添加一个守护条件到执行器 (Add a guard condition to the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] gc 守护条件指针 (Pointer to the guard condition)
 * @param[in] callback 回调函数指针 (Pointer to the callback function)
 * @return rcl_ret_t 返回结果 (Return result)
 */
rcl_ret_t rclc_executor_add_guard_condition(rclc_executor_t *executor,
                                            rcl_guard_condition_t *gc,
                                            rclc_gc_callback_t callback) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);  // 检查执行器 (Check executor)
  RCL_CHECK_ARGUMENT_FOR_NULL(gc,
                              RCL_RET_INVALID_ARGUMENT);  // 检查守护条件 (Check guard condition)
  RCL_CHECK_ARGUMENT_FOR_NULL(callback, RCL_RET_INVALID_ARGUMENT);  // 检查回调函数 (Check callback)

  rcl_ret_t ret = RCL_RET_OK;

  // 检查数组边界 (Check array bounds)
  if (executor->index >= executor->max_handles) {
    ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type =
      RCLC_GUARD_CONDITION;  // 设置类型为守护条件 (Set type to guard condition)
  executor->handles[executor->index].gc = gc;  // 设置守护条件 (Set guard condition)
  executor->handles[executor->index].gc_callback = callback;  // 设置回调函数 (Set callback)
  executor->handles[executor->index].invocation =
      ON_NEW_DATA;  // 当请求到达时调用 (Invoke when request arrives)
  executor->handles[executor->index].initialized = true;  // 标记为已初始化 (Mark as initialized)
  executor->handles[executor->index].callback_context =
      NULL;  // 设置回调上下文为空 (Set callback context to NULL)

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使 wait_set 无效，以便在下一次 spin_some() 调用中更新 'executor->wait_set' (Invalidate wait_set
  // so that in the next spin_some() call, the 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_guard_condition function.");
      return ret;
    }
  }

  // 增加守护条件的数量 (Increase the number of guard conditions)
  executor->info.number_of_guard_conditions++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added a guard_condition.");
  return ret;
}

/**
 * @brief 删除执行器中的一个句柄 (Remove a handle from the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] handle 要删除的句柄指针 (Pointer to the handle to be removed)
 * @return rcl_ret_t 返回操作结果 (Return operation result)
 */
static rcl_ret_t _rclc_executor_remove_handle(rclc_executor_t *executor,
                                              rclc_executor_handle_t *handle) {
  // 检查执行器参数是否为空 (Check if the executor argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 如果找不到句柄，_rclc_executor_find_handle 将返回 NULL (NULL will be returned by
  // _rclc_executor_find_handle if the handle is not found)
  if (NULL == handle) {
    RCL_SET_ERROR_MSG("handle not found in rclc_executor_remove_handle");
    return RCL_RET_ERROR;
  }

  // 检查句柄是否超出范围 (Check if the handle is out of bounds)
  if ((handle >= &executor->handles[executor->index]) || (handle < executor->handles)) {
    RCL_SET_ERROR_MSG("Handle out of bounds");
    return RCL_RET_ERROR;
  }
  // 检查是否有句柄可删除 (Check if there are any handles to remove)
  if (0 == executor->index) {
    RCL_SET_ERROR_MSG("No handles to remove");
    return RCL_RET_ERROR;
  }

  // 缩短句柄列表，同时保持剩余句柄的顺序不变 (shorten the list of handles without changing the
  // order of remaining handles)
  executor->index--;
  for (rclc_executor_handle_t *handle_dest = handle;
       handle_dest < &executor->handles[executor->index]; handle_dest++) {
    *handle_dest = *(handle_dest + 1);
  }
  ret = rclc_executor_handle_init(&executor->handles[executor->index], executor->max_handles);

  // 强制刷新等待集 (Force a refresh of the wait set)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in _rclc_executor_remove_handle.");
      return ret;
    }
  }

  // 记录已删除句柄的调试信息 (Log debug information about the removed handle)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a handle.");
  return ret;
}

/**
 * @brief 查找与给定的 rcl_handle 相关的执行器句柄 (Find the executor handle associated with the
 * given rcl_handle)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] rcl_handle 要查找的 rcl 句柄 (The rcl handle to find)
 * @return 如果找到，则返回与 rcl_handle 关联的执行器句柄；否则，返回 NULL (Returns the executor
 * handle associated with the rcl_handle if found, otherwise returns NULL)
 */
static rclc_executor_handle_t *_rclc_executor_find_handle(rclc_executor_t *executor,
                                                          const void *rcl_handle) {
  // 遍历执行器中的所有句柄 (Iterate through all handles in the executor)
  for (rclc_executor_handle_t *test_handle = executor->handles;
       test_handle < &executor->handles[executor->index]; test_handle++) {
    // 检查当前句柄是否与给定的 rcl_handle 匹配 (Check if the current handle matches the given
    // rcl_handle)
    if (rcl_handle == rclc_executor_handle_get_ptr(test_handle)) {
      // 如果匹配，则返回该句柄 (If it matches, return the handle)
      return test_handle;
    }
  }
  // 如果未找到匹配项，则返回 NULL (Return NULL if no match is found)
  return NULL;
}

/**
 * @brief 从执行器中移除订阅 (Remove a subscription from the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] subscription 要移除的订阅指针 (Pointer to the subscription to remove)
 * @return 成功时返回 RCL_RET_OK，否则返回相应的错误代码 (Returns RCL_RET_OK on success, otherwise
 * returns the corresponding error code)
 */
rcl_ret_t rclc_executor_remove_subscription(rclc_executor_t *executor,
                                            const rcl_subscription_t *subscription) {
  // 检查 executor 和 subscription 参数是否为 NULL (Check if the executor and subscription arguments
  // are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(subscription, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 查找与给定订阅关联的执行器句柄 (Find the executor handle associated with the given
  // subscription)
  rclc_executor_handle_t *handle = _rclc_executor_find_handle(executor, subscription);

  // 从执行器中移除找到的句柄 (Remove the found handle from the executor)
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    // 如果移除失败，则设置错误消息 (Set an error message if removal fails)
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_subscription.");
    return ret;
  }

  // 更新执行器中订阅的数量 (Update the number of subscriptions in the executor)
  executor->info.number_of_subscriptions--;

  // 记录已移除订阅的调试日志 (Log a debug message that the subscription has been removed)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a subscription.");

  return ret;
}

/**
 * @brief 删除执行器中的定时器 (Remove a timer from the executor)
 *
 * @param[in] executor 指向 rclc_executor_t 结构体的指针 (Pointer to an rclc_executor_t structure)
 * @param[in] timer 指向 rcl_timer_t 结构体的指针 (Pointer to an rcl_timer_t structure)
 * @return 返回操作结果，成功返回 RCL_RET_OK (Return the result of the operation, success returns
 * RCL_RET_OK)
 */
rcl_ret_t rclc_executor_remove_timer(rclc_executor_t *executor, const rcl_timer_t *timer) {
  // 检查 executor 参数是否为空 (Check if the executor argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  // 检查 timer 参数是否为空 (Check if the timer argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(timer, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 查找与给定 timer 相关联的句柄 (Find the handle associated with the given timer)
  rclc_executor_handle_t *handle = _rclc_executor_find_handle(executor, timer);
  // 从执行器中删除找到的句柄 (Remove the found handle from the executor)
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    // 如果删除失败，则设置错误消息 (If removal fails, set the error message)
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_timer.");
    return ret;
  }
  // 减少执行器中的定时器数量 (Decrease the number of timers in the executor)
  executor->info.number_of_timers--;
  // 记录调试信息 (Record debug information)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a timer.");
  return ret;
}

/**
 * @brief 删除执行器中的客户端 (Remove a client from the executor)
 *
 * @param[in] executor 指向 rclc_executor_t 结构体的指针 (Pointer to an rclc_executor_t structure)
 * @param[in] client 指向 rcl_client_t 结构体的指针 (Pointer to an rcl_client_t structure)
 * @return 返回操作结果，成功返回 RCL_RET_OK (Return the result of the operation, success returns
 * RCL_RET_OK)
 */
rcl_ret_t rclc_executor_remove_client(rclc_executor_t *executor, const rcl_client_t *client) {
  // 检查 executor 参数是否为空 (Check if the executor argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  // 检查 client 参数是否为空 (Check if the client argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(client, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 查找与给定 client 相关联的句柄 (Find the handle associated with the given client)
  rclc_executor_handle_t *handle = _rclc_executor_find_handle(executor, client);
  // 从执行器中删除找到的句柄 (Remove the found handle from the executor)
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    // 如果删除失败，则设置错误消息 (If removal fails, set the error message)
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_client.");
    return ret;
  }
  // 减少执行器中的客户端数量 (Decrease the number of clients in the executor)
  executor->info.number_of_clients--;
  // 记录调试信息 (Record debug information)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a client.");
  return ret;
}

/**
 * @brief 删除一个服务 (Remove a service)
 *
 * @param[in,out] executor 执行器指针 (Pointer to the executor)
 * @param[in] service 要删除的服务指针 (Pointer to the service to be removed)
 * @return rcl_ret_t 返回操作结果 (Return operation result)
 */
rcl_ret_t rclc_executor_remove_service(rclc_executor_t *executor, const rcl_service_t *service) {
  // 检查 executor 参数是否为空 (Check if the executor argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  // 检查 service 参数是否为空 (Check if the service argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(service, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 查找要删除的服务句柄 (Find the handle of the service to be removed)
  rclc_executor_handle_t *handle = _rclc_executor_find_handle(executor, service);
  // 移除找到的句柄 (Remove the found handle)
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    // 设置错误信息 (Set error message)
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_service.");
    return ret;
  }
  // 减少服务数量 (Decrease the number of services)
  executor->info.number_of_services--;
  // 记录调试信息 (Log debug information)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a service.");
  return ret;
}

/**
 * @brief 删除一个守护条件 (Remove a guard condition)
 *
 * @param[in,out] executor 执行器指针 (Pointer to the executor)
 * @param[in] guard_condition 要删除的守护条件指针 (Pointer to the guard condition to be removed)
 * @return rcl_ret_t 返回操作结果 (Return operation result)
 */
rcl_ret_t rclc_executor_remove_guard_condition(rclc_executor_t *executor,
                                               const rcl_guard_condition_t *guard_condition) {
  // 检查 executor 参数是否为空 (Check if the executor argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  // 检查 guard_condition 参数是否为空 (Check if the guard_condition argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(guard_condition, RCL_RET_INVALID_ARGUMENT);

  rcl_ret_t ret = RCL_RET_OK;

  // 查找要删除的守护条件句柄 (Find the handle of the guard condition to be removed)
  rclc_executor_handle_t *handle = _rclc_executor_find_handle(executor, guard_condition);
  // 移除找到的句柄 (Remove the found handle)
  ret = _rclc_executor_remove_handle(executor, handle);
  if (RCL_RET_OK != ret) {
    // 设置错误信息 (Set error message)
    RCL_SET_ERROR_MSG("Failed to remove handle in rclc_executor_remove_guard_condition.");
    return ret;
  }
  // 减少守护条件数量 (Decrease the number of guard conditions)
  executor->info.number_of_guard_conditions--;
  // 记录调试信息 (Log debug information)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Removed a guard condition.");
  return ret;
}

/**
 * @brief 添加一个动作客户端到执行器 (Add an action client to the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] action_client 动作客户端指针 (Pointer to the action client)
 * @param[in] handles_number 句柄数量 (Number of handles)
 * @param[in] ros_result_response ROS结果响应指针 (Pointer to ROS result response)
 * @param[in] ros_feedback ROS反馈指针 (Pointer to ROS feedback)
 * @param[in] goal_callback 目标回调函数 (Goal callback function)
 * @param[in] feedback_callback 反馈回调函数 (Feedback callback function)
 * @param[in] result_callback 结果回调函数 (Result callback function)
 * @param[in] cancel_callback 取消回调函数 (Cancel callback function)
 * @param[in] context 回调上下文指针 (Pointer to callback context)
 * @return rcl_ret_t 返回RCL_RET_OK表示成功，其他值表示失败 (Return RCL_RET_OK if successful, other
 * values indicate failure)
 */
rcl_ret_t rclc_executor_add_action_client(rclc_executor_t *executor,
                                          rclc_action_client_t *action_client,
                                          size_t handles_number,
                                          void *ros_result_response,
                                          void *ros_feedback,
                                          rclc_action_client_goal_callback_t goal_callback,
                                          rclc_action_client_feedback_callback_t feedback_callback,
                                          rclc_action_client_result_callback_t result_callback,
                                          rclc_action_client_cancel_callback_t cancel_callback,
                                          void *context) {
  // 检查输入参数是否为空 (Check if input arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(action_client, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(ros_result_response, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(goal_callback, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(result_callback, RCL_RET_INVALID_ARGUMENT);

  // 如果提供了反馈回调函数，检查ros_feedback是否为空 (If feedback_callback is provided, check if
  // ros_feedback is NULL)
  if (NULL != feedback_callback) {
    RCL_CHECK_ARGUMENT_FOR_NULL(ros_feedback, RCL_RET_INVALID_ARGUMENT);
  }

  rcl_ret_t ret = RCL_RET_OK;

  // 检查数组边界 (Check array bounds)
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 设置动作客户端的分配器 (Set the action client's allocator)
  action_client->allocator = executor->allocator;

  // 初始化目标句柄 (Initialize goal handles)
  action_client->goal_handles_memory = executor->allocator->allocate(
      handles_number * sizeof(rclc_action_goal_handle_t), executor->allocator->state);
  if (NULL == action_client->goal_handles_memory) {
    return RCL_RET_ERROR;
  }
  action_client->goal_handles_memory_size = handles_number;
  rclc_action_init_goal_handle_memory(action_client);

  // 设置动作客户端的ROS反馈和结果响应 (Set the action client's ROS feedback and result response)
  action_client->ros_feedback = ros_feedback;
  action_client->ros_result_response = ros_result_response;

  // 初始化取消响应的目标取消数组 (Initialize the goals_canceling array in cancel response)
  action_client->ros_cancel_response.goals_canceling.data =
      (action_msgs__msg__GoalInfo *)executor->allocator->allocate(
          handles_number * sizeof(action_msgs__msg__GoalInfo), executor->allocator->state);
  action_client->ros_cancel_response.goals_canceling.size = 0;
  action_client->ros_cancel_response.goals_canceling.capacity = handles_number;

  // 设置动作客户端的目标句柄 (Set the action client's goal handles)
  rclc_action_goal_handle_t *goal_handle = action_client->free_goal_handles;
  while (NULL != goal_handle) {
    goal_handle->action_client = action_client;
    goal_handle = goal_handle->next;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_ACTION_CLIENT;
  executor->handles[executor->index].action_client = action_client;
  executor->handles[executor->index].action_client->goal_callback = goal_callback;
  executor->handles[executor->index].action_client->feedback_callback = feedback_callback;
  executor->handles[executor->index].action_client->result_callback = result_callback;
  executor->handles[executor->index].action_client->cancel_callback = cancel_callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;  // i.e. when request came in
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = context;

  // 初始化可用状态标志 (Initialize available status flags)
  executor->handles[executor->index].action_client->feedback_available = false;
  executor->handles[executor->index].action_client->status_available = false;
  executor->handles[executor->index].action_client->goal_response_available = false;
  executor->handles[executor->index].action_client->result_response_available = false;
  executor->handles[executor->index].action_client->cancel_response_available = false;

  // 增加句柄数组的索引 (Increase the index of the handle array)
  executor->index++;

  // 使wait_set无效，以便在下一次spin_some()调用中更新'executor->wait_set' (Invalidate wait_set so
  // that in next spin_some() call the 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_action_client function.");
      return ret;
    }
  }

  // 获取动作客户端实体数量 (Get the number of entities for the action client)
  size_t num_subscriptions = 0, num_guard_conditions = 0, num_timers = 0, num_clients = 0,
         num_services = 0;

  ret = rcl_action_client_wait_set_get_num_entities(&action_client->rcl_handle, &num_subscriptions,
                                                    &num_guard_conditions, &num_timers,
                                                    &num_clients, &num_services);

  // 更新执行器信息 (Update executor information)
  executor->info.number_of_subscriptions += num_subscriptions;
  executor->info.number_of_guard_conditions += num_guard_conditions;
  executor->info.number_of_timers += num_timers;
  executor->info.number_of_clients += num_clients;
  executor->info.number_of_services += num_services;

  executor->info.number_of_action_clients++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added an action client.");
  return ret;
}

/**
 * @brief 添加一个动作服务器到执行器 (Add an action server to the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @param[in] action_server 动作服务器指针 (Pointer to the action server)
 * @param[in] handles_number 句柄数量 (Number of handles)
 * @param[in] ros_goal_request ROS目标请求指针 (Pointer to the ROS goal request)
 * @param[in] ros_goal_request_size ROS目标请求大小 (Size of the ROS goal request)
 * @param[in] goal_callback 目标回调函数 (Goal callback function)
 * @param[in] cancel_callback 取消回调函数 (Cancel callback function)
 * @param[in] context 回调上下文 (Callback context)
 * @return rcl_ret_t 返回RCL_RET_OK，如果成功添加动作服务器 (Return RCL_RET_OK if the action server
 * is added successfully)
 */
rcl_ret_t rclc_executor_add_action_server(
    rclc_executor_t *executor,
    rclc_action_server_t *action_server,
    size_t handles_number,
    void *ros_goal_request,
    size_t ros_goal_request_size,
    rclc_action_server_handle_goal_callback_t goal_callback,
    rclc_action_server_handle_cancel_callback_t cancel_callback,
    void *context) {
  // 检查参数是否为空 (Check if arguments are NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(action_server, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(ros_goal_request, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(goal_callback, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(cancel_callback, RCL_RET_INVALID_ARGUMENT);

  // 检查ros_goal_request_size是否小于等于0 (Check if ros_goal_request_size is less than or equal to
  // 0)
  if (ros_goal_request_size <= 0) {
    return RCL_RET_ERROR;
  }

  rcl_ret_t ret = RCL_RET_OK;

  // 设置动作服务器的内存分配器 (Set the action server's memory allocator)
  action_server->allocator = executor->allocator;

  // 检查数组边界 (Check array bounds)
  if (executor->index >= executor->max_handles) {
    rcl_ret_t ret = RCL_RET_ERROR;
    RCL_SET_ERROR_MSG("Buffer overflow of 'executor->handles'. Increase 'max_handles'");
    return ret;
  }

  // 初始化目标句柄 (Initialize goal handles)
  action_server->goal_handles_memory = executor->allocator->allocate(
      handles_number * sizeof(rclc_action_goal_handle_t), executor->allocator->state);
  if (NULL == action_server->goal_handles_memory) {
    return RCL_RET_ERROR;
  }
  action_server->goal_handles_memory_size = handles_number;
  rclc_action_init_goal_handle_memory(action_server);

  // 遍历并设置目标句柄 (Iterate and set goal handles)
  rclc_action_goal_handle_t *goal_handle = action_server->free_goal_handles;
  size_t ros_goal_request_index = 0;
  while (NULL != goal_handle) {
    goal_handle->ros_goal_request = (void *)&(
        (uint8_t *)ros_goal_request)[ros_goal_request_index * ros_goal_request_size];  // NOLINT()
    goal_handle->action_server = action_server;
    ros_goal_request_index++;
    goal_handle = goal_handle->next;
  }

  // 分配数据字段 (Assign data fields)
  executor->handles[executor->index].type = RCLC_ACTION_SERVER;
  executor->handles[executor->index].action_server = action_server;
  executor->handles[executor->index].action_server->goal_callback = goal_callback;
  executor->handles[executor->index].action_server->cancel_callback = cancel_callback;
  executor->handles[executor->index].invocation = ON_NEW_DATA;
  executor->handles[executor->index].initialized = true;
  executor->handles[executor->index].callback_context = context;

  // 初始化动作服务器状态 (Initialize action server states)
  executor->handles[executor->index].action_server->goal_ended = false;
  executor->handles[executor->index].action_server->goal_request_available = false;
  executor->handles[executor->index].action_server->cancel_request_available = false;
  executor->handles[executor->index].action_server->result_request_available = false;
  executor->handles[executor->index].action_server->goal_expired_available = false;

  // 增加句柄数组的索引 (Increase index of handle array)
  executor->index++;

  // 使wait_set无效，以便在下一个spin_some()调用中更新'executor->wait_set' (Invalidate wait_set so
  // that in the next spin_some() call, the 'executor->wait_set' is updated accordingly)
  if (rcl_wait_set_is_valid(&executor->wait_set)) {
    ret = rcl_wait_set_fini(&executor->wait_set);
    if (RCL_RET_OK != ret) {
      RCL_SET_ERROR_MSG("Could not reset wait_set in rclc_executor_add_action_server function.");
      return ret;
    }
  }

  // 获取实体数量 (Get the number of entities)
  size_t num_subscriptions = 0, num_guard_conditions = 0, num_timers = 0, num_clients = 0,
         num_services = 0;

  ret = rcl_action_server_wait_set_get_num_entities(&action_server->rcl_handle, &num_subscriptions,
                                                    &num_guard_conditions, &num_timers,
                                                    &num_clients, &num_services);

  // 更新执行器信息 (Update executor information)
  executor->info.number_of_subscriptions += num_subscriptions;
  executor->info.number_of_guard_conditions += num_guard_conditions;
  executor->info.number_of_timers += num_timers;
  executor->info.number_of_clients += num_clients;
  executor->info.number_of_services += num_services;

  executor->info.number_of_action_servers++;
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Added an action client.");
  return ret;
}

/**
 * @brief 检查新数据是否可用 (Check for new data availability)
 *
 * @param[in] handle 执行器句柄 (Executor handle)
 * @param[in] wait_set 等待集 (Wait set)
 * @return rcl_ret_t 返回 RCL_RET_OK 或错误代码 (Return RCL_RET_OK or error code)
 *
 * 这个函数根据执行器句柄的类型检查新数据是否可用。如果有新数据或定时器准备好，将执行器句柄的
 * data_available 设置为 true。 (This function checks for new data availability based on the type of
 * executor handle. If there is new data or timer ready, it sets the data_available of the executor
 * handle to true.)
 */
static rcl_ret_t _rclc_check_for_new_data(rclc_executor_handle_t *handle,
                                          rcl_wait_set_t *wait_set) {
  // 检查 handle 是否为空 (Check if handle is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);
  // 检查 wait_set 是否为空 (Check if wait_set is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(wait_set, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;

  // 根据句柄类型进行操作 (Operate based on handle type)
  switch (handle->type) {
    case RCLC_SUBSCRIPTION:
    case RCLC_SUBSCRIPTION_WITH_CONTEXT:
      // 如果订阅中有新消息，则设置 data_available 为 true (Set data_available to true if there is a
      // new message in the subscription)
      handle->data_available = (NULL != wait_set->subscriptions[handle->index]);
      break;

    case RCLC_TIMER:
      // 如果定时器准备好，则设置 data_available 为 true (Set data_available to true if timer is
      // ready)
      handle->data_available = (NULL != wait_set->timers[handle->index]);
      break;

    case RCLC_SERVICE:
    case RCLC_SERVICE_WITH_REQUEST_ID:
    case RCLC_SERVICE_WITH_CONTEXT:
      // 如果服务可用，则设置 data_available 为 true (Set data_available to true if service is
      // available)
      handle->data_available = (NULL != wait_set->services[handle->index]);
      break;

    case RCLC_CLIENT:
    case RCLC_CLIENT_WITH_REQUEST_ID:
      // 如果客户端可用，则设置 data_available 为 true (Set data_available to true if client is
      // available)
      handle->data_available = (NULL != wait_set->clients[handle->index]);
      break;

    case RCLC_GUARD_CONDITION:
      // 如果 guard_condition 可用，则设置 data_available 为 true (Set data_available to true if
      // guard_condition is available)
      handle->data_available = (NULL != wait_set->guard_conditions[handle->index]);
      break;

    case RCLC_ACTION_CLIENT:
      // 检查 action_client 的实体是否准备好 (Check if entities of action_client are ready)
      rc = rcl_action_client_wait_set_get_entities_ready(
          wait_set, &handle->action_client->rcl_handle, &handle->action_client->feedback_available,
          &handle->action_client->status_available, &handle->action_client->goal_response_available,
          &handle->action_client->cancel_response_available,
          &handle->action_client->result_response_available);
      break;

    case RCLC_ACTION_SERVER:
      // 检查 action_server 的实体是否准备好 (Check if entities of action_server are ready)
      rc = rcl_action_server_wait_set_get_entities_ready(
          wait_set, &handle->action_server->rcl_handle,
          &handle->action_server->goal_request_available,
          &handle->action_server->cancel_request_available,
          &handle->action_server->result_request_available,
          &handle->action_server->goal_expired_available);
      break;

    default:
      // 未知句柄类型，返回错误 (Unknown handle type, return error)
      RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME,
                              "Error in _rclc_check_for_new_data:wait_set unknwon handle type: %d",
                              handle->type);
      return RCL_RET_ERROR;
  }  // switch-case
  return rc;
}

/**
 * @brief 从订阅中获取新数据 (Take new data from subscription)
 *
 * @param[in] handle 指向 rclc_executor_handle_t 结构体的指针 (Pointer to the rclc_executor_handle_t
 * structure)
 * @param[in] wait_set 指向 rcl_wait_set_t 结构体的指针 (Pointer to the rcl_wait_set_t structure)
 * @return 返回操作结果，成功返回 RCL_RET_OK (Return the result of the operation, return RCL_RET_OK
 * on success)
 */
static rcl_ret_t _rclc_take_new_data(rclc_executor_handle_t *handle, rcl_wait_set_t *wait_set) {
  // 检查 handle 参数是否为空 (Check if the handle argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);
  // 检查 wait_set 参数是否为空 (Check if the wait_set argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(wait_set, RCL_RET_INVALID_ARGUMENT);
  // 初始化返回值为 RCL_RET_OK (Initialize return value as RCL_RET_OK)
  rcl_ret_t rc = RCL_RET_OK;

  // 根据 handle 类型执行不同的操作 (Perform different operations based on the handle type)
  switch (handle->type) {
    // 订阅类型 (Subscription type)
    case RCLC_SUBSCRIPTION:
    // 带上下文的订阅类型 (Subscription with context type)
    case RCLC_SUBSCRIPTION_WITH_CONTEXT:
      // 如果 wait_set 中存在订阅 (If there is a subscription in the wait_set)
      if (wait_set->subscriptions[handle->index]) {
        // 定义消息信息结构体变量 (Define message info structure variable)
        rmw_message_info_t messageInfo;
        // 从订阅中获取数据 (Take data from subscription)
        rc = rcl_take(handle->subscription, handle->data, &messageInfo, NULL);
        // 如果操作失败 (If the operation fails)
        if (rc != RCL_RET_OK) {
          // 即使 rcl_wait 成功，rcl_take 可能会返回此错误 (rcl_take might return this error even
          // with successful rcl_wait)
          if (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            // 打印错误信息 (Print error information)
            PRINT_RCLC_ERROR(rclc_take_new_data, rcl_take);
            // 记录错误日志 (Record error log)
            RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          }
          // 因为 rcl_take 失败，所以标记数据不可用 (Invalidate that data is available, because
          // rcl_take failed)
          if (rc == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            handle->data_available = false;
          }
          // 返回操作结果 (Return the result of the operation)
          return rc;
        }
      }
      break;

    case RCLC_TIMER:
      // case RCLC_TIMER_WITH_CONTEXT:
      // 对于定时器，无需执行任何操作。
      // For timers, there's nothing to do.
      // 在 _rclc_evaluate_data_availability() 中已经完成了定时器就绪的通知。
      // Notification that the timer is ready is already done in _rclc_evaluate_data_availability().
      break;

    case RCLC_ACTION_CLIENT:
      // 检查目标响应是否可用。
      // Check if goal response is available.
      if (handle->action_client->goal_response_available) {
        Generic_SendGoal_Response aux_goal_response;
        rmw_request_id_t aux_goal_response_header;
        // 获取目标响应。
        // Get the goal response.
        rc = rcl_action_take_goal_response(&handle->action_client->rcl_handle,
                                           &aux_goal_response_header, &aux_goal_response);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_take_new_data, rcl_action_take_goal_response);
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          return rc;
        }
        // 根据目标请求序列号找到对应的目标句柄。
        // Find the corresponding goal handle by goal request sequence number.
        rclc_action_goal_handle_t *goal_handle =
            rclc_action_find_handle_by_goal_request_sequence_number(
                handle->action_client, aux_goal_response_header.sequence_number);
        if (NULL != goal_handle) {
          goal_handle->available_goal_response = true;
          goal_handle->goal_accepted = aux_goal_response.accepted;
        }
      }
      // 检查反馈回调和反馈是否可用。
      // Check if feedback callback and feedback are available.
      if (handle->action_client->feedback_callback != NULL &&
          handle->action_client->feedback_available) {
        // 获取反馈。
        // Get the feedback.
        rc = rcl_action_take_feedback(&handle->action_client->rcl_handle,
                                      handle->action_client->ros_feedback);

        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_take_new_data, rcl_action_take_feedback);
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          return rc;
        }
        // 根据 UUID 找到对应的目标句柄。
        // Find the corresponding goal handle by UUID.
        rclc_action_goal_handle_t *goal_handle = rclc_action_find_goal_handle_by_uuid(
            handle->action_client, &handle->action_client->ros_feedback->goal_id);
        if (NULL != goal_handle) {
          goal_handle->available_feedback = true;
        }
      }
      // 检查取消响应是否可用。
      // Check if cancel response is available.
      if (handle->action_client->cancel_response_available) {
        rmw_request_id_t cancel_response_header;
        // 获取取消响应。
        // Get the cancel response.
        rc = rcl_action_take_cancel_response(&handle->action_client->rcl_handle,
                                             &cancel_response_header,
                                             &handle->action_client->ros_cancel_response);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_take_new_data, rcl_action_take_cancel_response);
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          return rc;
        }
        // 根据取消请求序列号找到对应的目标句柄。
        // Find the corresponding goal handle by cancel request sequence number.
        rclc_action_goal_handle_t *goal_handle =
            rclc_action_find_handle_by_cancel_request_sequence_number(
                handle->action_client, cancel_response_header.sequence_number);
        if (NULL != goal_handle) {
          goal_handle->available_cancel_response = true;
          goal_handle->goal_cancelled = false;
          // 遍历取消中的目标，检查是否有匹配的 UUID。
          // Iterate through goals being canceled and check for matching UUIDs.
          for (size_t i = 0; i < handle->action_client->ros_cancel_response.goals_canceling.size;
               i++) {
            rclc_action_goal_handle_t *aux = rclc_action_find_goal_handle_by_uuid(
                handle->action_client,
                &handle->action_client->ros_cancel_response.goals_canceling.data[i].goal_id);
            if (NULL != aux) {
              goal_handle->goal_cancelled = true;
            }
          }
        }
      }
      // 检查结果响应是否可用。
      // Check if result response is available.
      if (handle->action_client->result_response_available) {
        rmw_request_id_t result_request_header;
        // 获取结果响应。
        // Get the result response.
        rc = rcl_action_take_result_response(&handle->action_client->rcl_handle,
                                             &result_request_header,
                                             handle->action_client->ros_result_response);
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_take_new_data, rcl_action_take_result_response);
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          return rc;
        }
        // 根据结果请求序列号找到对应的目标句柄。
        // Find the corresponding goal handle by result request sequence number.
        rclc_action_goal_handle_t *goal_handle =
            rclc_action_find_handle_by_result_request_sequence_number(
                handle->action_client, result_request_header.sequence_number);
        if (NULL != goal_handle) {
          goal_handle->available_result_response = true;
        }
      }
      break;

    case RCLC_ACTION_SERVER:
      // 检查是否有可用的目标请求
      if (handle->action_server->goal_request_available) {
        // 获取一个目标处理器
        rclc_action_goal_handle_t *goal_handle =
            rclc_action_take_goal_handle(handle->action_server);
        // Check if a goal handle is available
        if (NULL != goal_handle) {
          // Assign the action server to the goal handle
          goal_handle->action_server = handle->action_server;
          // Take the goal request from the action server
          rc = rcl_action_take_goal_request(&handle->action_server->rcl_handle,
                                            &goal_handle->goal_request_header,
                                            goal_handle->ros_goal_request);
          // 如果获取目标请求失败，则删除已使用的目标处理器，并返回错误
          if (rc != RCL_RET_OK) {
            rclc_action_remove_used_goal_handle(handle->action_server, goal_handle);
            PRINT_RCLC_ERROR(rclc_take_new_data, rcl_action_take_goal_request);
            RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
            return rc;
          }
          // 设置目标处理器的目标ID和状态
          goal_handle->goal_id = goal_handle->ros_goal_request->goal_id;
          goal_handle->status = GOAL_STATE_UNKNOWN;
        }
      }
      // 检查是否有可用的结果请求
      if (handle->action_server->result_request_available) {
        Generic_GetResult_Request aux_result_request;
        rmw_request_id_t aux_result_request_header;
        // Take the result request from the action server
        rc = rcl_action_take_result_request(&handle->action_server->rcl_handle,
                                            &aux_result_request_header, &aux_result_request);
        // 如果获取结果请求失败，则返回错误
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_take_new_data, rcl_action_take_result_request);
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          return rc;
        }
        // 根据UUID查找目标处理器
        rclc_action_goal_handle_t *goal_handle = rclc_action_find_goal_handle_by_uuid(
            handle->action_server, &aux_result_request.goal_id);
        // 如果找到目标处理器，设置其结果请求头和状态
        if (NULL != goal_handle) {
          goal_handle->result_request_header = aux_result_request_header;
          goal_handle->status = GOAL_STATE_EXECUTING;
        }
        // 设置结果请求不可用
        handle->action_server->result_request_available = false;
      }
      // 检查是否有可用的取消请求
      if (handle->action_server->cancel_request_available) {
        action_msgs__srv__CancelGoal_Request aux_cancel_request;
        rmw_request_id_t aux_cancel_request_header;

        // Take the cancel request from the action server
        rc = rcl_action_take_cancel_request(&handle->action_server->rcl_handle,
                                            &aux_cancel_request_header, &aux_cancel_request);
        // 如果获取取消请求失败，则返回错误
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_take_new_data, rcl_action_take_cancel_request);
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          return rc;
        }
        // 根据UUID查找目标处理器
        rclc_action_goal_handle_t *goal_handle = rclc_action_find_goal_handle_by_uuid(
            handle->action_server, &aux_cancel_request.goal_info.goal_id);
        // 如果找到目标处理器，根据状态转换结果设置其取消请求头和状态
        if (NULL != goal_handle) {
          if (GOAL_STATE_CANCELING ==
              rcl_action_transition_goal_state(goal_handle->status, GOAL_EVENT_CANCEL_GOAL)) {
            goal_handle->cancel_request_header = aux_cancel_request_header;
            goal_handle->status = GOAL_STATE_CANCELING;
          } else {
            rclc_action_server_goal_cancel_reject(handle->action_server, CANCEL_STATE_TERMINATED,
                                                  aux_cancel_request_header);
          }
        } else {
          rclc_action_server_goal_cancel_reject(handle->action_server, CANCEL_STATE_UNKNOWN_GOAL,
                                                aux_cancel_request_header);
        }
      }
      break;

    case RCLC_SERVICE:
    case RCLC_SERVICE_WITH_REQUEST_ID:
    case RCLC_SERVICE_WITH_CONTEXT:
      // 检查等待集中的服务是否可用
      if (wait_set->services[handle->index]) {
        // Take the request from the service
        rc = rcl_take_request(handle->service, &handle->req_id, handle->data);
        // 如果获取请求失败，则返回错误
        if (rc != RCL_RET_OK) {
          // rcl_take_request might return this error even with successful rcl_wait
          if (rc != RCL_RET_SERVICE_TAKE_FAILED) {
            PRINT_RCLC_ERROR(rclc_take_new_data, rcl_take_request);
            RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          }
          // Invalidate that data is available because rcl_take failed
          if (rc == RCL_RET_SERVICE_TAKE_FAILED) {
            handle->data_available = false;
          }
          return rc;
        }
      }
      break;

    case RCLC_CLIENT:
    case RCLC_CLIENT_WITH_REQUEST_ID:
      // case RCLC_CLIENT_WITH_CONTEXT:
      if (wait_set->clients[handle->index]) {
        // 尝试从客户端接收响应（Try to receive response from client）
        rc = rcl_take_response(handle->client, &handle->req_id, handle->data);
        if (rc != RCL_RET_OK) {
          // rcl_take_response 可能在 rcl_wait 成功时返回此错误（rcl_take_response might return this
          // error even with successful rcl_wait）
          if (rc != RCL_RET_CLIENT_TAKE_FAILED) {
            PRINT_RCLC_ERROR(rclc_take_new_data, rcl_take_response);
            RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error number: %d", rc);
          }
          return rc;
        }
      }
      break;

    case RCLC_GUARD_CONDITION:
      // case RCLC_GUARD_CONDITION_WITH_CONTEXT:
      // 无需执行任何操作（Nothing to do）
      break;

    default:
      // 当遇到未知的句柄类型时，记录调试信息并返回错误（Log debug information and return error when
      // encountering unknown handle type）
      RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME,
                              "Error in _rclc_take_new_data:wait_set unknown handle type: %d",
                              handle->type);
      return RCL_RET_ERROR;
  }  // switch-case
  return rc;
}

/**
 * @brief 检查句柄中是否有可用数据 (Check if there is data available in the handle)
 *
 * @param[in] handle 要检查的 rclc_executor_handle_t 句柄 (The rclc_executor_handle_t handle to
 * check)
 * @return 如果句柄中有可用数据，则返回 true，否则返回 false (Returns true if there is data
 * available in the handle, otherwise returns false)
 */
bool _rclc_check_handle_data_available(rclc_executor_handle_t *handle) {
  // 检查传入的句柄是否为空 (Check if the passed handle is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, false);

  // 根据句柄类型进行处理 (Process according to the handle type)
  switch (handle->type) {
    // 处理动作客户端类型 (Handle action client type)
    case RCLC_ACTION_CLIENT:
      // 如果有反馈、状态、目标响应、取消响应或结果响应可用，则返回 true (Return true if feedback,
      // status, goal response, cancel response or result response are available)
      if (handle->action_client->feedback_available || handle->action_client->status_available ||
          handle->action_client->goal_response_available ||
          handle->action_client->cancel_response_available ||
          handle->action_client->result_response_available) {
        return true;
      }
      break;

    // 处理动作服务器类型 (Handle action server type)
    case RCLC_ACTION_SERVER:
      // 如果有目标请求、取消请求、目标过期、结果请求或目标结束可用，则返回 true (Return true if
      // goal request, cancel request, goal expired, result request or goal ended are available)
      if (handle->action_server->goal_request_available ||
          handle->action_server->cancel_request_available ||
          handle->action_server->goal_expired_available ||
          handle->action_server->result_request_available || handle->action_server->goal_ended) {
        return true;
      }
      break;

    // 处理其他类型 (Handle other types)
    default:
      // 如果句柄中有数据可用，则返回 true (Return true if there is data available in the handle)
      if (handle->data_available) {
        return true;
      }
      break;
  }  // switch-case

  // 如果没有可用数据，则返回 false (Return false if there is no data available)
  return false;
}

/**
 * @brief 操作 executor->handles[i] 对象，根据对象类型调用每个回调函数
 *        Operates on executor->handles[i] object, calls every callback of each object depending on
 * its type
 *
 * @param[in,out] handle 一个指向 rclc_executor_handle_t 结构体的指针
 *                       A pointer to a rclc_executor_handle_t structure
 * @return 返回执行结果，成功返回 RCL_RET_OK，否则返回相应的错误代码
 *         Returns the execution result, returns RCL_RET_OK on success, otherwise returns the
 * corresponding error code
 */
static rcl_ret_t _rclc_execute(rclc_executor_handle_t *handle) {
  // 检查传入的 handle 参数是否为空，为空则返回 RCL_RET_INVALID_ARGUMENT 错误
  // Check if the passed handle argument is NULL, return RCL_RET_INVALID_ARGUMENT error if it is
  // NULL
  RCL_CHECK_ARGUMENT_FOR_NULL(handle, RCL_RET_INVALID_ARGUMENT);

  // 定义返回值变量 rc，并初始化为 RCL_RET_OK
  // Define the return value variable rc and initialize it to RCL_RET_OK
  rcl_ret_t rc = RCL_RET_OK;

  // 定义一个布尔变量 invoke_callback，并初始化为 false
  // Define a boolean variable invoke_callback and initialize it to false
  bool invoke_callback = false;

  // 判断回调函数是否在有新数据时调用，并检查句柄中的数据是否可用
  // (Determine if the callback is called when there is new data, and check if the data in the
  // handle is available)
  if (handle->invocation == ON_NEW_DATA && _rclc_check_handle_data_available(handle)) {
    invoke_callback = true;
  }

  // 判断回调函数是否始终调用 (Determine if the callback is always called)
  if (handle->invocation == ALWAYS) {
    invoke_callback = true;
  }

  /**
   * @brief 执行回调 (Execute callback)
   *
   * @param[in] handle 一个指向 rclc_handle_t 结构体的指针，包含了回调函数和相关数据 (A pointer to a
   * rclc_handle_t structure containing the callback function and related data)
   * @param[in] invoke_callback 布尔值，表示是否执行回调函数 (Boolean value indicating whether to
   * execute the callback function)
   */
  // 执行回调 (execute callback)
  if (invoke_callback) {
    // 根据句柄类型选择相应的操作 (Select the appropriate operation based on the handle type)
    switch (handle->type) {
      // 订阅类型 (Subscription type)
      case RCLC_SUBSCRIPTION:
        // 如果有可用数据，则调用订阅回调函数并传入数据 (If there is available data, call the
        // subscription callback function with the data)
        if (handle->data_available) {
          handle->subscription_callback(handle->data);
        } else {
          // 如果没有可用数据，则调用订阅回调函数并传入空指针 (If there is no available data, call
          // the subscription callback function with a NULL pointer)
          handle->subscription_callback(NULL);
        }
        break;

      // 带上下文的订阅类型 (Subscription type with context)
      case RCLC_SUBSCRIPTION_WITH_CONTEXT:
        // 如果有可用数据，则调用带上下文的订阅回调函数并传入数据和上下文 (If there is available
        // data, call the subscription callback function with context with the data and context)
        if (handle->data_available) {
          handle->subscription_callback_with_context(handle->data, handle->callback_context);
        } else {
          // 如果没有可用数据，则调用带上下文的订阅回调函数并传入空指针和上下文 (If there is no
          // available data, call the subscription callback function with context with a NULL
          // pointer and context)
          handle->subscription_callback_with_context(NULL, handle->callback_context);
        }
        break;

      case RCLC_TIMER:
        // 调用定时器。Call the timer.
        rc = rcl_timer_call(handle->timer);

        // 取消的定时器不会被处理，返回成功。Canceled timers are not handled, return success.
        if (rc == RCL_RET_TIMER_CANCELED) {
          rc = RCL_RET_OK;
          break;
        }

        // 检查定时器调用是否成功。Check if the timer call was successful.
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_execute, rcl_timer_call);
          return rc;
        }
        break;

      case RCLC_SERVICE:
      case RCLC_SERVICE_WITH_REQUEST_ID:
      case RCLC_SERVICE_WITH_CONTEXT:
        // 区分用户侧服务类型。Differentiate user-side service types.
        switch (handle->type) {
          case RCLC_SERVICE:
            // 调用服务回调。Call the service callback.
            handle->service_callback(handle->data, handle->data_response_msg);
            break;
          case RCLC_SERVICE_WITH_REQUEST_ID:
            // 调用带请求 ID 的服务回调。Call the service callback with request ID.
            handle->service_callback_with_reqid(handle->data, &handle->req_id,
                                                handle->data_response_msg);
            break;
          case RCLC_SERVICE_WITH_CONTEXT:
            // 调用带上下文的服务回调。Call the service callback with context.
            handle->service_callback_with_context(handle->data, handle->data_response_msg,
                                                  handle->callback_context);
            break;
          default:
            break;  // 流程无法到达此处。Flow can't reach here.
        }
        // 处理 rcl 侧服务。Handle rcl-side services.
        rc = rcl_send_response(handle->service, &handle->req_id, handle->data_response_msg);
        // 检查发送响应是否成功。Check if sending the response was successful.
        if (rc != RCL_RET_OK) {
          PRINT_RCLC_ERROR(rclc_execute, rcl_send_response);
          return rc;
        }
        break;

      // 当回调类型为 RCLC_CLIENT 时，执行 client_callback 函数。
      // When the callback type is RCLC_CLIENT, execute the client_callback function.
      case RCLC_CLIENT:
        handle->client_callback(handle->data);
        break;

      // 当回调类型为 RCLC_CLIENT_WITH_REQUEST_ID 时，执行 client_callback_with_reqid 函数。
      // When the callback type is RCLC_CLIENT_WITH_REQUEST_ID, execute the
      // client_callback_with_reqid function.
      case RCLC_CLIENT_WITH_REQUEST_ID:
        handle->client_callback_with_reqid(handle->data, &handle->req_id);
        break;

      // 当回调类型为 RCLC_CLIENT_WITH_CONTEXT 时，该功能尚未实现。 //TODO
      // When the callback type is RCLC_CLIENT_WITH_CONTEXT, this feature is not yet implemented.
      // //TODO case RCLC_CLIENT_WITH_CONTEXT:
      //   break;

      // 当回调类型为 RCLC_GUARD_CONDITION 时，执行 gc_callback 函数。
      // When the callback type is RCLC_GUARD_CONDITION, execute the gc_callback function.
      case RCLC_GUARD_CONDITION:
        handle->gc_callback();
        break;

        // 当回调类型为 RCLC_GUARD_CONDITION_WITH_CONTEXT 时，该功能尚未实现。 //TODO
        // When the callback type is RCLC_GUARD_CONDITION_WITH_CONTEXT, this feature is not yet
        // implemented. //TODO case RCLC_GUARD_CONDITION_WITH_CONTEXT:
        //   break;

      case RCLC_ACTION_CLIENT:
        // TODO(pablogs9): 处理动作客户端状态 (Handle action client status)
        if (handle->action_client->goal_response_available) {
          // 处理动作客户端目标响应消息 (Handle action client goal response messages)
          //
          // 前置条件 (Pre-condition):
          // - 目标在 action_client->used_goal_handles 列表中 (goal in
          // action_client->used_goal_handles list)
          // - goal->available_goal_response = true
          //
          // 后置条件 (Post-condition):
          // - goal->available_goal_response = false
          rclc_action_goal_handle_t *goal_handle;
          while (
              goal_handle = rclc_action_find_first_handle_with_goal_response(handle->action_client),
              NULL != goal_handle) {
            // 设置后置条件 (Set post-condition)
            goal_handle->available_goal_response = false;
            handle->action_client->goal_callback(goal_handle, goal_handle->goal_accepted,
                                                 handle->callback_context);
            if (!goal_handle->goal_accepted ||
                RCL_RET_OK != rclc_action_send_result_request(goal_handle)) {
              rclc_action_remove_used_goal_handle(handle->action_client, goal_handle);
            } else {
              goal_handle->status = GOAL_STATE_ACCEPTED;
            }
          }
        }
        if (handle->action_client->feedback_available) {
          rclc_action_goal_handle_t *goal_handle;
          for (goal_handle = handle->action_client->used_goal_handles; NULL != goal_handle;
               goal_handle = goal_handle->next) {
            if (goal_handle->available_feedback) {
              goal_handle->available_feedback = false;

              if (handle->action_client->feedback_callback != NULL) {
                handle->action_client->feedback_callback(
                    goal_handle, handle->action_client->ros_feedback, handle->callback_context);
              }
            }
          }
        }
        if (handle->action_client->cancel_response_available) {
          rclc_action_goal_handle_t *goal_handle;
          for (goal_handle = handle->action_client->used_goal_handles; NULL != goal_handle;
               goal_handle = goal_handle->next) {
            if (goal_handle->available_cancel_response) {
              goal_handle->available_cancel_response = false;

              if (handle->action_client->cancel_callback != NULL) {
                handle->action_client->cancel_callback(goal_handle, goal_handle->goal_cancelled,
                                                       handle->callback_context);
              }
            }
          }
        }
        if (handle->action_client->result_response_available) {
          // 处理动作客户端结果响应消息 (Handle action client result response messages)
          //
          // 前置条件 (Pre-condition):
          // - 目标在 action_client->used_goal_handles 列表中 (goal in
          // action_client->used_goal_handles list)
          // - goal->available_result_response = true
          //
          // 后置条件 (Post-condition):
          // - goal->available_result_response = false
          // - 目标从 action_client->used_goal_handles 列表中删除 (goal deleted from
          // action_client->used_goal_handles list)
          rclc_action_goal_handle_t *goal_handle;
          while (goal_handle =
                     rclc_action_find_first_handle_with_result_response(handle->action_client),
                 NULL != goal_handle) {
            // 设置第一个后置条件 (Set first post-condition):
            goal_handle->available_result_response = false;
            handle->action_client->result_callback(
                goal_handle, handle->action_client->ros_result_response, handle->callback_context);

            // 设置第二个后置条件 (Set second post-condition)
            rclc_action_remove_used_goal_handle(handle->action_client, goal_handle);
          }
        }
        break;

      /**
       * @brief 处理 RCLC_ACTION_SERVER 类型的执行器句柄 (Handle RCLC_ACTION_SERVER type executor
       * handle)
       *
       * @param[in] handle 执行器句柄 (Executor handle)
       */
      case RCLC_ACTION_SERVER:
        // 如果目标操作已结束，处理操作服务器终止的目标（成功、取消或中止）
        // (If the goal action has ended, handle action server terminated goals (succeeded, canceled
        // or aborted))
        if (handle->action_server->goal_ended) {
          // 预处理条件：
          // - 目标在 action_server->used_goal_handles 列表中
          // - goal->status > GOAL_STATE_CANCELING
          //
          // 后处理条件：
          // - 从 action_server->used_goal_handles 列表中删除目标
          //
          // Pre-condition:
          // - goal in action_server->used_goal_handles list
          // - goal->status > GOAL_STATE_CANCELING
          //
          // Post-condition:
          // - goal deleted from action_server->used_goal_handles list
          rclc_action_goal_handle_t *goal_handle;
          while (goal_handle = rclc_action_find_first_terminated_handle(handle->action_server),
                 NULL != goal_handle) {
            // 设置后处理条件 (Set post-condition)
            rclc_action_remove_used_goal_handle(goal_handle->action_server, goal_handle);
          }
          handle->action_server->goal_ended = false;
        }
        // 如果有可用的目标请求，处理操作服务器目标请求消息
        // (If there is a goal request available, handle action server goal request messages)
        if (handle->action_server->goal_request_available) {
          // 预处理条件：
          // - 目标在 action_server->used_goal_handles 列表中
          // - goal->status = GOAL_STATE_UNKNOWN
          //
          // 接受后处理条件：
          // - goal->status = GOAL_STATE_ACCEPTED
          // 拒绝/错误后处理条件：
          // - 从 action_server->used_goal_handles 列表中删除目标
          //
          // Pre-condition:
          // - goal in action_server->used_goal_handles list
          // - goal->status = GOAL_STATE_UNKNOWN
          //
          // Accepted post-condition:
          // - goal->status = GOAL_STATE_ACCEPTED
          // Rejected/Error post-condition:
          // - goal deleted from action_server->used_goal_handles list
          rclc_action_goal_handle_t *goal_handle;
          while (goal_handle = rclc_action_find_first_handle_by_status(handle->action_server,
                                                                       GOAL_STATE_UNKNOWN),
                 NULL != goal_handle) {
            rcl_ret_t ret =
                handle->action_server->goal_callback(goal_handle, handle->callback_context);
            switch (ret) {
              case RCL_RET_ACTION_GOAL_ACCEPTED:
                rclc_action_server_response_goal_request(goal_handle, true);
                // 设置接受后处理条件 (Set accepted post-condition)
                goal_handle->status = GOAL_STATE_ACCEPTED;
                break;
              case RCL_RET_ACTION_GOAL_REJECTED:
              default:
                rclc_action_server_response_goal_request(goal_handle, false);
                // 设置拒绝/错误后处理条件 (Set rejected/error post-condition)
                rclc_action_remove_used_goal_handle(handle->action_server, goal_handle);
                break;
            }
          }
          handle->action_server->goal_request_available = false;
        }
        // 如果有可用的取消请求，处理操作服务器取消请求消息
        // (If there is a cancel request available, handle action server cancel request messages)
        if (handle->action_server->cancel_request_available) {
          rclc_action_goal_handle_t *goal_handle;
          for (goal_handle = handle->action_server->used_goal_handles; NULL != goal_handle;
               goal_handle = goal_handle->next) {
            if (GOAL_STATE_CANCELING == goal_handle->status) {
              goal_handle->goal_cancelled =
                  handle->action_server->cancel_callback(goal_handle, handle->callback_context);
              if (goal_handle->goal_cancelled) {
                rclc_action_server_goal_cancel_accept(goal_handle);
              } else {
                rclc_action_server_goal_cancel_reject(handle->action_server, CANCEL_STATE_REJECTED,
                                                      goal_handle->cancel_request_header);
                goal_handle->status = GOAL_STATE_EXECUTING;
              }
            }
          }
          handle->action_server->cancel_request_available = false;
        }
        break;

      default:
        RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Error in _rclc_execute: unknwon handle type: %d",
                                handle->type);
        return RCL_RET_ERROR;
    }  // switch-case
  }

  return rc;
}

/**
 * @brief 默认调度函数，用于处理执行器中的句柄 (Default scheduling function for handling handles in
 * the executor)
 *
 * @param[in] executor 执行器指针 (Pointer to the executor)
 * @return rcl_ret_t 返回 RCL_RET_OK 或其他错误代码 (Returns RCL_RET_OK or other error codes)
 */
static rcl_ret_t _rclc_default_scheduling(rclc_executor_t *executor) {
  // 检查传入的执行器参数是否为空 (Check if the passed executor argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;

  // 遍历所有句柄并检查新数据 (Iterate through all handles and check for new data)
  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    // 检查新数据并更新 wait_set (Check for new data and update wait_set)
    rc = _rclc_check_for_new_data(&executor->handles[i], &executor->wait_set);
    // 如果返回值不是 RCL_RET_OK 且不是 RCL_RET_SUBSCRIPTION_TAKE_FAILED，则返回错误代码 (If the
    // return value is not RCL_RET_OK and not RCL_RET_SUBSCRIPTION_TAKE_FAILED, return the error
    // code)
    if ((rc != RCL_RET_OK) && (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED)) {
      return rc;
    }
  }
  // 如果触发条件满足，则获取数据并执行 (If the trigger condition is fulfilled, fetch data and
  // execute)
  if (executor->trigger_function(executor->handles, executor->max_handles,
                                 executor->trigger_object)) {
    // 从 DDS-队列中获取新的输入数据并执行句柄的相应回调 (Take new input data from the DDS-queue and
    // execute the corresponding callback of the handle)
    for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
      // 获取新数据并更新 wait_set (Take new data and update wait_set)
      rc = _rclc_take_new_data(&executor->handles[i], &executor->wait_set);
      // 如果返回值不是 RCL_RET_OK、RCL_RET_SUBSCRIPTION_TAKE_FAILED 或
      // RCL_RET_SERVICE_TAKE_FAILED，则返回错误代码 (If the return value is not RCL_RET_OK,
      // RCL_RET_SUBSCRIPTION_TAKE_FAILED or RCL_RET_SERVICE_TAKE_FAILED, return the error code)
      if ((rc != RCL_RET_OK) && (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED) &&
          (rc != RCL_RET_SERVICE_TAKE_FAILED)) {
        return rc;
      }
      // 执行句柄的回调函数 (Execute the callback function of the handle)
      rc = _rclc_execute(&executor->handles[i]);
      // 如果返回值不是 RCL_RET_OK，则返回错误代码 (If the return value is not RCL_RET_OK, return
      // the error code)
      if (rc != RCL_RET_OK) {
        return rc;
      }
    }
  }
  return rc;
}

/**
 * @brief 这个函数实现了基于LET（Logical Execution Time）的调度策略。
 *        1. 读取所有输入数据
 *        2. 处理数据
 *        3. 写入数据（*）数据不是在所有回调结束时写入，但在这一轮中不会被回调处理，
 *           因为所有输入数据都是在开始时读取的，传入的消息已经被复制。
 *
 * This function implements the LET (Logical Execution Time) scheduling strategy.
 *        1. Read all input data
 *        2. Process data
 *        3. Write data (*) Data is not written at the end of all callbacks, but it will not be
 *           processed by the callbacks 'in this round' because all input data is read in the
 *           beginning and the incoming messages were copied.
 *
 * @param[in] executor 指向rclc_executor_t类型的指针
 * @param[in] executor A pointer to an rclc_executor_t type
 * @return 返回执行结果状态码
 * @return Returns the execution result status code
 */
static rcl_ret_t _rclc_let_scheduling(rclc_executor_t *executor) {
  // 检查executor是否为空
  // Check if executor is NULL
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;

  // step 0: 从DDS队列检查可用的输入数据
  // complexity: O(n) 其中n表示句柄的数量
  // step 0: check for available input data from DDS queue
  // complexity: O(n) where n denotes the number of handles
  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    rc = _rclc_check_for_new_data(&executor->handles[i], &executor->wait_set);
    if ((rc != RCL_RET_OK) && (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED)) {
      return rc;
    }
  }

  // 如果触发条件满足，则获取数据并执行
  // complexity: O(n) 其中n表示句柄的数量
  // if the trigger condition is fulfilled, fetch data and execute
  // complexity: O(n) where n denotes the number of handles
  if (executor->trigger_function(executor->handles, executor->max_handles,
                                 executor->trigger_object)) {
    // step 1: 读取输入数据
    // step 1: read input data
    for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
      rc = _rclc_take_new_data(&executor->handles[i], &executor->wait_set);
      if ((rc != RCL_RET_OK) && (rc != RCL_RET_SUBSCRIPTION_TAKE_FAILED)) {
        return rc;
      }
    }

    // step 2: 处理（执行）
    // step 2: process (execute)
    for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
      rc = _rclc_execute(&executor->handles[i]);
      if (rc != RCL_RET_OK) {
        return rc;
      }
    }
  }
  return rc;
}

/**
 * @brief 准备执行器，初始化 wait_set（等待集）。
 * @param[in,out] executor 执行器指针
 * @return 返回 rcl_ret_t 类型的结果
 *
 * @details 该函数用于准备执行器，主要是初始化 wait_set。当满足以下条件时，会初始化 wait_set：
 *          (1) 首次调用 executor_spin_some()；
 *          (2) 调用了 executor_add_timer() 或 executor_add_subscription()，
 *              即向执行器添加了新的定时器或订阅。
 */
rcl_ret_t rclc_executor_prepare(rclc_executor_t *executor) {
  // 定义返回值变量，并初始化为 RCL_RET_OK
  rcl_ret_t rc = RCL_RET_OK;
  // 检查 executor 参数是否为空，为空则返回 RCL_RET_INVALID_ARGUMENT
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  // 输出调试信息
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "executor_prepare");

  // 如果 wait_set 无效，则进行初始化
  // 条件：(1) 首次调用 executor_spin_some()
  //       (2) 调用了 executor_add_timer() 或 executor_add_subscription()
  if (!rcl_wait_set_is_valid(&executor->wait_set)) {
    // 多次调用 wait_set 的 zero_initialized wait_set 是允许的
    rcl_ret_t rc = rcl_wait_set_fini(&executor->wait_set);
    if (rc != RCL_RET_OK) {
      PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_fini);
    }
    // 初始化 wait_set
    executor->wait_set = rcl_get_zero_initialized_wait_set();
    // 为 wait_set 中的所有句柄创建足够的内存空间
    rc = rcl_wait_set_init(&executor->wait_set, executor->info.number_of_subscriptions,
                           executor->info.number_of_guard_conditions,
                           executor->info.number_of_timers, executor->info.number_of_clients,
                           executor->info.number_of_services, executor->info.number_of_events,
                           executor->context, *executor->allocator);

    if (rc != RCL_RET_OK) {
      PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_init);
      return rc;
    }
  }

  return rc;
}

/**
 * @brief 执行器执行一部分可用的处理程序 (Execute a portion of available handlers in the executor)
 *
 * @param[in] executor 指向 rclc_executor_t 结构体的指针 (Pointer to an rclc_executor_t structure)
 * @param[in] timeout_ns 超时时间，以纳秒为单位 (Timeout duration in nanoseconds)
 * @return rcl_ret_t 返回执行结果 (Return the execution result)
 */
rcl_ret_t rclc_executor_spin_some(rclc_executor_t *executor, const uint64_t timeout_ns) {
  // 初始化返回值为 RCL_RET_OK (Initialize the return value as RCL_RET_OK)
  rcl_ret_t rc = RCL_RET_OK;

  // 检查 executor 参数是否为空，如果为空则返回 RCL_RET_INVALID_ARGUMENT (Check if the executor
  // argument is NULL, return RCL_RET_INVALID_ARGUMENT if it is)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);

  // 输出调试信息 (Output debug information)
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "spin_some");

  // 检查执行器上下文是否有效，如果无效则返回 RCL_RET_ERROR (Check if the executor context is valid,
  // return RCL_RET_ERROR if it's not)
  if (!rcl_context_is_valid(executor->context)) {
    // 打印错误信息 (Print error message)
    PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_context_not_valid);
    return RCL_RET_ERROR;
  }

  // 准备执行器 (Prepare the executor)
  rclc_executor_prepare(executor);

  // 设置 rmw 字段为空 (Set rmw fields to NULL)
  rcl_ret_t rc = rcl_wait_set_clear(&executor->wait_set);
  // 如果返回值不是 RCL_RET_OK，则打印错误信息并返回该值 (If the return value is not RCL_RET_OK,
  // print the error message and return the value)
  if (rc != RCL_RET_OK) {
    PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_clear);
    return rc;
  }

  // (jst3si) 将其放入子函数以提高可读性 (put in a sub-function - for improved readability)
  // 向 wait_set 添加 handles (add handles to wait_set)
  for (size_t i = 0; (i < executor->max_handles && executor->handles[i].initialized); i++) {
    // 打印调试信息，显示要添加的 handle 类型 (Print debug information, showing the type of handle
    // to be added)
    RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "wait_set_add_* %d", executor->handles[i].type);

    // 根据 handle 类型执行相应操作 (Perform corresponding operations according to the handle type)
    switch (executor->handles[i].type) {
      case RCLC_SUBSCRIPTION:
      case RCLC_SUBSCRIPTION_WITH_CONTEXT:
        // 将订阅添加到 wait_set 并保存索引 (add subscription to wait_set and save index)
        rc = rcl_wait_set_add_subscription(&executor->wait_set, executor->handles[i].subscription,
                                           &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          // 订阅成功添加到 wait_set 中，打印调试信息 (Subscription successfully added to wait_set,
          // print debug information)
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME,
                                  "Subscription added to wait_set_subscription[%ld]",
                                  executor->handles[i].index);
        } else {
          // 添加订阅失败，打印错误信息并返回错误代码 (Failed to add subscription, print error
          // message and return error code)
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_subscription);
          return rc;
        }
        break;

      case RCLC_TIMER:
        // case RCLC_TIMER_WITH_CONTEXT:
        // 将定时器添加到 wait_set 并保存索引 (add timer to wait_set and save index)
        rc = rcl_wait_set_add_timer(&executor->wait_set, executor->handles[i].timer,
                                    &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          // 定时器成功添加到 wait_set 中，打印调试信息 (Timer successfully added to wait_set, print
          // debug information)
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Timer added to wait_set_timers[%ld]",
                                  executor->handles[i].index);
        } else {
          // 添加定时器失败，打印错误信息并返回错误代码 (Failed to add timer, print error message
          // and return error code)
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_timer);
          return rc;
        }
        break;

        /**
         * @brief 根据不同的类型，将服务、客户端、保护条件和动作客户端添加到等待集中，并保存索引。
         *        Add services, clients, guard conditions, and action clients to the wait set based
         * on their types, and save the index.
         */
      case RCLC_SERVICE:
      case RCLC_SERVICE_WITH_REQUEST_ID:
      case RCLC_SERVICE_WITH_CONTEXT:
        // 将服务添加到等待集并保存索引。Add service to wait_set and save index.
        rc = rcl_wait_set_add_service(&executor->wait_set, executor->handles[i].service,
                                      &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          // 记录服务已添加到等待集的日志。Log that the service has been added to the wait set.
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Service added to wait_set_service[%ld]",
                                  executor->handles[i].index);
        } else {
          // 打印错误信息。Print error message.
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_service);
          return rc;
        }
        break;

      case RCLC_CLIENT:
      case RCLC_CLIENT_WITH_REQUEST_ID:
        // 将客户端添加到等待集并保存索引。Add client to wait_set and save index.
        rc = rcl_wait_set_add_client(&executor->wait_set, executor->handles[i].client,
                                     &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          // 记录客户端已添加到等待集的日志。Log that the client has been added to the wait set.
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Client added to wait_set_client[%ld]",
                                  executor->handles[i].index);
        } else {
          // 打印错误信息。Print error message.
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_client);
          return rc;
        }
        break;

      case RCLC_GUARD_CONDITION:
        // 将保护条件添加到等待集并保存索引。Add guard_condition to wait_set and save index.
        rc = rcl_wait_set_add_guard_condition(&executor->wait_set, executor->handles[i].gc,
                                              &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          // 记录保护条件已添加到等待集的日志。Log that the guard condition has been added to the
          // wait set.
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Guard_condition added to wait_set_client[%ld]",
                                  executor->handles[i].index);
        } else {
          // 打印错误信息。Print error message.
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_guard_condition);
          return rc;
        }
        break;

      case RCLC_ACTION_CLIENT:
        // 将动作客户端添加到等待集并保存索引。Add action client to wait_set and save index.
        rc = rcl_action_wait_set_add_action_client(&executor->wait_set,
                                                   &executor->handles[i].action_client->rcl_handle,
                                                   &executor->handles[i].index, NULL);
        if (rc == RCL_RET_OK) {
          // 记录动作客户端已添加到等待集的日志。Log that the action client has been added to the
          // wait set.
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME,
                                  "Action client added to wait_set_action_clients[%ld]",
                                  executor->handles[i].index);
        } else {
          // 打印错误信息。Print error message.
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_action_client);
          return rc;
        }
        break;

      case RCLC_ACTION_SERVER:
        // 添加动作服务器到等待集合并保存索引 (Add action server to wait_set and save index)
        rc = rcl_action_wait_set_add_action_server(&executor->wait_set,
                                                   &executor->handles[i].action_server->rcl_handle,
                                                   &executor->handles[i].index);
        if (rc == RCL_RET_OK) {
          // 如果成功添加，打印调试信息 (If successfully added, print debug information)
          RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME,
                                  "Action server added to wait_set_action_servers[%ld]",
                                  executor->handles[i].index);
        } else {
          // 如果添加失败，打印错误信息并返回错误代码 (If adding fails, print error information and
          // return error code)
          PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_action_server);
          return rc;
        }
        break;

      default:
        // 如果句柄类型未知，打印调试信息 (If the handle type is unknown, print debug information)
        RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "Error: unknown handle type: %d",
                                executor->handles[i].type);
        // 打印错误信息并返回错误代码 (Print error information and return error code)
        PRINT_RCLC_ERROR(rclc_executor_spin_some, rcl_wait_set_add_unknown_handle);
        return RCL_RET_ERROR;
    }
  }

  /**
   * @brief 等待接收 DDS 队列中的新数据通知，最长等待时间为 'timeout_ns'。
   * @brief Wait up to 'timeout_ns' to receive notification about which handles received new data
   * from DDS queue.
   */
  rc = rcl_wait(&executor->wait_set, timeout_ns);

  // RCLC_UNUSED 是一个宏，用于消除未使用变量的编译器警告。
  // RCLC_UNUSED is a macro used to eliminate compiler warnings for unused variables.
  RCLC_UNUSED(rc);

  /**
   * @brief 根据语义处理输入数据。
   * @brief Process input data based on semantics.
   *
   * @param[in] executor->data_comm_semantics 数据通信语义。
   * @param[in] executor->data_comm_semantics Data communication semantics.
   */
  switch (executor->data_comm_semantics) {
    case LET:
      /**
       * @brief 使用 LET 调度策略处理输入数据。
       * @brief Process input data using LET scheduling strategy.
       */
      rc = _rclc_let_scheduling(executor);
      break;
    case RCLCPP_EXECUTOR:
      /**
       * @brief 使用默认调度策略处理输入数据。
       * @brief Process input data using default scheduling strategy.
       */
      rc = _rclc_default_scheduling(executor);
      break;
    default:
      /**
       * @brief 打印未知语义错误，并返回 RCL_RET_ERROR。
       * @brief Print unknown semantics error and return RCL_RET_ERROR.
       */
      PRINT_RCLC_ERROR(rclc_executor_spin_some, unknown_semantics);
      return RCL_RET_ERROR;
  }

  return rc;
}

/**
 * @brief 执行器执行一次spin操作，处理订阅者、服务和定时器等回调函数。
 *        Execute a single spin of the executor, processing callbacks from subscribers, services,
 * and timers.
 *
 * @param[in] executor 指向rclc_executor_t类型的指针。
 *                    Pointer to an rclc_executor_t object.
 * @return 返回rcl_ret_t类型的结果。
 *         Returns an rcl_ret_t result.
 */
rcl_ret_t rclc_executor_spin(rclc_executor_t *executor) {
  // 检查输入参数是否为空
  // Check if input argument is NULL
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  // 打印调试信息，显示rcl_wait的超时时间
  // Print debug info, showing the timeout for rcl_wait
  RCUTILS_LOG_DEBUG_NAMED(ROS_PACKAGE_NAME, "INFO: rcl_wait timeout %ld ms",
                          ((executor->timeout_ns / 1000) / 1000));
  while (true) {
    // 调用rclc_executor_spin_some函数处理回调函数
    // Call rclc_executor_spin_some to process callbacks
    ret = rclc_executor_spin_some(executor, executor->timeout_ns);
    // 如果返回值不是RCL_RET_OK或RCL_RET_TIMEOUT，则设置错误消息并返回错误码
    // If the return value is not RCL_RET_OK or RCL_RET_TIMEOUT, set error message and return error
    // code
    if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
      RCL_SET_ERROR_MSG("rclc_executor_spin_some error");
      return ret;
    }
  }
  return ret;
}

/**
 * @brief 执行器执行一个周期的spin操作。
 *        Execute a single period spin of the executor.
 *
 * @param[in] executor 指向rclc_executor_t类型的指针。
 *                    Pointer to an rclc_executor_t object.
 * @param[in] period 周期时间，单位为纳秒。
 *                  Period time in nanoseconds.
 * @return 返回rcl_ret_t类型的结果。
 *         Returns an rcl_ret_t result.
 */
rcl_ret_t rclc_executor_spin_one_period(rclc_executor_t *executor, const uint64_t period) {
  // 检查输入参数是否为空
  // Check if input argument is NULL
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret = RCL_RET_OK;
  rcutils_time_point_value_t end_time_point;
  rcutils_duration_value_t sleep_time;

  // 如果调用时间为0，则获取当前系统时间
  // If invocation time is 0, get the current system time
  if (executor->invocation_time == 0) {
    ret = rcutils_system_time_now(&executor->invocation_time);
    RCLC_UNUSED(ret);
  }
  // 调用rclc_executor_spin_some函数处理回调函数
  // Call rclc_executor_spin_some to process callbacks
  ret = rclc_executor_spin_some(executor, executor->timeout_ns);
  // 如果返回值不是RCL_RET_OK或RCL_RET_TIMEOUT，则设置错误消息并返回错误码
  // If the return value is not RCL_RET_OK or RCL_RET_TIMEOUT, set error message and return error
  // code
  if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
    RCL_SET_ERROR_MSG("rclc_executor_spin_some error");
    return ret;
  }
  // 计算下一个调用时间点，即当前调用时间加上周期
  // Calculate the next invocation time point, which is the current invocation time plus the period
  ret = rcutils_system_time_now(&end_time_point);
  sleep_time = (executor->invocation_time + period) - end_time_point;
  // 如果sleep_time大于0，则进行休眠
  // If sleep_time is greater than 0, sleep
  if (sleep_time > 0) {
    rclc_sleep_ms(sleep_time / 1000000);
  }
  // 更新调用时间
  // Update invocation time
  executor->invocation_time += period;
  return ret;
}

/**
 * @brief 执行器周期性执行 (Periodically execute the executor)
 *
 * @param[in] executor 指向 rclc_executor_t 结构体的指针 (Pointer to an rclc_executor_t structure)
 * @param[in] period 周期时间，单位为纳秒 (Period time in nanoseconds)
 * @return 返回 rcl_ret_t 类型的结果 (Returns a result of type rcl_ret_t)
 */
rcl_ret_t rclc_executor_spin_period(rclc_executor_t *executor, const uint64_t period) {
  // 检查 executor 参数是否为空 (Check if the executor argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t ret;
  // 循环执行 (Loop execution)
  while (true) {
    // 在给定周期内执行一个任务 (Execute one task within the given period)
    ret = rclc_executor_spin_one_period(executor, period);
    // 如果返回值不是 RCL_RET_OK 或 RCL_RET_TIMEOUT，则设置错误消息并返回错误码 (If the return value
    // is not RCL_RET_OK or RCL_RET_TIMEOUT, set the error message and return the error code)
    if (!((ret == RCL_RET_OK) || (ret == RCL_RET_TIMEOUT))) {
      RCL_SET_ERROR_MSG("rclc_executor_spin_one_period error");
      return ret;
    }
  }
  // 从未到达此处 (Never get here)
  return RCL_RET_OK;
}

/**
 * @brief 设置执行器触发器 (Set the executor trigger)
 *
 * @param[in] executor 指向 rclc_executor_t 结构体的指针 (Pointer to an rclc_executor_t structure)
 * @param[in] trigger_function 触发器函数 (Trigger function)
 * @param[in] trigger_object 触发器对象 (Trigger object)
 * @return 返回 rcl_ret_t 类型的结果 (Returns a result of type rcl_ret_t)
 */
rcl_ret_t rclc_executor_set_trigger(rclc_executor_t *executor,
                                    rclc_executor_trigger_t trigger_function,
                                    void *trigger_object) {
  // 检查 executor 参数是否为空 (Check if the executor argument is NULL)
  RCL_CHECK_ARGUMENT_FOR_NULL(executor, RCL_RET_INVALID_ARGUMENT);
  // 设置触发器函数和对象 (Set the trigger function and object)
  executor->trigger_function = trigger_function;
  executor->trigger_object = trigger_object;
  return RCL_RET_OK;
}

/**
 * @brief 检查所有句柄是否有数据可用 (Check if data is available for all handles)
 *
 * @param[in] handles 句柄数组 (Handle array)
 * @param[in] size 句柄数组大小 (Size of the handle array)
 * @param[in] obj 未使用的参数 (Unused parameter)
 * @return 如果所有句柄都有数据可用，则返回 true，否则返回 false (Returns true if data is available
 * for all handles, false otherwise)
 */
bool rclc_executor_trigger_all(rclc_executor_handle_t *handles, unsigned int size, void *obj) {
  // 检查 handles 参数是否为空 (Check if the handles argument is NULL)
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);
  // 未使用的参数 (Unused parameter)
  RCLC_UNUSED(obj);
  // 遍历句柄数组 (Iterate through the handle array)
  for (unsigned int i = 0; i < size; i++) {
    // 如果句柄已初始化 (If the handle is initialized)
    if (handles[i].initialized) {
      // 检查句柄是否有数据可用，如果没有，则返回 false (Check if data is available for the handle,
      // return false if not)
      if (_rclc_check_handle_data_available(&handles[i]) == false) {
        return false;
      }
    } else {
      break;
    }
  }
  // 如果所有句柄都有数据可用，则返回 true (Return true if data is available for all handles)
  return true;
}

/**
 * @brief 检查任何一个句柄是否有数据可用 (Check if data is available for any handle)
 *
 * @param[in] handles 句柄数组 (Handle array)
 * @param[in] size 句柄数组大小 (Size of the handle array)
 * @param[in] obj 未使用的参数 (Unused parameter)
 * @return 如果任何一个句柄有数据可用，则返回 true，否则返回 false (Returns true if data is
 * available for any handle, false otherwise)
 */
bool rclc_executor_trigger_any(rclc_executor_handle_t *handles, unsigned int size, void *obj) {
  // 检查 handles 参数是否为空 (Check if the handles argument is NULL)
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);
  // 未使用的参数 (Unused parameter)
  RCLC_UNUSED(obj);
  // 遍历句柄数组 (Iterate through the handle array)
  for (unsigned int i = 0; i < size; i++) {
    // 如果句柄已初始化 (If the handle is initialized)
    if (handles[i].initialized) {
      // 检查句柄是否有数据可用，如果有，则返回 true (Check if data is available for the handle,
      // return true if so)
      if (_rclc_check_handle_data_available(&handles[i])) {
        return true;
      }
    } else {
      break;
    }
  }
  // 如果没有任何一个句柄有数据可用，则返回 false (Return false if no data is available for any
  // handle)
  return false;
}

/**
 * @brief 检查给定的对象是否触发了其中一个句柄 (Check if the given object triggers one of the
 * handles)
 *
 * @param[in] handles 句柄数组 (Array of handles)
 * @param[in] size 句柄数组的大小 (Size of the handles array)
 * @param[in] obj 要检查的对象 (Object to check for triggering)
 * @return bool 如果给定的对象触发了其中一个句柄，则返回 true，否则返回 false (Returns true if the
 * given object triggers one of the handles, false otherwise)
 */
bool rclc_executor_trigger_one(rclc_executor_handle_t *handles, unsigned int size, void *obj) {
  // 检查 handles 是否为空，如果为空，返回错误消息并返回 false
  // (Check if handles is NULL, if it is, return an error message and return false)
  RCL_CHECK_FOR_NULL_WITH_MSG(handles, "handles is NULL", return false);

  // 遍历句柄数组，但不使用 (i<size && handles[i].initialized) 作为循环条件
  // 因为对于最后一个索引 i==size，这将导致越界访问
  // (Iterate through the handles array, but do not use (i<size && handles[i].initialized) as
  // loop-condition because for the last index i==size this would result in out-of-bound access)
  for (unsigned int i = 0; i < size; i++) {
    // 如果当前句柄已初始化 (If the current handle is initialized)
    if (handles[i].initialized) {
      // 检查当前句柄是否有可用的数据 (Check if the current handle has data available)
      if (_rclc_check_handle_data_available(&handles[i])) {
        // 获取当前句柄的对象指针 (Get the object pointer of the current handle)
        void *handle_obj_ptr = rclc_executor_handle_get_ptr(&handles[i]);
        // 如果获取到的对象指针为空，说明不支持该类型，返回 false
        // (If the obtained object pointer is NULL, it means that the type is not supported, return
        // false)
        if (NULL == handle_obj_ptr) {
          // rclc_executor_handle_get_ptr returns null for unsupported types
          return false;
        }
        // 如果给定的对象与当前句柄的对象指针相同，则返回 true
        // (If the given object is the same as the object pointer of the current handle, return
        // true)
        if (obj == handle_obj_ptr) {
          return true;
        }
      }
    } else {
      break;
    }
  }
  return false;
}

/**
 * @brief 总是触发执行器 (Always trigger the executor)
 *
 * @param[in] handles 句柄数组 (Array of handles)
 * @param[in] size 句柄数组的大小 (Size of the handles array)
 * @param[in] obj 要检查的对象 (Object to check for triggering)
 * @return bool 始终返回 true (Always returns true)
 */
bool rclc_executor_trigger_always(rclc_executor_handle_t *handles, unsigned int size, void *obj) {
  // 将传入的参数标记为未使用，以避免编译器警告
  // (Mark the incoming parameters as unused to avoid compiler warnings)
  RCLC_UNUSED(handles);
  RCLC_UNUSED(size);
  RCLC_UNUSED(obj);

  // 始终返回 true (Always return true)
  return true;
}
