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
#ifndef RCLC__EXECUTOR_H_
#define RCLC__EXECUTOR_H_

#if __cplusplus
extern "C" {
#endif

#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>
#include <stdarg.h>
#include <stdio.h>

#include "rclc/action_client.h"
#include "rclc/action_server.h"
#include "rclc/executor_handle.h"
#include "rclc/sleep.h"
#include "rclc/types.h"
#include "rclc/visibility_control.h"

/*! \file executor.h
    \brief RCLC-Executor 提供了一个基于 RCL 的执行器，其中所有回调都按照用户定义的顺序进行处理。
           The RCLC-Executor provides an Executor based on RCL in which all callbacks are
           processed in a user-defined order.
*/

/* 定义数据通信的语义
   RCLCPP_EXECUTOR - 与 rclcpp Executor ROS2(Eloquent) 中的语义相同
   LET             - 逻辑执行时间
   Defines the semantics of data communication
   RCLCPP_EXECUTOR - same semantics as in the rclcpp Executor ROS2(Eloquent)
   LET             - logical execution time
*/
typedef enum { RCLCPP_EXECUTOR, LET } rclc_executor_semantics_t;

/// 触发函数的类型定义。带有以下参数：
/// - 执行器句柄数组
/// - 数组大小
/// - 触发函数中使用的特定于应用程序的结构
/// Type definition for trigger function. With the parameters:
/// - array of executor_handles
/// - size of array
/// - application specific struct used in the trigger function
typedef bool (*rclc_executor_trigger_t)(rclc_executor_handle_t *, unsigned int, void *);

/** 容器 RCLC-Executor
 * Container for RCLC-Executor
 *
 */
typedef struct {
  /// 上下文（用于获取 ROS 是否正在运行的信息）
  /// Context (to get information if ROS is up-and-running)
  rcl_context_t *context;
  /// DDS 句柄的动态数组容器
  /// Container for dynamic array for DDS-handles
  rclc_executor_handle_t *handles;
  /// 数组 'handles' 的最大大小
  /// Maximum size of array 'handles'
  size_t max_handles;
  /// 数组 handles 中下一个空闲元素的索引
  /// Index to the next free element in array handles
  size_t index;
  /// 用于存储分配器的数组句柄的容器
  /// Container to memory allocator for array handles
  const rcl_allocator_t *allocator;
  /// 等待集合（仅在第一次调用 rclc_executor_spin_some 函数时初始化）
  /// Wait set (is initialized only in the first call of the rclc_executor_spin_some function)
  rcl_wait_set_t wait_set;
  /// 关于订阅、定时器、客户端、服务等的总数的统计对象
  /// Statistics objects about total number of subscriptions, timers, clients, services, etc.
  rclc_executor_handle_counters_t info;
  /// 用于 rclc_executor_spin_once() 中的 rcl_wait() 的超时时间（以纳秒为单位）。默认值为 100ms
  /// timeout in nanoseconds for rcl_wait() used in rclc_executor_spin_once(). Default 100ms
  uint64_t timeout_ns;
  /// 用于 spin_period() 的时间点
  /// timepoint used for spin_period()
  rcutils_time_point_value_t invocation_time;
  /// 触发器函数，何时处理新数据
  /// trigger function, when to process new data
  rclc_executor_trigger_t trigger_function;
  /// 触发器函数的应用特定数据结构
  /// application specific data structure for trigger function
  void *trigger_object;
  /// 数据通信语义
  /// data communication semantics
  rclc_executor_semantics_t data_comm_semantics;
} rclc_executor_t;

/**
 * 返回一个 rclc_executor_t 结构体，指针成员初始化为 `NULL`，成员变量为 0。
 * Return a rclc_executor_t struct with pointer members initialized to `NULL`
 * and member variables to 0.
 */
RCLC_PUBLIC
rclc_executor_t rclc_executor_get_zero_initialized_executor(void);

/**
 * 初始化一个执行器。
 * 使用 \p 分配器创建大小为 \p number_of_handles 的动态数组。
 * 由于执行器用于嵌入式控制器，动态内存管理至关重要。
 * 因此，在 RCLC-Executor 初始化时，用户定义总的 \p number_of_handles。
 * 句柄是订阅、计时器、服务、客户端和保护条件的术语。
 * 堆将仅在此阶段分配，执行器运行阶段不再分配内存。
 *
 * Initializes an executor.
 * It creates a dynamic array with size \p number_of_handles using the
 * \p allocator.
 * As the Executor is intended for embedded controllers, dynamic memory management is crucial.
 * Therefore at initialization of the RCLC-Executor, the user defines the total \p
 * number_of_handles. A handle is a term for subscriptions, timers, services, clients and guard
 * conditions. The heap will be allocated only in this phase and no more memory will be allocated in
 * the running phase in the executor.
 *
 * 在 XRCE-DDS 中间件中也配置了最大数量。请参阅
 * [内存管理教程](https://docs.vulcanexus.org/en/humble/rst/tutorials/micro/memory_management/memory_management.html#entity-creation)
 * 了解默认值。如果需要更大的值，您需要更新 colcon.meta 配置文件并重新构建。
 * 要确保更改已应用，可以检查以下库包含文件中定义的值：
 * build/rmw_microxrcedds/include/rmw_microxrcedds_c/config.h
 *
 * Also in the XRCE-DDS middleware the maximum number are configured. See [Memory Management
 * Tutorial](https://docs.vulcanexus.org/en/humble/rst/tutorials/micro/memory_management/memory_management.html#entity-creation)
 * for the default values. If you need larger values, you need to update your colcon.meta
 * configuration file and rebuild. To make sure that the changes were applied, you can check
 * the defined values in the following library include file:
 * build/rmw_microxrcedds/include/rmw_microxrcedds_c/config.h
 *
 * 相应的 wait-set 堆内存在 spin-method 的第一次迭代中分配，该方法在内部调用 rclc_executor_prepare。
 * 或者，在调用任何 spin-method 之前，您也可以选择调用 rclc_executor_prepare。
 * 然后，所有与 wait-set 相关的内存分配将在 rclc_executor_prepare 中完成，而不是在 spin-method
 * 的第一次迭代中。
 *
 * The heap memory of corresponding wait-set is
 * allocated in the first iteration of a spin-method, which calls internally rclc_executor_prepare.
 * Optionally, you can also call rclc_executor_prepare before calling any of the spin-methods.
 * Then all wait-set related memory allocation will be done in rclc_executor_prepare and not
 * in the first iteration of the spin-method.
 *
 * 这使得执行器在内存分配方面是静态的，即在运行时不会发生堆分配。
 * 然而，您可以在运行时向执行器添加尽可能多的句柄（例如订阅），直到达到最大句柄数。
 * 在这种情况下，需要更新 wait-set 并再次调用 rclc_executor_prepare（在 RCL 中具有动态内存分配）。
 *
 * This makes this Executor static in
 * terms of memory allocation, in the sense, that during runtime no heap allocations occur.
 * You can add, however, at runtime as many handles, e.g. subscriptions, to the executor
 * until the maximum number of handles is reached. In this case, the wait-set needs to be
 * updated and rclc_executor_prepare is called again (with dynamic memory allocation in RCL).
 *
 *
 * <hr>
 * 属性          | 符合性
 * --------------| -------------
 * 分配内存       | 是
 * 线程安全        | 否
 * 使用原子操作     | 否
 * 无锁            | 是
 *
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[inout] executor 预分配的 rclc_executor_t
 * \param[in] context RCL 上下文
 * \param[in] number_of_handles
 * 是订阅、计时器、服务、客户端和保护条件的总数。不包括节点和发布者的数量。 \param[in] allocator
 * 用于分配内存的分配器 \return `RCL_RET_OK` 如果执行器成功初始化 \return `RCL_RET_INVALID_ARGUMENT`
 * 如果参数中存在空指针 \return `RCL_RET_ERROR` 如果出现失败
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_init(rclc_executor_t *executor,
                             rcl_context_t *context,
                             const size_t number_of_handles,
                             const rcl_allocator_t *allocator);

/**
 * 设置 rcl_wait 的超时时间（以纳秒为单位），在 {@link rclc_executor_spin_once()} 中调用。
 * Set timeout in nanoseconds for rcl_wait (called during {@link rclc_executor_spin_once()}).
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 *               pointer to an initialized executor
 * \param [in] timeout_ns  rcl_wait（DDS中间件）的超时时间，以纳秒为单位
 *               timeout in nanoseconds for the rcl_wait (DDS middleware)
 * \return `RCL_RET_OK` 如果成功设置了超时时间
 *         `RCL_RET_OK` if timeout was set successfully
 * \return `RCL_RET_INVALID_ARGUMENT` 如果 \p executor 是空指针
 *         `RCL_RET_INVALID_ARGUMENT` if \p executor is a null pointer
 * \return `RCL_RET_ERROR` 如果发生错误
 *         `RCL_RET_ERROR` in an error occured
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_set_timeout(rclc_executor_t *executor, const uint64_t timeout_ns);

/**
 * 设置数据通信语义
 * Set data communication semantics
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 *               pointer to an initialized executor
 * \param [in] valid 根据枚举类型 {@link rclc_executor_semantics_t} 中定义的有效语义值
 *             valid semantics value as defined in enum type {@link rclc_executor_semantics_t}
 * \return `RCL_RET_OK` 如果成功设置了语义
 *         `RCL_RET_OK` if semantics was set successfully
 * \return `RCL_RET_INVALID_ARGUMENT` 如果 \p executor 是空指针
 *         `RCL_RET_INVALID_ARGUMENT` if \p executor is a null pointer
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_set_semantics(rclc_executor_t *executor,
                                      rclc_executor_semantics_t semantics);

/**
 * 清理执行器。
 * 释放 {@link rclc_executor_t.handles} 的动态内存并
 * 重置 {@link rclc_executor_t} 的所有其他值。
 *
 * Cleans up executor.
 * Deallocates dynamic memory of {@link rclc_executor_t.handles} and
 * resets all other values of {@link rclc_executor_t}.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化执行器的指针
 * \param [inout] executor pointer to initialized executor
 * \return `RCL_RET_OK` 如果重置操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果 \p executor 是空指针
 * \return `RCL_RET_INVALID_ARGUMENT` 如果 \p executor.handles 是空指针
 * \return `RCL_RET_ERROR` 如果发生错误（即执行器未初始化）
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_fini(rclc_executor_t *executor);

/**
 * 向执行器添加订阅。
 * * 如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 * * {@link rclc_executor_t.info} 的 total_number_of_subscriptions 字段增加 1。
 *
 * Adds a subscription to an executor.
 * * An error is returned, if {@link rclc_executor_t.handles} array is full.
 * * The total number_of_subscriptions field of {@link rclc_executor_t.info}
 *   is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化执行器的指针
 * \param [in] subscription 指向已分配订阅的指针
 * \param [in] msg 指向已分配消息的指针
 * \param [in] callback 回调函数指针
 * \param [in] invocation 用于回调的调用类型（ALWAYS 或仅 ON_NEW_DATA）
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *executor,
                                         rcl_subscription_t *subscription,
                                         void *msg,
                                         rclc_subscription_callback_t callback,
                                         rclc_executor_handle_invocation_t invocation);

/**
 *  添加一个订阅到执行器。 (Adds a subscription to an executor.)
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。 (An error is returned, if {@link
 * rclc_executor_t.handles} array is full.)
 *  {@link rclc_executor_t.info} 的 total_number_of_subscriptions 字段增加1。 (The
 * total_number_of_subscriptions field of {@link rclc_executor_t.info} is incremented by one.)
 *
 * <hr>
 * 属性 (Attribute)          | 符合性 (Adherence)
 * ------------------ | -------------
 * 分配内存 (Allocates Memory)   | 否 (No)
 * 线程安全 (Thread-Safe)        | 否 (No)
 * 使用原子操作 (Uses Atomics)       | 否 (No)
 * 无锁 (Lock-Free)          | 是 (Yes)
 *
 * \param [inout] executor 指向初始化的执行器的指针 (pointer to initialized executor)
 * \param [in] subscription 指向分配的订阅的指针 (pointer to an allocated subscription)
 * \param [in] msg 指向分配的消息的指针 (pointer to an allocated message)
 * \param [in] callback 函数指针指向回调 (function pointer to a callback)
 * \param [in] context 类型擦除的指针，指向附加的回调上下文 (type-erased ptr to additional callback
 * context) \param [in] invocation 回调的调用类型 (ALWAYS 或 ON_NEW_DATA) (invocation type for the
 * callback (ALWAYS or only ON_NEW_DATA)) \return 如果添加操作成功，则返回 `RCL_RET_OK` (if
 * add-operation was successful) \return 如果任何参数是空指针（忽略 NULL 上下文），则返回
 * `RCL_RET_INVALID_ARGUMENT` (if any parameter is a null pointer (NULL context is ignored)) \return
 * 如果发生其他错误，则返回 `RCL_RET_ERROR` (if any other error occured)
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_subscription_with_context(
    rclc_executor_t *executor,  // 指向初始化的执行器的指针 (pointer to initialized executor)
    rcl_subscription_t
        *subscription,  // 指向分配的订阅的指针 (pointer to an allocated subscription)
    void *msg,          // 指向分配的消息的指针 (pointer to an allocated message)
    rclc_subscription_callback_with_context_t
        callback,       // 函数指针指向回调 (function pointer to a callback)
    void *context,  // 类型擦除的指针，指向附加的回调上下文 (type-erased ptr to additional callback
                    // context)
    rclc_executor_handle_invocation_t
        invocation);  // 回调的调用类型 (ALWAYS 或 ON_NEW_DATA) (invocation type for the callback
                      // (ALWAYS or only ON_NEW_DATA))

/**
 *  添加定时器到执行器。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_timers 字段增加 1。
 *
 *  Add a timer to an executor.
 *  An error is returned, if {@link rclc_executor_t.handles} array is full.
 *  The total_number_of_timers field of {@link rclc_executor_t.info} is
 *    incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] timer 指向已分配的定时器的指针
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 *
 * \param [inout] executor pointer to initialized executor
 * \param [in] timer pointer to an allocated timer
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_timer(rclc_executor_t *executor, rcl_timer_t *timer);

/**
 *  将客户端添加到执行器。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_clients 字段增加 1。
 *
 *  Adds a client to an executor.
 *  An error is returned if {@link rclc_executor_t.handles} array is full.
 *  The total number_of_clients field of {@link rclc_executor_t.info}
 *    is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] client 指向已分配且已初始化的客户端的指针
 * \param [in] request_msg 指向已分配请求消息的类型擦除指针
 * \param [in] callback 函数指针，指向回调函数
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 *
 * \param [inout] executor pointer to initialized executor
 * \param [in] client pointer to a allocated and initialized client
 * \param [in] request_msg type-erased ptr to an allocated request message
 * \param [in] callback    function pointer to a callback function
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_client(rclc_executor_t *executor,
                                   rcl_client_t *client,
                                   void *response_msg,
                                   rclc_client_callback_t callback);

/**
 *  向执行器添加客户端。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_clients 字段增加 1。
 *
 *  Adds a client to an executor.
 *  An error is returned if {@link rclc_executor_t.handles} array is full.
 *  The total_number_of_clients field of {@link rclc_executor_t.info}
 *    is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] client 指向已分配和初始化的客户端的指针
 * \param [in] request_msg 指向已分配请求消息的类型擦除指针
 * \param [in] callback 带有 request_id 的回调函数的函数指针
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_client_with_request_id(rclc_executor_t *executor,
                                                   rcl_client_t *client,
                                                   void *response_msg,
                                                   rclc_client_callback_with_request_id_t callback);

/**
 *  向执行器添加服务。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_services 字段增加 1。
 *
 *  Adds a service to an executor.
 *  An error is returned if {@link rclc_executor_t.handles} array is full.
 *  The total_number_of_services field of {@link rclc_executor_t.info}
 *    is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] service 指向已分配和初始化的服务的指针
 * \param [in] request_msg 指向已分配请求消息的类型擦除指针
 * \param [in] response_msg 指向已分配响应消息的类型擦除指针
 * \param [in] callback 回调函数的函数指针
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_service(rclc_executor_t *executor,
                                    rcl_service_t *service,
                                    void *request_msg,
                                    void *response_msg,
                                    rclc_service_callback_t callback);

/**
 *  将动作客户端添加到执行器中。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_action_clients 字段增加 1。
 *
 *  Add an action client to an executor.
 *  An error is returned if {@link rclc_executor_t.handles} array is full.
 *  The total_number_of_action_clients field of {@link rclc_executor_t.info}
 *  is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] action_client 指向已分配和初始化的动作客户端的指针
 * \param [in] handles_number 客户端处理的最大目标数量
 * \param [in] ros_result_response 分配给 ROS 结果消息的类型擦除指针
 * \param [in] ros_feedback 分配给 ROS 反馈消息的类型擦除指针
 * \param [in] goal_callback 目标回调函数的指针
 * \param [in] feedback_callback 反馈回调函数的指针
 * \param [in] result_callback 结果回调函数的指针
 * \param [in] cancel_callback 结果取消回调函数的指针
 * \param [in] context 传递给回调函数的上下文
 *
 * \param [inout] executor pointer to initialized executor
 * \param [in] action_client pointer to a allocated and initialized action client
 * \param [in] handles_number max number of goals to handle with the client
 * \param [in] ros_result_response type-erased ptr to an allocated ROS result message
 * \param [in] ros_feedback type-erased ptr to an allocated ROS feedback message
 * \param [in] goal_callback function pointer to a goal callback
 * \param [in] feedback_callback function pointer to a feedback callback
 * \param [in] result_callback function pointer to a result callback
 * \param [in] cancel_callback function pointer to a result cancel callback
 * \param [in] context context to pass to the callback functions
 *
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 *
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
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
                                          void *context);

/**
 *  将动作服务器添加到执行器中。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_action_servers 字段增加1。
 *
 *  Add an action server to an executor.
 *  An error is returned if {@link rclc_executor_t.handles} array is full.
 *  The total_number_of_action_servers field of {@link rclc_executor_t.info}
 *  is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] action_server 指向已分配和初始化的动作服务器的指针
 * \param [in] handles_number 服务器处理的最大目标数量
 * \param [in] ros_goal_request 指向已分配的 ROS 目标请求消息的类型擦除指针
 * \param [in] ros_goal_request_size ROS 目标请求消息类型的大小
 * \param [in] goal_callback 指向目标请求回调函数的函数指针
 * \param [in] cancel_callback 指向取消请求回调函数的函数指针
 * \param [in] context 传递给回调函数的上下文
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 *
 * \param [inout] executor pointer to initialized executor
 * \param [in] action_server pointer to a allocated and initialized action server
 * \param [in] handles_number max number of goals to handle with the server
 * \param [in] ros_goal_request type-erased ptr to an allocated ROS goal request message
 * \param [in] ros_goal_request_size size of the ROS goal request message type
 * \param [in] goal_callback    function pointer to a goal request callback
 * \param [in] cancel_callback    function pointer to a cancel request callback
 * \param [in] context context to pass to the callback functions
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
rcl_ret_t rclc_executor_add_action_server(
    rclc_executor_t *executor,
    rclc_action_server_t *action_server,
    size_t handles_number,
    void *ros_goal_request,
    size_t ros_goal_request_size,
    rclc_action_server_handle_goal_callback_t goal_callback,
    rclc_action_server_handle_cancel_callback_t cancel_callback,
    void *context);

/**
 *  添加一个服务到执行器。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_services 字段增加1。
 *
 *  Add a service to an executor.
 *  An error is returned if {@link rclc_executor_t.handles} array is full.
 *  The total_number_of_services field of {@link rclc_executor_t.info}
 *  is incremented by one.
 *
 * <hr>
 * 属性              | 遵循
 * ------------------ | -------------
 * 分配内存            | 否
 * 线程安全            | 否
 * 使用原子操作        | 否
 * 无锁                | 是
 *
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] service 指向已分配并初始化的服务的指针
 * \param [in] request_msg 指向已分配请求消息的类型擦除指针
 * \param [in] response_msg 指向已分配响应消息的类型擦除指针
 * \param [in] callback 函数指针，指向带有 request_id 的回调函数
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 *
 * \param [inout] executor pointer to initialized executor
 * \param [in] service pointer to an allocated and initialized service
 * \param [in] request_msg type-erased ptr to an allocated request message
 * \param [in] response_msg type-erased ptr to an allocated response message
 * \param [in] callback    function pointer to a callback function with request_id
 * \return `RCL_RET_OK` if add-operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` if any other error occured
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_service_with_request_id(
    rclc_executor_t *executor,
    rcl_service_t *service,
    void *request_msg,
    void *response_msg,
    rclc_service_callback_with_request_id_t callback);

/**
 *  添加一个服务到执行器中。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_services 字段增加1。
 *
 *  Add a service to an executor.
 *  An error is returned if {@link rclc_executor_t.handles} array is full.
 *  The total_number_of_services field of {@link rclc_executor_t.info}
 *    is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] service 指向已分配和初始化的服务的指针
 * \param [in] request_msg 类型擦除的指向已分配请求消息的指针
 * \param [in] response_msg 类型擦除的指向已分配响应消息的指针
 * \param [in] callback 函数指针，指向带有 request_id 的回调函数
 * \param [in] context 类型擦除的指向附加服务上下文的指针
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_service_with_context(rclc_executor_t *executor,
                                                 rcl_service_t *service,
                                                 void *request_msg,
                                                 void *response_msg,
                                                 rclc_service_callback_with_context_t callback,
                                                 void *context);

/**
 *  添加一个 guard_condition 到执行器中。
 *  如果 {@link rclc_executor_t.handles} 数组已满，则返回错误。
 *  {@link rclc_executor_t.info} 的 total_number_of_guard_conditions 字段增加1。
 *
 *  Adds a guard_condition to an executor.
 *  An error is returned if {@link rclc_executor_t.handles} array is full.
 *  The total number_of_guard_conditions field of {@link rclc_executor_t.info}
 *    is incremented by one.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param [inout] executor 指向已初始化的执行器的指针
 * \param [in] gc 指向已分配和初始化的 guard_condition 的指针
 * \param [in] callback 函数指针，指向回调函数
 * \return `RCL_RET_OK` 如果添加操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_add_guard_condition(rclc_executor_t *executor,
                                            rcl_guard_condition_t *gc,
                                            rclc_gc_callback_t callback);

/**
 * @brief 从执行器中移除一个订阅。(Removes a subscription from an executor.)
 *
 * @details
 * - 如果 {@link rclc_executor_t.handles} 数组为空，则返回错误。(An error is returned if {@link
 * rclc_executor_t.handles} array is empty.)
 * - 如果在 {@link rclc_executor_t.handles} 中找不到订阅，则返回错误。(An error is returned if
 * subscription is not found in {@link rclc_executor_t.handles}.)
 * - {@link rclc_executor_t.info} 的 total number_of_subscriptions 字段减少1。(The total
 * number_of_subscriptions field of {@link rclc_executor_t.info} is decremented by one.)
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * @param [inout] executor 指向已初始化的执行器指针。(pointer to initialized executor)
 * @param [in] subscription 指向先前添加到执行器的已分配和初始化的订阅。(pointer to an allocated and
 * initialized subscription previously added to executor)
 * @return `RCL_RET_OK` 如果删除操作成功。(if remove-operation was successful)
 * @return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针。(if any parameter is a null pointer)
 * @return `RCL_RET_ERROR` 如果发生其他错误。(if any other error occured)
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_remove_subscription(rclc_executor_t *executor,
                                            const rcl_subscription_t *subscription);

/**
 * @brief 从执行器中移除一个定时器。(Removes a timer from an executor.)
 *
 * @details
 * - 如果 {@link rclc_executor_t.handles} 数组为空，则返回错误。(An error is returned if {@link
 * rclc_executor_t.handles} array is empty.)
 * - 如果在 {@link rclc_executor_t.handles} 中找不到定时器，则返回错误。(An error is returned if
 * timer is not found in {@link rclc_executor_t.handles}.)
 * - {@link rclc_executor_t.info} 的 total number_of_timers 字段增加1。(The total number_of_timers
 * field of {@link rclc_executor_t.info} is incremented by one.)
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * @param [inout] executor 指向已初始化的执行器指针。(pointer to initialized executor)
 * @param [in] timer 指向先前添加到执行器的已分配和初始化的定时器。(pointer to an allocated and
 * initialized timer previously added to executor)
 * @return `RCL_RET_OK` 如果删除操作成功。(if remove-operation was successful)
 * @return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针。(if any parameter is a null pointer)
 * @return `RCL_RET_ERROR` 如果发生其他错误。(if any other error occured)
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_remove_timer(rclc_executor_t *executor, const rcl_timer_t *timer);

/**
 * @brief 从执行器中移除客户端 (Removes a client from an executor)
 *
 * - 如果 {@link rclc_executor_t.handles} 数组为空，则返回错误 (An error is returned if {@link
 * rclc_executor_t.handles} array is empty)
 * - 如果在 {@link rclc_executor_t.handles} 中找不到客户端，则返回错误 (An error is returned if
 * client is not found in {@link rclc_executor_t.handles})
 * - {@link rclc_executor_t.info} 的 total_number_of_clients 字段加一 (The total number_of_clients
 * field of {@link rclc_executor_t.info} is incremented by one)
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * @param [inout] executor 指向已初始化的执行器指针 (pointer to initialized executor)
 * @param [in] client 指向之前添加到执行器的已分配和初始化的客户端指针 (pointer to an allocated and
 * initialized client previously added to executor)
 * @return `RCL_RET_OK` 如果添加操作成功 (if add-operation was successful)
 * @return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针 (if any parameter is a null pointer)
 * @return `RCL_RET_ERROR` 如果发生其他错误 (if any other error occured)
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_remove_client(rclc_executor_t *executor, const rcl_client_t *client);

/**
 * @brief 从执行器中移除服务 (Removes a service from an executor)
 *
 * - 如果 {@link rclc_executor_t.handles} 数组为空，则返回错误 (An error is returned if {@link
 * rclc_executor_t.handles} array is empty)
 * - 如果在 {@link rclc_executor_t.handles} 中找不到服务，则返回错误 (An error is returned if
 * service is not found in {@link rclc_executor_t.handles})
 * - {@link rclc_executor_t.info} 的 total_number_of_services 字段加一 (The total number_of_services
 * field of {@link rclc_executor_t.info} is incremented by one)
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * @param [inout] executor 指向已初始化的执行器指针 (pointer to initialized executor)
 * @param [in] service 指向之前添加到执行器的已分配和初始化的服务指针 (pointer to an allocated and
 * initialized service previously added to executor)
 * @return `RCL_RET_OK` 如果添加操作成功 (if add-operation was successful)
 * @return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针 (if any parameter is a null pointer)
 * @return `RCL_RET_ERROR` 如果发生其他错误 (if any other error occured)
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_remove_service(rclc_executor_t *executor, const rcl_service_t *service);

/**
 * @brief 从执行器中移除一个 guard_condition。
 * Removes a guard_condition from an executor.
 *
 * @details
 * * 如果 {@link rclc_executor_t.handles} 数组为空，则返回错误。
 * * An error is returned if {@link rclc_executor_t.handles} array is empty.
 * * 如果在 {@link rclc_executor_t.handles} 中找不到 guard_condition，则返回错误。
 * * An error is returned if guard_condition is not found in {@link rclc_executor_t.handles}.
 * * {@link rclc_executor_t.info} 的 total number_of_guard_conditions 字段增加 1。
 * * The total number_of_guard_conditions field of {@link rclc_executor_t.info}
 *   is incremented by one.
 *
 * <hr>
 * 属性                | 符合性
 * Attribute          | Adherence
 * ------------------ | -------------
 * 分配内存            | 否
 * Allocates Memory   | No
 * 线程安全            | 否
 * Thread-Safe        | No
 * 使用原子操作        | 否
 * Uses Atomics       | No
 * 无锁                | 是
 * Lock-Free          | Yes
 *
 * @param [inout] executor 指向已初始化的执行器的指针
 * \param [inout] executor pointer to initialized executor
 * @param [in] guard_condition 指向先前添加到执行器的已分配且已初始化的 guard_condition 的指针
 * \param [in] guard_condition pointer to an allocated and initialized guard_condition previously
 * added to executor
 * @return 如果添加操作成功，则返回 `RCL_RET_OK`
 * \return `RCL_RET_OK` if add-operation was successful
 * @return 如果任何参数是空指针，则返回 `RCL_RET_INVALID_ARGUMENT`
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * @return 如果发生任何其他错误，则返回 `RCL_RET_ERROR`
 * \return `RCL_RET_ERROR` if any other error occured
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_remove_guard_condition(rclc_executor_t *executor,
                                               const rcl_guard_condition_t *guard_condition);

/**
 *  @brief
 * 准备执行器的等待集，如果无效，则对其进行准备。如果已经准备了有效的等待集，则不执行任何操作。 The
 * executor prepare function prepares the waitset of the executor if it is invalid. Does nothing if
 * a valid waitset is already prepared.
 *
 *  @details 当使用 rcl_wait_set_init() 访问 DDS 队列时，在 rcl 层动态分配内存。
 *  Memory is dynamically allocated within rcl-layer, when DDS queue is accessed with
 *  rcl_wait_set_init()
 *
 * <hr>
 * 属性                  | 符合性
 * ------------------   | -------------
 * 分配内存               | 是
 * 线程安全               | 否
 * 使用原子操作           | 否
 * 无锁                   | 是
 * Attribute            | Adherence
 * Allocates Memory     | Yes
 * Thread-Safe          | No
 * Uses Atomics         | No
 * Lock-Free            | Yes
 *
 *
 * \param [inout] executor 指向已初始化执行器的指针
 * \param [inout] executor pointer to initialized executor
 * \return `RCL_RET_OK` 如果执行器准备操作成功
 * \return `RCL_RET_OK` if executor prepare operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针
 * \return `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer
 * \return `RCL_RET_ERROR` 如果发生其他错误
 * \return `RCL_RET_ERROR` if any other error occurred
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_prepare(rclc_executor_t *executor);

/**
 * @brief spin_some 函数检查 DDS-queue 中的新数据一次。
 *        spin_some function checks one-time for new data from the DDS-queue.
 *
 * @param [inout] executor 指向已初始化的执行器的指针。
 *                 pointer to initialized executor.
 * @param[in] timeout_ns 超时时间（以纳秒为单位）。
 *               timeout in nanoseconds.
 *
 * @return `RCL_RET_OK` 如果 spin_once 操作成功。
 *         `RCL_RET_OK` if spin_once operation was successful.
 * @return `RCL_RET_INVALID_ARGUMENT` 如果任何参数是空指针。
 *         `RCL_RET_INVALID_ARGUMENT` if any parameter is a null pointer.
 * @return `RCL_RET_TIMEOUT` 如果 rcl_wait() 返回超时（即在超时期间没有可用的数据）。
 *         `RCL_RET_TIMEOUT` if rcl_wait() returned timeout (aka no data is available during until
 * the timeout).
 * @return `RCL_RET_ERROR` 如果发生其他错误。
 *         `RCL_RET_ERROR` if any other error occurred.
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_spin_some(rclc_executor_t *executor, const uint64_t timeout_ns);

/**
 *  spin 函数在 ROS 上下文可用的情况下检查 DDS 队列中的新数据。
 *  只要 rcl_context_is_valid() 返回 true，它就会调用 {@link rclc_executor_spin_some()}。
 *
 *  当使用 rcl_wait_set_init()（在 spin_some 函数中）访问 DDS 队列时，
 *  内存会在 rcl 层动态分配。
 *
 * The spin function checks for new data at DDS queue as long as ros context is available.
 * It calls {@link rclc_executor_spin_some()} as long as rcl_context_is_valid() returns true.
 *
 * Memory is dynamically allocated within rcl-layer, when DDS queue is accessed with
 * rcl_wait_set_init() (in spin_some function)
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 *
 * \param [inout] executor 指向已初始化执行器的指针
 * \param [inout] executor pointer to initialized executor
 * \return `RCL_RET_OK` 如果 spin 操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果执行器是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_spin(rclc_executor_t *executor);

/**
 *  spin_period 函数在 ROS 上下文可用的情况下检查 DDS 队列中的新数据。
 *  它每隔一段时间（以纳秒为单位）调用一次。
 *  只要 rcl_context_is_valid() 返回 true，它就会调用 {@link rclc_executor_spin_some()}。
 *
 *  当使用 rcl_wait_set_init()（在 spin_some 函数中）访问 DDS 队列时，
 *  内存会在 rcl 层动态分配。 <hr> Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 *
 * \param [inout] executor 指向已初始化执行器的指针
 * \param [inout] executor pointer to initialized executor
 * \param [in] period 以纳秒为单位的周期
 * \param [in] period in nanoseconds
 * \return `RCL_RET_OK` 如果 spin 操作成功
 * \return `RCL_RET_INVALID_ARGUMENT` 如果执行器是空指针
 * \return `RCL_RET_ERROR` 如果发生其他错误
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_spin_period(rclc_executor_t *executor, const uint64_t period);

/**
 * 为了编写一个单元测试来测试周期持续时间的准确性，我们将 rclc_executor_spin_period 函数拆分。
 * (The reason for splitting up the rclc_executor_spin_period function is to write a unit test for
 * testing the accuracy of the period duration.)
 *
 * rclc_executor_spin_period 是一个无限循环，因此无法在 x 次迭代后停止。
 * (The rclc_executor_spin_period is an endless loop, therefore it is not possible to stop after x
 * iterations.) rclc_executor_spin_one_period 实现了一次迭代。 (The function
 * rclc_executor_spin_one_period implements one iteration.) rclc_executor_spin_period
 * 的单元测试仅涵盖 rclc_executor_spin_one_period。 (The unit test for rclc_executor_spin_period
 * covers only rclc_executor_spin_one_period.)
 *
 * <hr>
 * 属性 (Attribute)          | 符合性 (Adherence)
 * ------------------ | -------------
 * 分配内存 (Allocates Memory)   | 是 (Yes)
 * 线程安全 (Thread-Safe)        | 否 (No)
 * 使用原子操作 (Uses Atomics)       | 否 (No)
 * 无锁 (Lock-Free)          | 是 (Yes)
 *
 *
 * \param [inout] executor 指向已初始化执行器的指针 (pointer to initialized executor)
 * \param [in] period 以纳秒为单位的周期 (period in nanoseconds)
 * \return `RCL_RET_OK` 如果旋转操作成功 (if spin operation was successful)
 * \return `RCL_RET_INVALID_ARGUMENT` 如果执行器是空指针 (if executor is a null pointer)
 * \return `RCL_RET_ERROR` 如果发生任何其他错误 (if any other error occurred)
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_spin_one_period(rclc_executor_t *executor, const uint64_t period);

/**
 * 设置触发条件。
 * (Set the trigger condition.)
 *
 * <hr>
 * 属性 (Attribute)          | 符合性 (Adherence)
 * ------------------ | -------------
 * 分配内存 (Allocates Memory)   | 否 (No)
 * 线程安全 (Thread-Safe)        | 否 (No)
 * 使用原子操作 (Uses Atomics)       | 否 (No)
 * 无锁 (Lock-Free)          | 是 (Yes)
 *
 *
 * \param [inout] executor 指向已初始化执行器的指针 (pointer to initialized executor)
 * \param [in] trigger_function 触发条件的函数 (function of the trigger condition)
 * \param [in] trigger_object 指向触发器中使用的 rcl 句柄的指针 (pointer to a rcl-handle used in the
 * trigger) \return `RCL_RET_OK` 如果旋转操作成功 (if spin operation was successful) \return
 * `RCL_RET_INVALID_ARGUMENT` 如果执行器是空指针 (if executor is a null pointer) \return
 * `RCL_RET_ERROR` 如果发生任何其他错误 (if any other error occurred)
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_set_trigger(rclc_executor_t *executor,
                                    rclc_executor_trigger_t trigger_function,
                                    void *trigger_object);

/**
 * 触发条件：所有，如果所有句柄都准备好了，则返回 true。
 * Trigger condition: all, returns true if all handles are ready.
 *
 * 参数 obj 没有使用。
 * Parameter obj is not used.
 * <hr>
 * 属性              | 遵循
 * Attribute          | Adherence
 * ------------------ | -------------
 * 分配内存            | 否
 * Allocates Memory   | No
 * 线程安全            | 否
 * Thread-Safe        | No
 * 使用原子操作        | 否
 * Uses Atomics       | No
 * 无锁                | 是
 * Lock-Free          | Yes
 *
 * \param [in] handles 指向句柄数组的指针
 * \param [in] handles pointer to array of handles
 * \param [in] size 数组大小
 * \param [in] size size of array
 * \param [in] obj 由 rclc_executor_set_trigger 设置的触发对象（未使用）
 * \param [in] obj trigger_object set by rclc_executor_set_trigger (not used)
 * \return 如果所有句柄都准备好了（订阅有新数据，定时器已准备好），则返回 true
 * \return true - if all handles are ready (subscriptions have new data, timers are ready)
 * \return 否则返回 false
 * \return false - otherwise
 */
RCLC_PUBLIC
bool rclc_executor_trigger_all(rclc_executor_handle_t *handles, unsigned int size, void *obj);

/**
 * 触发条件：任意，如果至少有一个句柄准备好了，则返回 true。
 * Trigger condition: any, returns true if at least one handles is ready.
 *
 * 参数 obj 没有使用。
 * Parameter obj is not used.
 * <hr>
 * 属性              | 遵循
 * Attribute          | Adherence
 * ------------------ | -------------
 * 分配内存            | 否
 * Allocates Memory   | No
 * 线程安全            | 否
 * Thread-Safe        | No
 * 使用原子操作        | 否
 * Uses Atomics       | No
 * 无锁                | 是
 * Lock-Free          | Yes
 *
 * \param [in] handles 指向句柄数组的指针
 * \param [in] handles pointer to array of handles
 * \param [in] size 数组大小
 * \param [in] size size of array
 * \param [in] obj 由 rclc_executor_set_trigger 设置的触发对象（未使用）
 * \param [in] obj trigger_object set by rclc_executor_set_trigger (not used)
 * \return 如果至少有一个句柄准备好了（订阅有新数据，定时器已准备好），则返回 true
 * \return true - if at least one handles is ready (subscriptions have new data, timers are ready)
 * \return 否则返回 false
 * \return false - otherwise
 */
RCLC_PUBLIC
bool rclc_executor_trigger_any(rclc_executor_handle_t *handles, unsigned int size, void *obj);

/**
 * 触发条件：始终触发，总是返回 true。
 * Trigger condition: always, returns always true.
 *
 * 参数 handles、size 和 obj 未使用。
 * Parameter handles, size and obj are not used.
 * <hr>
 * 属性                | 符合性
 * Attribute          | Adherence
 * ------------------ | -------------
 * 分配内存            | 否
 * Allocates Memory   | No
 * 线程安全            | 否
 * Thread-Safe        | No
 * 使用原子操作        | 否
 * Uses Atomics       | No
 * 无锁                | 是
 * Lock-Free          | Yes
 *
 * \param [in] handles 指向句柄数组的指针（未使用）
 * \param [in] handles pointer to array of handles (not used)
 * \param [in] size 数组大小（未使用）
 * \param [in] size size of array (not used)
 * \param [in] obj 由 rclc_executor_set_trigger 设置的触发对象（未使用）
 * \param [in] obj trigger_object set by rclc_executor_set_trigger (not used)
 * \return 总是返回 true
 * \return true always
 */
RCLC_PUBLIC
bool rclc_executor_trigger_always(rclc_executor_handle_t *handles, unsigned int size, void *obj);

/**
 * 触发条件：一次，如果 rcl 句柄 obj 准备好，则返回 true
 * （当 obj 是订阅时，如果有新数据可用，
 *   当 obj 是计时器时，如果计时器准备好）
 * Trigger condition: one, returns true, if rcl handle obj is ready
 * (when obj is a subscription, if new data available,
 *  when obj is a timer, if the timer is ready)
 *
 * <hr>
 * 属性                | 符合性
 * Attribute          | Adherence
 * ------------------ | -------------
 * 分配内存            | 否
 * Allocates Memory   | No
 * 线程安全            | 否
 * Thread-Safe        | No
 * 使用原子操作        | 否
 * Uses Atomics       | No
 * 无锁                | 是
 * Lock-Free          | Yes
 *
 * \param [in] handles 指向句柄数组的指针（未使用）
 * \param [in] handles pointer to array of handles (not used)
 * \param [in] size 数组大小（未使用）
 * \param [in] size size of array (not used)
 * \param [in] obj 由 rclc_executor_set_trigger 设置的触发对象
 * \param [in] obj trigger_object set by rclc_executor_set_trigger
 * \return 如果 rcl 句柄 obj 准备好，则返回 true
 * \return true if rcl-handle obj is ready
 * \return 否则返回 false
 * \return false otherwise
 */
RCLC_PUBLIC
bool rclc_executor_trigger_one(rclc_executor_handle_t *handles, unsigned int size, void *obj);

#if __cplusplus
}
#endif

#endif  // RCLC__EXECUTOR_H_
