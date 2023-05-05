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
#ifndef RCLC__EXECUTOR_HANDLE_H_
#define RCLC__EXECUTOR_HANDLE_H_

#if __cplusplus
extern "C" {
#endif

#include <rcl/rcl.h>
#include <rclc/action_client.h>
#include <rclc/action_server.h>
#include <rclc/visibility_control.h>

/// TODO (jst3si) Where is this defined? - in my build environment this variable is not set.
// #define ROS_PACKAGE_NAME "rclc"

/// @brief 枚举类型，用于等待的计时器、订阅、保护条件等。(Enumeration for timer, subscription, guard
/// conditions etc to be waited on.)
typedef enum {
  RCLC_SUBSCRIPTION,               ///< 订阅。(Subscription)
  RCLC_SUBSCRIPTION_WITH_CONTEXT,  ///< 带上下文的订阅。(Subscription with context)
  RCLC_TIMER,                      ///< 计时器。(Timer)
  // RCLC_TIMER_WITH_CONTEXT,  // TODO: 带上下文的计时器。(Timer with context)
  RCLC_CLIENT,                  ///< 客户端。(Client)
  RCLC_CLIENT_WITH_REQUEST_ID,  ///< 带请求 ID 的客户端。(Client with request ID)
  // RCLC_CLIENT_WITH_CONTEXT,  // TODO: 带上下文的客户端。(Client with context)
  RCLC_SERVICE,                  ///< 服务。(Service)
  RCLC_SERVICE_WITH_REQUEST_ID,  ///< 带请求 ID 的服务。(Service with request ID)
  RCLC_SERVICE_WITH_CONTEXT,     ///< 带上下文的服务。(Service with context)
  RCLC_ACTION_CLIENT,            ///< 动作客户端。(Action client)
  RCLC_ACTION_SERVER,            ///< 动作服务器。(Action server)
  RCLC_GUARD_CONDITION,          ///< 保护条件。(Guard condition)
  // RCLC_GUARD_CONDITION_WITH_CONTEXT,  //TODO: 带上下文的保护条件。(Guard condition with context)
  RCLC_NONE  ///< 无。(None)
} rclc_executor_handle_type_t;

/// 枚举调用类型。当有新数据可用时，ON_NEW_DATA 只调用回调函数
/// ALWAYS 总是调用回调函数，即使没有数据可用（例如 FUNCTION_CALL 类型）
/// Enumeration for invocation type. ON_NEW_DATA calls a callback only when new data is available
/// ALWAYS calls the callback always, even if no data is available (e.g. for type FUNCTION_CALL)
typedef enum { ON_NEW_DATA, ALWAYS } rclc_executor_handle_invocation_t;

/// 订阅回调函数的类型定义
/// - 输入消息
/// Type definition for subscription callback function
/// - incoming message
typedef void (*rclc_subscription_callback_t)(const void *);

/// 订阅回调函数的类型定义（重复）（适用于 foxy 和 galactic 的别名）。
/// - 输入消息
/// Type definition (duplicate) for subscription callback function (alias for foxy and galactic).
/// - incoming message
typedef rclc_subscription_callback_t rclc_callback_t;

/// 订阅回调函数的类型定义
/// - 输入消息
/// - 附加回调上下文
/// Type definition for subscription callback function
/// - incoming message
/// - additional callback context
typedef void (*rclc_subscription_callback_with_context_t)(const void *, void *);

/// 客户端回调函数的类型定义
/// - 请求消息
/// - 响应消息
/// Type definition for client callback function
/// - request message
/// - response message
typedef void (*rclc_service_callback_t)(const void *, void *);

/// 客户端回调函数的类型定义
/// - 请求消息
/// - 请求 ID
/// - 响应消息
/// Type definition for client callback function
/// - request message
/// - request id
/// - response message
typedef void (*rclc_service_callback_with_request_id_t)(const void *, rmw_request_id_t *, void *);

/// 客户端回调函数的类型定义
/// - 请求消息
/// - 响应消息
/// - 附加服务上下文
/// Type definition for client callback function
/// - request message
/// - response message
/// - additional service context
typedef void (*rclc_service_callback_with_context_t)(const void *, void *, void *);

/// 客户端回调函数的类型定义
/// - 响应消息
/// Type definition for client callback function
/// - response message
typedef void (*rclc_client_callback_t)(const void *);

/// 客户端回调函数的类型定义
/// - 响应消息
/// - 请求 ID
/// Type definition for client callback function
/// - response message
/// - request id
typedef void (*rclc_client_callback_with_request_id_t)(const void *, rmw_request_id_t *);

/// 类型定义，用于保护条件回调函数。
/// Type definition for guard condition callback function.
typedef void (*rclc_gc_callback_t)();

/// 句柄容器。
/// Container for a handle.
typedef struct {
  /// 句柄类型。
  /// Type of handle.
  rclc_executor_handle_type_t type;
  /// 调用类型决定何时执行回调。
  /// Invocation type determines when to execute the callback.
  rclc_executor_handle_invocation_t invocation;
  /// 指向句柄的指针。
  /// Pointer to the handle.
  union {
    rcl_subscription_t *subscription;
    rcl_timer_t *timer;
    rcl_client_t *client;
    rcl_service_t *service;
    rcl_guard_condition_t *gc;
    rclc_action_client_t *action_client;
    rclc_action_server_t *action_server;
  };
  /// 存储数据，其中包含订阅、服务等的消息。
  /// Storage of data, which holds the message of a subscription, service, etc.
  /// 订阅：指向消息的指针。
  /// subscription: ptr to message.
  /// 服务：指向请求消息的指针。
  /// service: ptr to request message.
  void *data;

  /// 仅用于类型为 service/client 请求/响应的请求 ID。
  /// request-id only for type service/client request/response.
  rmw_request_id_t req_id;

  /// 仅用于服务 - 指向响应消息的指针。
  /// only for service - ptr to response message.
  void *data_response_msg;

  /// 指向附加回调上下文的指针。
  /// ptr to additional callback context.
  void *callback_context;

  // TODO(jst3si) 新类型作为数据存储
  //              用于服务/客户端对象
  //              查看此结构的内存分配！
  // TODO(jst3si) new type to be stored as data for
  //              service/client objects
  //              look at memory allocation for this struct!
  // struct {
  //   void * request_msg
  //   void * response_msg
  //   rmw_request_id_t req_id;
  //} rclc_service_data_type_t

  /**
   * @brief 存储回调函数的联合体 (Storage for callbacks)
   */
  union {
    rclc_subscription_callback_t subscription_callback;  ///< 订阅回调 (Subscription callback)
    rclc_subscription_callback_with_context_t
        subscription_callback_with_context;  ///< 带上下文的订阅回调 (Subscription callback with
                                             ///< context)
    rclc_service_callback_t service_callback;  ///< 服务回调 (Service callback)
    rclc_service_callback_with_request_id_t
        service_callback_with_reqid;  ///< 带请求ID的服务回调 (Service callback with request ID)
    rclc_service_callback_with_context_t
        service_callback_with_context;  ///< 带上下文的服务回调 (Service callback with context)
    rclc_client_callback_t client_callback;  ///< 客户端回调 (Client callback)
    rclc_client_callback_with_request_id_t
        client_callback_with_reqid;  ///< 带请求ID的客户端回调 (Client callback with request ID)
    rclc_gc_callback_t gc_callback;  ///< 垃圾收集回调 (Garbage collection callback)
  };

  /**
   * @brief 内部变量 (Internal variable)
   *
   * 表示此句柄在相应 wait_set 条目中的索引。
   * (wait_set_subscriptions[index], wait_set_timers[index], ...
   * 范围为 [0, executor.max_handles)，初始化值为: executor_max_handles
   * 因为此值永远不会被分配为 wait_set 中的索引。
   *
   * Denotes the index of this handle in the corresponding wait_set entry.
   * (wait_set_subscriptions[index], wait_set_timers[index], ...
   * is in the range [0, executor.max_handles), initialization: executor_max_handles
   * because this value will never be assigned as an index in the wait_set.
   */
  size_t index;

  /**
   * @brief 内部变量。如果句柄已初始化，则为 true (Internal variable. Flag, which is true, if the
   * handle is initialized)
   */
  bool initialized;

  /**
   * @brief 间隔变量。如果从 DDS 队列中有新数据可用（在调用 rcl_take 之后设置），则为 true
   * (Interval variable. Flag, which is true, if new data is available from DDS queue after calling
   * rcl_take)
   */
  bool data_available;
} rclc_executor_handle_t;

/// 关于订阅、保护条件、定时器、订阅等的总数信息 (Information about total number of subscriptions,
/// guard_conditions, timers, subscription etc.)
typedef struct {
  /// 订阅总数 (Total number of subscriptions)
  size_t number_of_subscriptions;
  /// 定时器总数 (Total number of timers)
  size_t number_of_timers;
  /// 客户端总数 (Total number of clients)
  size_t number_of_clients;
  /// 服务总数 (Total number of services)
  size_t number_of_services;
  /// 动作客户端总数 (Total number of action clients)
  size_t number_of_action_clients;
  /// 动作服务器总数 (Total number of action servers)
  size_t number_of_action_servers;
  /// 保护条件总数 (Total number of guard conditions)
  size_t number_of_guard_conditions;
  /// 事件总数 (Total number of events)
  size_t number_of_events;
} rclc_executor_handle_counters_t;

/**
 * 将每个句柄类型的计数器初始化为零。 (Initializes the counters of each handle type to zero.)
 *
 * <hr>
 * 属性 (Attribute)          | 符合性 (Adherence)
 * ------------------ | -------------
 * 分配内存 (Allocates Memory)   | 否 (No)
 * 线程安全 (Thread-Safe)        | 否 (No)
 * 使用原子操作 (Uses Atomics)       | 否 (No)
 * 无锁 (Lock-Free)          | 是 (Yes)
 *
 * \param[inout] handle_counters 预分配的 rclc_executor_handle_counters_t (preallocated
 * rclc_executor_handle_counters_t) \return 如果 `handle_counters` 是空指针，则返回
 * `RCL_RET_INVALID_ARGUMENT` (Returns `RCL_RET_INVALID_ARGUMENT` if `handle_counters` is a null
 * pointer)
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_handle_counters_zero_init(rclc_executor_handle_counters_t *handle_counters);

/**
 *  使用默认值初始化一个句柄。{@link rclc_executor_handle_t.index}
 *  初始化为 `max_handles`，这是一个非有效索引。请注意，有效索引
 *  是 [0,max-handles-1]。{@link rclc_executor_handle_t.invocation} 设置为 `ON_NEW_DATA`，
 *  这样只有在接收到新数据时才会调用潜在的回调。所有其他成员
 *  字段都设置了适当的默认值，如 `none`、`NULL` 或 `false`。
 *
 *  Initializes a handle with default values. The {@link rclc_executor_handle_t.index}
 *  is initialized with `max_handles`, which is a non-valid index. Note that, valid indicies
 *  are [0,max-handles-1]. The {@link rclc_executor_handle_t.invocation} is set to `ON_NEW_DATA`,
 *  so that a potential callback is invoced only whenever new data is received. All other member
 *  fields are set appropriate default values, like `none`, `NULL` or `false`.
 *
 *  <hr>
 *  属性               | 遵循
 *  ------------------ | -------------
 *  分配内存           | 否
 *  线程安全           | 否
 *  使用原子操作       | 否
 *  无锁               | 是
 *
 *  Attribute          | Adherence
 *  ------------------ | -------------
 *  Allocates Memory   | No
 *  Thread-Safe        | No
 *  Uses Atomics       | No
 *  Lock-Free          | Yes
 *
 *  \param[inout] handle 预分配的 rclc_executor_handle_t
 *  \param[in] max_handles 句柄的最大数量
 *  \return `RCL_RET_OK` 如果 \p handle 初始化成功
 *  \return `RCL_RET_INVALID_ARGUMENT` 如果 \p h 是空指针
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_handle_init(rclc_executor_handle_t *handle, size_t max_handles);

/**
 *  重置 rclc_executor_handle_t。与函数 {@link rclc_executor_handle_init()} 相比，
 *  只有 {@link rclc_executor_handle_t.index} 和 {@link rclc_executor_handle_t.initialized}
 *  变量被重置为默认值。
 *
 *  Resets a rclc_executor_handle_t. Compared to the function  {@link rclc_executor_handle_init()}
 *   only the {@link rclc_executor_handle_t.index} and {@link rclc_executor_handle_t.initialized}
 *   variables are reset to default values.
 *
 * <hr>
 *  属性               | 遵循
 *  ------------------ | -------------
 *  分配内存           | 否
 *  线程安全           | 否
 *  使用原子操作       | 否
 *  无锁               | 是
 *
 *  Attribute          | Adherence
 *  ------------------ | -------------
 *  Allocates Memory   | No
 *  Thread-Safe        | No
 *  Uses Atomics       | No
 *  Lock-Free          | Yes
 *
 *  \param[inout] handle 预分配的 rclc_executor_handle_t
 *  \param[in] max_handles 句柄的最大数量
 *  \return `RCL_RET_OK` 如果 \p h 成功清除
 *  \return `RCL_RET_INVALID_ARGUMENT` 如果 \p h 是空指针
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_handle_clear(rclc_executor_handle_t *handle, size_t max_handles);

/**
 *  打印 rclc_executor_handle_t 类型的名称。
 *  Print out type name of a rclc_executor_handle_t.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[inout] handle 预分配的 rclc_executor_handle_t
 * \param[inout] handle preallocated rclc_executor_handle_t
 * \return `RCL_RET_OK` 如果句柄成功打印
 * \return `RCL_RET_OK` if the handle was printed successfully
 * \return `RCL_RET_INVALID_ARGUMENT` 如果 \p h 是空指针
 * \return `RCL_RET_INVALID_ARGUMENT` if \p h is a null pointer
 */
RCLC_PUBLIC
rcl_ret_t rclc_executor_handle_print(rclc_executor_handle_t *handle);

/**
 *  返回存储在 rclc_executor_handle_t 中的 rcl-handle 的指针。
 *  Returns a pointer to the rcl-handle stored in the rclc_executor_handle_t.
 *  可以是 rcl_subscription_t 或 rcl_timer_t
 *  That can be rcl_subscription_t or rcl_timer_t
 *
 *  如果句柄为 NULL，则返回 NULL。
 *  If handle is NULL, then NULL is returned.
 *
 * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | No
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[inout] handle 预分配的 rclc_executor_handle_t
 * \param[inout] handle preallocated rclc_executor_handle_t
 * \return 指向 rcl-handle 的指针（rcl_subscription_t 或 rcl_timer_t）
 * \return pointer to the rcl-handle (rcl_subscription_t or rcl_timer_t)
 * \return NULL，如果句柄是空指针。
 * \return NULL, if handle is a NULL pointer.
 */
RCLC_PUBLIC
void *rclc_executor_handle_get_ptr(rclc_executor_handle_t *handle);

#if __cplusplus
}
#endif

#endif  // RCLC__EXECUTOR_HANDLE_H_
