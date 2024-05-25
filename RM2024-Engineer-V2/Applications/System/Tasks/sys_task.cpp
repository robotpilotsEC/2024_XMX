/**
 * @file        sys_task.cpp
 * @version     1.0
 * @date        2024-03-31
 * @author      Morthine Xiang
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-31   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#include "Core.hpp"

#define deg2rad (3.1415926f / 180.0f)

namespace robotpilots {

void JointAngleToEulerAngle (const float32_t *jointAngle, float32_t *eulerAngle);


/**
 * @brief Monitor Task Entrypoint
 * @param arg[in] Not Used
 */
void StartMonitorTask(void *arg) {

  auto *uart = reinterpret_cast<CUartInterface *>(InterfaceMap.at(EInterfaceID::INF_UART10));

  int32_t mtrAngle;

  /* Monitor Task */
  while (true) {

    mtrAngle = MotorMap.at(EDeviceID::DEV_MANIP_MTR_ROLL)->motorData[CMtrInstance::DATA_ANGLE];

    uart->FormatTransmit("Joint Angle: %d\r\n",
                         mtrAngle);

    proc_waitMs(100);  // 10Hz
  }
}


/**
 * @brief System Update Task Entrypoint
 * @param arg[in] Not Used
 */
void StartSystemUpdateTask(void *arg) {

  /* System Update Task */
  while (true) {

    /* System Update */
    for (const auto &item: SystemMap)
      item.second->UpdateHandler_();

    proc_waitMs(4);   // 250Hz
  }
}


/**
 * @brief Update Task Entrypoint
 * @param arg[in] Not Used
 */
void StartUpdateTask(void *arg) {

  /* System Core Initialization */
  SystemCore.InitSystemCore();

  /* Update Task */
  while (true) {

    /* Device Update */
    for (const auto &item : DeviceMap)
      item.second->UpdateHandler_();

    /* System Core Update */
    SystemCore.UpdateHandler_();

    /* Module Update */
    for (const auto &item : ModuleMap)
      item.second->UpdateHandler_();

    proc_waitMs(1);   // 1000Hz
  }
}


/**
 * @brief Heartbeat Task Entrypoint
 * @param arg[in] Not Used
 */
void StartHeartbeatTask(void *arg) {

  /* Heartbeat Task */
  while (true) {

    /* Interface Heartbeat */
    for (const auto &item : InterfaceMap)
      item.second->HeartbeatHandler_();

    /* Device Heartbeat */
    for (const auto &item : DeviceMap)
      item.second->HeartbeatHandler_();

    /* System Heartbeat */
    for (const auto &item: SystemMap)
      item.second->HeartbeatHandler_();

    /* System Core Heartbeat */
    SystemCore.HeartbeatHandler_();

    /* Module Heartbeat */
    for (const auto &item : ModuleMap)
      item.second->HeartbeatHandler_();

    proc_waitMs(10);  // 100Hz
  }
}

} // namespace robotpilots