/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-25
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-25   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "Core.hpp"

namespace robotpilots {

/**
 * @brief Monitor Task Entrypoint
 * @param arg[in] Not Used
 */
void StartMonitorTask(void *arg) {

  auto *uart = reinterpret_cast<CUartInterface *>(InterfaceMap.at(EInterfaceID::INF_UART10));

  int32_t mtrAngle[2];

  /* Monitor Task */
  while (true) {

    mtrAngle[0] = MotorMap.at(EDeviceID::DEV_LIFT_MTR_L)->motorData[CMtrInstance::DATA_POSIT];
    mtrAngle[1] = MotorMap.at(EDeviceID::DEV_LIFT_MTR_R)->motorData[CMtrInstance::DATA_POSIT];

    uart->FormatTransmit("L: %d, R %d\r\n",
                         mtrAngle[0], mtrAngle[1]);

    proc_waitMs(100);  // 10Hz
  }
}

} // namespace robotpilots
