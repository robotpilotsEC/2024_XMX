/**
 * @file        conf_interface.cpp
 * @version     1.0
 * @date        2024-03-15
 * @author      Morthine Xiang
 * @email       xiang@morthine.com
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-15   <td>1.0         <td>Morthine Xiang  <td>First Create.
 * </table>
 */

#include "conf_interface.hpp"
#include "Interface.hpp"

/* Extern HAL Handle Variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart10;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern SPI_HandleTypeDef hspi2;

namespace robotpilots {

/**
 * @brief Initialize Interface
 *
 * @return RP_OK - Success
 * @return RP_ERROR - Error
 */
ERpStatus InitializeInterface() {

  /* DEBUG (UART10) */
  static CUartInterface inf_debug;
  CUartInterface::SUartInfInitParam inf_debug_initparam;
  inf_debug_initparam.interfaceId = EInterfaceID::INF_UART10;
  inf_debug_initparam.halUartHandle = &huart10;
  inf_debug_initparam.useTxHandle = true;
  inf_debug_initparam.txQueueLength = 4;
  inf_debug_initparam.txBufferSize = 256;
  inf_debug_initparam.useRxHandle = false;
  inf_debug.InitInterface(&inf_debug_initparam);

  /* DBUS (UART5) */
  static CUartInterface inf_dbus;
  CUartInterface::SUartInfInitParam inf_dbus_initparam;
  inf_dbus_initparam.interfaceId = EInterfaceID::INF_DBUS;
  inf_dbus_initparam.halUartHandle = &huart5;
  inf_dbus_initparam.useRxHandle = true;
  inf_dbus_initparam.rxQueueLength = 2;
  inf_dbus_initparam.rxBufferSize = 64;
  inf_dbus.InitInterface(&inf_dbus_initparam);

  /* Referee (UART1) */
  static CUartInterface inf_referee;
  CUartInterface::SUartInfInitParam inf_referee_initparam;
  inf_referee_initparam.interfaceId = EInterfaceID::INF_UART1;
  inf_referee_initparam.halUartHandle = &huart1;
  inf_referee_initparam.useRxHandle = true;
  inf_referee_initparam.rxQueueLength = 2;
  inf_referee_initparam.rxBufferSize = 256;
  inf_referee_initparam.useTxHandle = true;
  inf_referee_initparam.txQueueLength = 4;
  inf_referee_initparam.txBufferSize = 256;
  inf_referee.InitInterface(&inf_referee_initparam);

  /* Vision (UART7) */
  static CUartInterface inf_vision;
  CUartInterface::SUartInfInitParam inf_vision_initparam;
  inf_vision_initparam.interfaceId = EInterfaceID::INF_UART7;
  inf_vision_initparam.halUartHandle = &huart7;
  inf_vision_initparam.useRxHandle = true;
  inf_vision_initparam.rxQueueLength = 2;
  inf_vision_initparam.rxBufferSize = 256;
  inf_vision_initparam.useTxHandle = true;
  inf_vision_initparam.txQueueLength = 4;
  inf_vision_initparam.txBufferSize = 256;
  inf_vision.InitInterface(&inf_vision_initparam);

  /* CAN1 */
  static CCanInterface inf_can1;
  CCanInterface::SCanInfInitParam inf_can1_initparam;
  inf_can1_initparam.interfaceId = EInterfaceID::INF_CAN1;
  inf_can1_initparam.halFdcanHandle = &hfdcan1;
  inf_can1.InitInterface(&inf_can1_initparam);

  /* CAN2 */
  static CCanInterface inf_can2;
  CCanInterface::SCanInfInitParam inf_can2_initparam;
  inf_can2_initparam.interfaceId = EInterfaceID::INF_CAN2;
  inf_can2_initparam.halFdcanHandle = &hfdcan2;
  inf_can2.InitInterface(&inf_can2_initparam);

  /* CAN3 */
  static CCanInterface inf_can3;
  CCanInterface::SCanInfInitParam inf_can3_initparam;
  inf_can3_initparam.interfaceId = EInterfaceID::INF_CAN3;
  inf_can3_initparam.halFdcanHandle = &hfdcan3;
  inf_can3.InitInterface(&inf_can3_initparam);

  /* SPI2 */
  static CSpiInterface inf_spi2;
  CSpiInterface::SSpiInfInitParam inf_spi2_initparam;
  inf_spi2_initparam.interfaceId = EInterfaceID::INF_SPI2;
  inf_spi2_initparam.halSpiHandle = &hspi2;
  inf_spi2.InitInterface(&inf_spi2_initparam);

  return RP_OK;
}

} // namespace robotpilots
