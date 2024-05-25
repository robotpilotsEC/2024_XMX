/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-03-22
 * @author      qianchen
 * @email
 * @brief
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-03-22   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "System.hpp"
#include "Module.hpp"

namespace robotpilots {

/**
 * @brief
 */
std::map<ESystemID, CSysInstance *> SystemMap;


/**
   * @brief
   * @return
   */
ERpStatus CSysInstance::RegisterSystem_() {

  if (systemID == ESystemID::SYS_NULL) return RP_ERROR;

  SystemMap.insert(std::make_pair(systemID, this));
  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CSysInstance::UnregisterSystem_() {

  if (systemID == ESystemID::SYS_NULL) return RP_ERROR;

  SystemMap.erase(systemID);
  return RP_OK;
}

} // namespace robotpilots
