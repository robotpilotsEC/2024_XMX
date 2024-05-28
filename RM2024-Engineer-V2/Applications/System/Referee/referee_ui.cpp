/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-22
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-22   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "Core.hpp"

namespace robotpilots {

/**
 * @brief
 * @return
 */
ERpStatus CSysReferee::InitUiDrawing() {

  /* Text - Pump Config */
  pumpTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  pumpTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  pumpTextMsg.message.figureConfig.figureName[2] = 0;    // Figure ID
  pumpTextMsg.message.figureConfig.operate = 1;
  pumpTextMsg.message.figureConfig.figureType = 7;
  pumpTextMsg.message.figureConfig.layerID = 0;
  pumpTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  pumpTextMsg.message.figureConfig.posit_X = 920;
  pumpTextMsg.message.figureConfig.posit_Y = 208,
  pumpTextMsg.message.figureConfig.color = 4;
  pumpTextMsg.message.figureConfig.details_2 = 4;        // String Length
  pumpTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(pumpTextMsg.message.text), "PUMP");

  /* Text - Mode Config */
  modeTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  modeTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  modeTextMsg.message.figureConfig.figureName[2] = 1;    // Figure ID
  modeTextMsg.message.figureConfig.operate = 1;
  modeTextMsg.message.figureConfig.figureType = 7;
  modeTextMsg.message.figureConfig.layerID = 0;
  modeTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  modeTextMsg.message.figureConfig.posit_X = 920;
  modeTextMsg.message.figureConfig.posit_Y = 820,
  modeTextMsg.message.figureConfig.color = 4;
  modeTextMsg.message.figureConfig.details_2 = 4;        // String Length
  modeTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(pumpTextMsg.message.text), "MODE");

  /* Text - CurMode Config */
  modeTextMsg.message.figureConfig.figureName[0] = 0;    // Frame ID
  modeTextMsg.message.figureConfig.figureName[1] = 0;    // Layer ID
  modeTextMsg.message.figureConfig.figureName[2] = 2;    // Figure ID
  modeTextMsg.message.figureConfig.operate = 1;
  modeTextMsg.message.figureConfig.figureType = 7;
  modeTextMsg.message.figureConfig.layerID = 0;
  modeTextMsg.message.figureConfig.posit_X = 920;
  modeTextMsg.message.figureConfig.posit_Y = 780,
  modeTextMsg.message.figureConfig.color = 4;
  modeTextMsg.message.figureConfig.details_1 = 20;       // Font Size
  modeTextMsg.message.figureConfig.details_2 = 4;        // String Length
  modeTextMsg.message.figureConfig.width = 2;            // Line Width
  strcpy(reinterpret_cast<char *>(pumpTextMsg.message.text), "NONE");

  /* Figure - State Config */
  stateFigureMsg.message.figureConfig[0].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[0].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[0].figureName[2] = 3;    // Figure ID
  stateFigureMsg.message.figureConfig[0].operate = 1;
  stateFigureMsg.message.figureConfig[0].figureType = 2;
  stateFigureMsg.message.figureConfig[0].layerID = 1;
  stateFigureMsg.message.figureConfig[0].posit_X = 960;
  stateFigureMsg.message.figureConfig[0].posit_Y = 240,
  stateFigureMsg.message.figureConfig[0].color = 1;
  stateFigureMsg.message.figureConfig[0].details_3 = 10;       // Radius
  stateFigureMsg.message.figureConfig[0].width = 4;            // Line Width

  stateFigureMsg.message.figureConfig[1].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[1].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[1].figureName[2] = 4;    // Figure ID
  stateFigureMsg.message.figureConfig[1].operate = 1;
  stateFigureMsg.message.figureConfig[1].figureType = 2;
  stateFigureMsg.message.figureConfig[1].layerID = 1;
  stateFigureMsg.message.figureConfig[1].posit_X = 920;
  stateFigureMsg.message.figureConfig[1].posit_Y = 240,
  stateFigureMsg.message.figureConfig[1].color = 1;
  stateFigureMsg.message.figureConfig[1].details_3 = 10;       // Radius
  stateFigureMsg.message.figureConfig[1].width = 4;            // Line Width

  stateFigureMsg.message.figureConfig[2].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[2].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[2].figureName[2] = 5;    // Figure ID
  stateFigureMsg.message.figureConfig[2].operate = 1;
  stateFigureMsg.message.figureConfig[2].figureType = 2;
  stateFigureMsg.message.figureConfig[2].layerID = 1;
  stateFigureMsg.message.figureConfig[2].posit_X = 1000;
  stateFigureMsg.message.figureConfig[2].posit_Y = 240,
  stateFigureMsg.message.figureConfig[2].color = 1;
  stateFigureMsg.message.figureConfig[2].details_3 = 10;       // Radius
  stateFigureMsg.message.figureConfig[2].width = 4;            // Line Width

  stateFigureMsg.message.figureConfig[3].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[3].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[3].figureName[2] = 6;    // Figure ID
  stateFigureMsg.message.figureConfig[3].operate = 1;
  stateFigureMsg.message.figureConfig[3].figureType = 0;
  stateFigureMsg.message.figureConfig[3].layerID = 1;
  stateFigureMsg.message.figureConfig[3].posit_X = 350;
  stateFigureMsg.message.figureConfig[3].posit_Y = 200,
  stateFigureMsg.message.figureConfig[3].color = 8;
  stateFigureMsg.message.figureConfig[3].details_3 = 10;       // Radius
  stateFigureMsg.message.figureConfig[3].width = 4;            // Line Width

  stateFigureMsg.message.figureConfig[4].figureName[0] = 0;    // Frame ID
  stateFigureMsg.message.figureConfig[4].figureName[1] = 1;    // Layer ID
  stateFigureMsg.message.figureConfig[4].figureName[2] = 7;    // Figure ID
  stateFigureMsg.message.figureConfig[4].operate = 1;
  stateFigureMsg.message.figureConfig[4].figureType = 0;
  stateFigureMsg.message.figureConfig[4].layerID = 1;
  stateFigureMsg.message.figureConfig[4].posit_X = 1570;
  stateFigureMsg.message.figureConfig[4].posit_Y = 200,
  stateFigureMsg.message.figureConfig[4].color = 1;
  stateFigureMsg.message.figureConfig[4].details_3 = 10;       // Radius
  stateFigureMsg.message.figureConfig[4].width = 4;            // Line Width

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CSysReferee::UpdateUiDrawing_() {

  // Func Freq: 25Hz

  return RP_OK;
}

} // namespace robotpilots
