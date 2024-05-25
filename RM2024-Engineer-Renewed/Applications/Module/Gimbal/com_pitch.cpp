/**
 * @file        ${PACKAGE_NAME}
 * @version     1.0
 * @date        2024-05-15
 * @author      qianchen
 * @email       
 * @brief       
 *
 * @details
 *
 * @par History:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author          <th>Description
 * <tr><td>2024-05-15   <td>1.0         <td>qianchen  <td>First Create.
 * </table>
 */

#include "mod_gimbal.hpp"

namespace robotpilots {

/**
 * @brief
 * @param pStruct
 * @return
 */
ERpStatus CModGimbal::CComPitch::InitComponent(SModInitParam &param) {

  if (param.moduleId == EModuleID::MOD_NULL) return RP_ERROR;

  /* Initialize Component */
  auto &gimbalParam = reinterpret_cast<SModGimbalInitParam &>(param);
  motor[0] = MotorMap.at(gimbalParam.pitchMotorID);

  /* Initialize PID Controllers */
  gimbalParam.pitchPosPidParam.threadNum = 1;
  pidPosCtrl.InitAlgorithm(&gimbalParam.pitchPosPidParam);

  gimbalParam.pitchSpdPidParam.threadNum = 1;
  pidSpdCtrl.InitAlgorithm(&gimbalParam.pitchSpdPidParam);

  /* Clear Motor Output Buffer */
  mtrOutputBuffer.fill(0);

  /* Set Component Flags */
  processFlag_   = 0;
  componentState = RP_OK;

  return RP_OK;
}


/**
 * @brief
 * @return
 */
ERpStatus CModGimbal::CComPitch::UpdateComponent() {

  if (componentState == RP_RESET) return RP_ERROR;

  /* FSM Status Enum */
  enum {
    PITCH_RESET    = 0,
    PITCH_PRE_INIT = 1,
    PITCH_INIT,
    PITCH_CTRL,
  };

  /* Gimbal Pitch Component Control FSM */
  switch (processFlag_) {

    case PITCH_RESET: {

      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();
      return RP_OK;
    }

    case PITCH_PRE_INIT: {

      pitchCmd.setPosit = static_cast<int32_t>(rangeLimit * 1.2);
      motor[0]->motorData[CMtrInstance::DATA_POSIT] = -pitchCmd.setPosit;
      mtrOutputBuffer.fill(0);
      pidPosCtrl.ResetAlgorithm();
      pidSpdCtrl.ResetAlgorithm();

      processFlag_ = PITCH_INIT;   // Enter LIFT_INIT
      return RP_OK;
    }

    case PITCH_INIT: {

      if (motor[0]->motorState == CMtrInstance::EMotorStatus::STALL) {
        motor[0]->motorData[CMtrInstance::DATA_POSIT] = +8192 * 1.0;
        pidPosCtrl.ResetAlgorithm();
        pidSpdCtrl.ResetAlgorithm();

        pitchCmd.setPosit = 0;
        componentState    = RP_OK;
        processFlag_      = PITCH_CTRL;// Enter PITCH_CTRL
        return RP_OK;
      }

      pitchCmd.setPosit -= 200;   // Reset Speed
      return _UpdateOutput(static_cast<float_t>(pitchCmd.setPosit));
    }

    case PITCH_CTRL: {

      return RP_OK;
    }

    default: {
      StopComponent();
      componentState = RP_ERROR;
      return RP_ERROR;
    }
  }
}


/**
 * @brief
 * @param posit
 * @return
 */
ERpStatus CModGimbal::CComPitch::_UpdateOutput(float_t posit) {

  DataBuffer<float_t> liftPos = {
    static_cast<float_t>(-posit),
  };

  DataBuffer<float_t> liftPosMeasure = {
    static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_POSIT]),
  };

  auto liftSpd =
    pidPosCtrl.UpdatePidController(liftPos, liftPosMeasure);

  DataBuffer<float_t> liftSpdMeasure = {
    static_cast<float_t>(motor[0]->motorData[CMtrInstance::DATA_SPEED]),
  };

  auto output =
    pidSpdCtrl.UpdatePidController(liftSpd, liftSpdMeasure);

  mtrOutputBuffer = {
    static_cast<int16_t>(output[0]),
  };

  return RP_OK;
}

} // namespace robotpilots
