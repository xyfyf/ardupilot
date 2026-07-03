#include "Copter.h"

// Code to integrate AC_Fence library with main ArduCopter code

#if AP_FENCE_ENABLED

// async fence checking io callback at 1Khz
void Copter::fence_checks_async()
{
    const uint32_t now = AP_HAL::millis();

    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 40U)) { // 25Hz update rate
        return;
    }

    if (fence_breaches.have_updates) {
        return; // wait for the main loop to pick up the new breaches before checking again
    }

    fence_breaches.last_check_ms = now;
    const uint8_t orig_breaches = fence.get_breaches();
    bool is_landing_or_landed = flightmode->is_landing() || ap.land_complete  || !motors->armed();

    // check for new breaches; new_breaches is bitmask of fence types breached
    fence_breaches.new_breaches = fence.check(is_landing_or_landed);

    if (!fence_breaches.new_breaches && orig_breaches && fence.get_breaches() == 0) {
        if (!copter.ap.land_complete) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence breach cleared");
        }
        // record clearing of breach
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
    fence_breaches.have_updates = true; // new breach status latched so main loop will now pick it up
}

// fence_check - ask fence library to check for breaches and initiate the response
// called at 25hz
void Copter::fence_check()
{
    // only take action if there is a new breach
    if (!fence_breaches.have_updates) {
        return;
    }

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached 
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed.
    if (!motors->armed()) {
        fence_breaches.have_updates = false; // fence checking can now be processed again
        return;
    }

    if (fence_breaches.new_breaches) {

        if (!copter.ap.land_complete) {
            fence.print_fence_message("breached", fence_breaches.new_breaches);
        }

        // if the user wants some kind of response and motors are armed
        const auto fence_act = fence.get_action();
        if (fence_act != AC_Fence::Action::REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if (ap.land_complete || (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                arming.disarm(AP_Arming::Method::FENCEBREACH);

            } else {

                // 围栏越界后统一切到 Loiter：依靠 Loiter 自身的刹车 + 围栏避障把飞机拦在围栏内，
                // 同时完整保留飞手摇杆控制权（飞手可自行把飞机打回围栏内），不再夺权切 RTL/Brake/Land。
                // 这样无论 FENCE_ACTION 设为哪个非「仅报警(0)」的值，都不会抢走遥控。
                // 仅当 Loiter 不可用（无 GPS / 定位不可靠）时，才退回原来的失控保护逻辑以保证安全。
                if (!set_mode(Mode::Number::LOITER, ModeReason::FENCE_BREACHED)) {
                    if (!set_mode(Mode::Number::BRAKE, ModeReason::FENCE_BREACHED)) {
                        set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                    }
                }
            }
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));
    }
    fence_breaches.have_updates = false; // fence checking can now be processed again
}

#endif
