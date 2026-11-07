# VTOL runway takeoff

The fixed-wing mode manager retains a one-shot runway-start qualification from stationary pre-arm estimator data, despite the VTOL land detector reporting airborne when armed in fixed-wing mode. Global land detection is unchanged. Longitudinal control now honors the requested underspeed-protection suppression and restores FW_PSP_OFF on TECS pitch output; direct pitch commands remain unchanged.

## Basis

[Cymberly’s analysis](https://pennaerial.slack.com/archives/D0AGF122GAX/p1793954025372319) reports immediate nose-down pitch during a tailsitter runway takeoff on fdccc2fc. Source inspection confirms the armed fixed-wing VTOL land report, takeoff bypass, dropped configuration flag, and missing offset conversion. The original ULog was not independently reanalyzed for this change.

## Runway-start qualification

With runway takeoff enabled on a VTOL, qualification requires one second disarmed with observed landed state and valid, finite local XYZ position and velocity. Local position must be at most 0.5 seconds old; stable land-detector reports arrive at 1 Hz and may be at most two seconds old. Horizontal speed must not exceed 1 m/s, vertical speed 0.5 m/s, displacement from the qualification anchor 5 m horizontally or 1 m vertically. Estimator resets restart qualification.

Arming in fixed-wing mode preserves this permission. Transition, armed rotary-wing operation, offboard/external control, invalid/stale data, motion beyond those limits, or disabling runway takeoff discards it. An armed takeoff consumes it once, including when takeoff was selected before arming. Requalification requires disarming. Other airborne takeoff entries retain the original bypass. Valid XY is required for this exception even for a subsequent no-navigation takeoff.

This is stationary pre-arm estimator evidence, not independent proof of physical ground contact. No new takeoff-abort or throttle-cut behavior is added.

## Aircraft settings and validation limits

Cymberly reports RWTO_ROT_GSPD=3 m/s against FW_AIRSPD_STALL=17 m/s and an 18 m/s takeoff airspeed target. No replacement threshold or parameter change is selected here: the aircraft team must reconcile rotation speed, wind conditions, and TECS airspeed configuration. The intended +8 degree ground pitch, 0.60 maximum runway throttle, and three-second ramp depend on the loaded aircraft parameters; these are not new firmware defaults.

Native regression tests cover the runway qualification and GPS rotation logic. The hardware target is px4_fmu-v6c_pennair_advanced. Simulator and physical-aircraft validation are not performed as part of this change. Compilation and unit tests do not establish flight readiness.
