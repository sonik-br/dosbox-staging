// SPDX-FileCopyrightText:  2002-2026 The DOSBox Staging Team
// SPDX-License-Identifier: GPL-2.0-or-later

// Joystick force-feedback output. A wrapper around SDL haptics that lets
// emulated force-feedback devices (e.g. the serial Immersion I-FORCE joystick)
// drive a real gamepad, racing wheel or joystick.
//
// Two output modes:
//   - Rumble: simple vibration motors (gamepads). Driven by PlayRumble /
//     SetSustainedRumble.
//   - Wheel:  true force feedback (wheels/joysticks). Constant force, spring
//     conditions, and periodic vibration via SDL haptic effects.
//
// NOTE: Single global device and effect state, so only one
// force-feedback device can be driven at a time; concurrent
// callers would share (and overwrite) each other's effects.

#ifndef DOSBOX_JOYSTICK_FFB_H
#define DOSBOX_JOYSTICK_FFB_H

#include <cstdint>

namespace JoyFfb {

// Rumble: gamepad vibration motors. Wheel: true force feedback, steering
// (X) axis only. Joystick: true force feedback on both X and Y axes.
enum class Mode { Off, Rumble, Wheel, Joystick };

// Open the first capable device for the requested mode, applying an overall
// strength of 0.0..1.0. Idempotent; returns true if a usable device is
// available afterwards.
bool Open(Mode mode, float strength);

// Release the device.
void Close();

// True once a device is open and usable for the active mode.
bool IsOpen();

// --- Rumble mode ---

// Transient rumble for impacts/jolts. strength is clamped to 0.0..1.0.
void PlayRumble(float strength, uint32_t duration_ms);

// Sustained rumble level for continuous vibration; 0 stops it.
void SetSustainedRumble(float strength);

// --- Wheel/Joystick (true force-feedback) mode ---

// Sustained constant force. x and y are -1.0..1.0 (joystick orientation:
// +x = right, +y = forward); (0, 0) clears it.
void SetConstantForce(float x, float y);

// Transient directional kick (jolt) of the given magnitude/direction.
void PlayConstantPulse(float x, float y, uint32_t duration_ms);

// Centering spring with independent stiffness per direction. center is
// -1.0..1.0; left/right coefficients are 0.0..1.0 stiffness for each side of
// each axis. Passing all-zero coefficients clears the spring.
void SetSpring(float center_x, float left_x, float right_x,
               float center_y, float left_y, float right_y);

// Periodic (sine) vibration. strength 0.0..1.0, frequency in Hz; 0 stops it.
void SetPeriodic(float strength, float frequency_hz);

// Stop every active effect.
void StopAll();

} // namespace JoyFfb

#endif // DOSBOX_JOYSTICK_FFB_H
