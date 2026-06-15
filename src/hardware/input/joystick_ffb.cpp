// SPDX-FileCopyrightText:  2002-2026 The DOSBox Staging Team
// SPDX-License-Identifier: GPL-2.0-or-later

#include "joystick_ffb.h"

#include <algorithm>
#include <cmath>

#include <SDL.h>

#include "misc/logging.h"
#include "utils/checks.h"

CHECK_NARROWING();

namespace JoyFfb {

namespace {
SDL_Haptic* haptic     = nullptr; // used by both modes
SDL_Joystick* joystick = nullptr; // rumble-mode fallback only

Mode active_mode    = Mode::Off;
bool init_attempted = false;
float global_gain   = 1.0f;
unsigned int features = 0; // SDL_HapticQuery() capability bits

// Running effect ids for wheel mode (-1 = not created)
int constant_id = -1;
int pulse_id    = -1;
int spring_id   = -1;
int periodic_id = -1;

// Rumble-mode bookkeeping
float sustained_rumble = 0.0f;
float applied_rumble   = -1.0f;
constexpr uint32_t SustainedRefreshMs = 1000;

// True for the modes that drive real force-feedback effects (as opposed to
// simple rumble).
bool IsForceMode()
{
	return active_mode == Mode::Wheel || active_mode == Mode::Joystick;
}

float Clamp01(const float v)
{
	return std::clamp(v, 0.0f, 1.0f);
}

int16_t ToLevel(const float unit) // 0.0..1.0 -> 0..32767
{
	return static_cast<int16_t>(Clamp01(unit) * 32767.0f);
}

// --- Rumble mode device selection ---

bool TryOpenHapticRumble()
{
	const int count = SDL_NumHaptics();
	for (int i = 0; i < count; ++i) {
		SDL_Haptic* h = SDL_HapticOpen(i);
		if (!h) {
			continue;
		}
		if (SDL_HapticRumbleSupported(h) == SDL_TRUE &&
		    SDL_HapticRumbleInit(h) == 0) {
			haptic = h;
			LOG_MSG("FFB: Rumble via force-feedback device '%s'",
			        SDL_HapticName(i));
			return true;
		}
		SDL_HapticClose(h);
	}
	return false;
}

bool TryOpenJoystickRumble()
{
	const int count = SDL_NumJoysticks();
	for (int i = 0; i < count; ++i) {
		SDL_Joystick* j = SDL_JoystickOpen(i);
		if (!j) {
			continue;
		}
		if (SDL_JoystickRumble(j, 0, 0, 0) == 0) {
			joystick = j;
			LOG_MSG("FFB: Rumble via gamepad '%s'", SDL_JoystickName(j));
			return true;
		}
		SDL_JoystickClose(j);
	}
	return false;
}

// --- Wheel/joystick mode device selection ---

bool TryOpenWheel()
{
	const int count = SDL_NumHaptics();
	for (int i = 0; i < count; ++i) {
		SDL_Haptic* h = SDL_HapticOpen(i);
		if (!h) {
			continue;
		}
		const unsigned int caps = SDL_HapticQuery(h);
		if (caps & SDL_HAPTIC_CONSTANT) {
			haptic   = h;
			features = caps;
			// Disable the driver's built-in autocentre spring so it does
			// not fight the game's own constant/spring forces.
			SDL_HapticSetAutocenter(h, 0);
			LOG_MSG("FFB: Force feedback via wheel/joystick '%s'",
			        SDL_HapticName(i));
			LOG_MSG("FFB: Supported effects:%s%s%s%s%s",
			        (caps & SDL_HAPTIC_CONSTANT) ? " constant" : "",
			        (caps & SDL_HAPTIC_SPRING) ? " spring" : "",
			        (caps & SDL_HAPTIC_SINE) ? " sine" : "",
			        (caps & SDL_HAPTIC_DAMPER) ? " damper" : "",
			        (caps & SDL_HAPTIC_FRICTION) ? " friction" : "");
			return true;
		}
		SDL_HapticClose(h);
	}
	return false;
}

void DestroyEffect(int& id)
{
	if (id >= 0 && haptic) {
		SDL_HapticDestroyEffect(haptic, id);
	}
	id = -1;
}

// Create-or-update a running effect from a filled-in template.
void RunEffect(int& id, SDL_HapticEffect& effect, const bool retrigger)
{
	if (id < 0) {
		id = SDL_HapticNewEffect(haptic, &effect);
		if (id < 0) {
			LOG_WARNING("FFB: SDL_HapticNewEffect failed: %s",
			            SDL_GetError());
			return;
		}
		if (SDL_HapticRunEffect(haptic, id, 1) != 0) {
			LOG_WARNING("FFB: SDL_HapticRunEffect failed: %s",
			            SDL_GetError());
		}
	} else {
		if (SDL_HapticUpdateEffect(haptic, id, &effect) != 0) {
			LOG_WARNING("FFB: SDL_HapticUpdateEffect failed: %s",
			            SDL_GetError());
		}
		if (retrigger) {
			SDL_HapticRunEffect(haptic, id, 1);
		}
	}
}
} // namespace

bool Open(const Mode mode, const float strength)
{
	global_gain = Clamp01(strength);

	if ((haptic || joystick) && active_mode == mode) {
		return true; // already open in the right mode
	}
	if (active_mode != mode) {
		Close(); // mode change -> reopen
	}
	if (init_attempted && !haptic && !joystick) {
		return false; // already tried and found nothing
	}
	init_attempted = true;

	if (mode == Mode::Off) {
		return false;
	}

	if (SDL_WasInit(SDL_INIT_HAPTIC) != SDL_INIT_HAPTIC) {
		SDL_InitSubSystem(SDL_INIT_HAPTIC);
	}
	if (SDL_WasInit(SDL_INIT_JOYSTICK) != SDL_INIT_JOYSTICK) {
		SDL_InitSubSystem(SDL_INIT_JOYSTICK);
	}

	bool ok = false;
	if (mode == Mode::Wheel || mode == Mode::Joystick) {
		ok = TryOpenWheel();
		if (!ok) {
			LOG_WARNING("FFB: No force-feedback wheel/joystick found "
			            "(try 'ffb:rumble' for a gamepad)");
		}
	} else { // Rumble
		ok = TryOpenHapticRumble() || TryOpenJoystickRumble();
		if (!ok) {
			LOG_WARNING("FFB: No rumble-capable joystick or wheel found");
		}
	}

	active_mode = ok ? mode : Mode::Off;
	return ok;
}

bool IsOpen()
{
	return active_mode != Mode::Off && (haptic || joystick);
}

// ---------------------------------------------------------------------------
// Rumble mode
// ---------------------------------------------------------------------------

void PlayRumble(const float strength, const uint32_t duration_ms)
{
	if (active_mode != Mode::Rumble) {
		return;
	}
	const float level = std::max(Clamp01(strength) * global_gain,
	                             sustained_rumble);
	if (haptic) {
		if (level > 0.0f) {
			SDL_HapticRumblePlay(haptic, level, duration_ms);
		}
	} else if (joystick) {
		const auto motor = static_cast<uint16_t>(level * 0xffff);
		SDL_JoystickRumble(joystick, motor, motor, duration_ms);
	}
	applied_rumble = -1.0f; // make the next sustained update re-apply
}

void SetSustainedRumble(const float strength)
{
	if (active_mode != Mode::Rumble) {
		return;
	}
	sustained_rumble = Clamp01(strength) * global_gain;
	if (sustained_rumble == applied_rumble) {
		return;
	}
	applied_rumble = sustained_rumble;

	if (haptic) {
		if (sustained_rumble > 0.0f) {
			SDL_HapticRumblePlay(haptic, sustained_rumble, SDL_HAPTIC_INFINITY);
		} else {
			SDL_HapticRumbleStop(haptic);
		}
	} else if (joystick) {
		const auto motor = static_cast<uint16_t>(sustained_rumble * 0xffff);
		SDL_JoystickRumble(joystick, motor, motor, SustainedRefreshMs);
	}
}

// ---------------------------------------------------------------------------
// Wheel/Joystick mode
// ---------------------------------------------------------------------------

void SetConstantForce(const float x, const float y)
{
	if (!IsForceMode() || !haptic || !(features & SDL_HAPTIC_CONSTANT)) {
		return;
	}
	const float mag = std::min(std::hypot(x, y), 1.0f) * global_gain;

	SDL_HapticEffect e = {};
	e.type                   = SDL_HAPTIC_CONSTANT;
	e.constant.direction.type = SDL_HAPTIC_CARTESIAN;
	e.constant.direction.dir[0] = static_cast<int32_t>(x * 10000.0f);
	e.constant.direction.dir[1] = static_cast<int32_t>(y * 10000.0f);
	e.constant.length        = SDL_HAPTIC_INFINITY;
	e.constant.level         = ToLevel(mag);

	RunEffect(constant_id, e, false);
}

void PlayConstantPulse(const float x, const float y, const uint32_t duration_ms)
{
	if (!IsForceMode() || !haptic || !(features & SDL_HAPTIC_CONSTANT)) {
		return;
	}
	const float mag = std::min(std::hypot(x, y), 1.0f) * global_gain;

	SDL_HapticEffect e = {};
	e.type                   = SDL_HAPTIC_CONSTANT;
	e.constant.direction.type = SDL_HAPTIC_CARTESIAN;
	e.constant.direction.dir[0] = static_cast<int32_t>(x * 10000.0f);
	e.constant.direction.dir[1] = static_cast<int32_t>(y * 10000.0f);
	e.constant.length        = static_cast<uint32_t>(duration_ms);
	e.constant.level         = ToLevel(mag);

	RunEffect(pulse_id, e, true);
}

void SetSpring(const float center_x, const float left_x, const float right_x,
               const float center_y, const float left_y, const float right_y)
{
	if (!IsForceMode() || !haptic) {
		return;
	}
	if (!(features & SDL_HAPTIC_SPRING)) {
		static bool warned = false;
		if (!warned) {
			warned = true;
			LOG_WARNING("FFB: device has no spring effect; "
			            "XSpring/YSpring will produce no force");
		}
		return;
	}
	if (left_x <= 0.0f && right_x <= 0.0f && left_y <= 0.0f && right_y <= 0.0f) {
		DestroyEffect(spring_id);
		return;
	}

	auto to_coeff = [](const float v) {
		return static_cast<int16_t>(Clamp01(v) * global_gain * 32767.0f);
	};

	SDL_HapticEffect e = {};
	e.type                    = SDL_HAPTIC_SPRING;
	e.condition.direction.type = SDL_HAPTIC_CARTESIAN;
	e.condition.direction.dir[0] = 1;
	e.condition.length        = SDL_HAPTIC_INFINITY;

	const float left[2]   = {left_x, left_y};
	const float right[2]  = {right_x, right_y};
	const float center[2] = {center_x, center_y};
	for (int axis = 0; axis < 2; ++axis) {
		e.condition.left_coeff[axis]  = to_coeff(left[axis]);
		e.condition.right_coeff[axis] = to_coeff(right[axis]);
		e.condition.right_sat[axis]   = 0xffff;
		e.condition.left_sat[axis]    = 0xffff;
		e.condition.center[axis] = static_cast<int16_t>(
		        std::clamp(center[axis], -1.0f, 1.0f) * 32767.0f);
	}

	RunEffect(spring_id, e, false);
}

void SetPeriodic(const float strength, const float frequency_hz)
{
	if (!IsForceMode() || !haptic || !(features & SDL_HAPTIC_SINE)) {
		return;
	}
	if (strength <= 0.0f) {
		DestroyEffect(periodic_id);
		return;
	}
	const auto period_ms = static_cast<uint16_t>(
	        frequency_hz > 0.0f ? std::clamp(1000.0f / frequency_hz, 1.0f, 1000.0f)
	                            : 50.0f);

	SDL_HapticEffect e = {};
	e.type                     = SDL_HAPTIC_SINE;
	e.periodic.direction.type  = SDL_HAPTIC_CARTESIAN;
	e.periodic.direction.dir[0] = 1;
	e.periodic.length          = SDL_HAPTIC_INFINITY;
	e.periodic.period          = period_ms;
	e.periodic.magnitude       = ToLevel(Clamp01(strength) * global_gain);

	RunEffect(periodic_id, e, false);
}

void StopAll()
{
	sustained_rumble = 0.0f;
	applied_rumble   = 0.0f;

	if (haptic) {
		if (active_mode == Mode::Rumble) {
			SDL_HapticRumbleStop(haptic);
		} else {
			DestroyEffect(constant_id);
			DestroyEffect(pulse_id);
			DestroyEffect(spring_id);
			DestroyEffect(periodic_id);
		}
	} else if (joystick) {
		SDL_JoystickRumble(joystick, 0, 0, 0);
	}
}

void Close()
{
	StopAll();
	if (haptic) {
		SDL_HapticClose(haptic);
		haptic = nullptr;
	}
	if (joystick) {
		SDL_JoystickClose(joystick);
		joystick = nullptr;
	}
	active_mode    = Mode::Off;
	init_attempted = false;
	features       = 0;
	applied_rumble = -1.0f;
}

} // namespace JoyFfb
