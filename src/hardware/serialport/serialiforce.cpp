// SPDX-FileCopyrightText:  2002-2026 The DOSBox Staging Team
// SPDX-License-Identifier: GPL-2.0-or-later

#include "serialiforce.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "hardware/input/joystick.h"
#include "hardware/input/joystick_ffb.h"
#include "shell/command_line.h"
#include "utils/checks.h"
#include "utils/string_utils.h"

CHECK_NARROWING();

// Set to 1 to log every raw byte the guest writes to the port, plus frames that
// fail the checksum. Diagnostic aid for figuring out whether a game is talking
// to the port at all, and whether it uses the expected frame format. Leave off
// for normal use; the decoded per-command logging (LogIForce) is enough.
#define IFORCE_LOG_RAW 0

namespace {
// Set to false to silence the per-command I-FORCE console logging. The
// periodic status broadcasts and position reports are never logged regardless,
// to avoid flooding the console.
constexpr bool LogIForce = true;
// Frame protocol
constexpr uint8_t StartChar = '+';

// Custom periodic event types (must be > SERIAL_BASE_EVENT_COUNT and not
// collide with the predefined SERIAL_*_EVENT identifiers).
constexpr uint16_t IForceStatusEvent     = 100;
constexpr uint16_t IForcePositionEvent   = 101;
constexpr uint16_t IForceButtonPollEvent = 102;

// The real joystick broadcasts its status roughly every two seconds.
constexpr float StatusPeriodMs       = 2000.0f;
constexpr uint8_t MinPositionPeriodMs = 20;

// How often button-reflex states are polled to fire/repeat jolts.
constexpr float ButtonPollIntervalMs = 20.0f;

// --- Host -> device command types ---
constexpr uint8_t CmdInit            = 0x81;
constexpr uint8_t CmdClearForces     = 0x82;
constexpr uint8_t CmdEnableForces    = 0x83;
constexpr uint8_t CmdDisableForces   = 0x84;
constexpr uint8_t CmdEnablePosition  = 0x91;
constexpr uint8_t CmdDisablePosition = 0x92;
constexpr uint8_t CmdJolt            = 0xA1;
constexpr uint8_t CmdButtonClear     = 0xA2;
constexpr uint8_t CmdButtonJolt      = 0xA3;
constexpr uint8_t CmdXVibration      = 0xA4;
constexpr uint8_t CmdYVibration      = 0xA5;
constexpr uint8_t CmdBuffeting       = 0xA6;
constexpr uint8_t CmdVectorForce     = 0xA7;
constexpr uint8_t CmdXSpring         = 0xA8;
constexpr uint8_t CmdYSpring         = 0xA9; // also copyright request when len 0

// Single-character query commands (sent with no data)
constexpr uint8_t QueryReset    = 'R';
constexpr uint8_t QueryEcho     = 'E';
constexpr uint8_t QueryVersion  = 'V';
constexpr uint8_t QueryDate     = 'D';
constexpr uint8_t QuerySerial   = 'N';
constexpr uint8_t QueryModel    = 'M';
constexpr uint8_t QueryAxes     = 'A';
constexpr uint8_t QueryPosition = 'P';

// --- Device -> host message types ---
constexpr uint8_t MsgStatus    = 'S';
constexpr uint8_t MsgPosition  = 'p';
constexpr uint8_t MsgCopyright = 0xA9;

// Effect bits within status byte 1
constexpr uint8_t EffectXVibration = 0x01;
constexpr uint8_t EffectYVibration = 0x02;
constexpr uint8_t EffectBuffeting  = 0x04;
constexpr uint8_t EffectVector     = 0x08;
constexpr uint8_t EffectXSpring    = 0x10;
constexpr uint8_t EffectYSpring    = 0x20;

// Status byte 0 bits
constexpr uint8_t StatusForcesSoftware = 0x01;
constexpr uint8_t StatusForcesHardware = 0x02; // DTR line
constexpr uint8_t StatusDeadmanActive  = 0x04; // 1 = inactive (forces allowed)
constexpr uint8_t StatusPowerConnected = 0x08;
constexpr uint8_t StatusNoCopyrightYet = 0x10; // 1 = copyright not yet requested
constexpr uint8_t StatusPositionActive = 0x20;

// Emulated device identity (modelled on the CH Products Force-FX, model 102,
// which supports both digital position reporting and spring effects).
constexpr uint16_t DeviceModelNumber  = 102;
constexpr uint32_t DeviceSerialNumber = 1;
constexpr uint8_t DeviceVersionMajor  = 1;
constexpr uint8_t DeviceVersionMinor  = 5;
constexpr uint8_t DeviceVersionSub    = 0;
constexpr uint8_t DeviceDateMonth     = 12;
constexpr uint8_t DeviceDateDay       = 10;
constexpr uint8_t DeviceDateYear      = 96;
constexpr uint8_t DeviceForceAxes     = 2;
constexpr uint8_t DevicePositionAxes  = 2;

// Production / DirectInput-era query responses
constexpr uint16_t DeviceVendorId    = 0x06a3; // Saitek/CH-style I-FORCE vendor
constexpr uint16_t DeviceProductId   = 0x0066; // model 102
constexpr uint16_t DeviceMemoryBytes = 1024;   // effect storage RAM
constexpr uint8_t DeviceNumEffects   = 20;     // simultaneous effects

uint8_t AxisToByte(const double normalised)
{
	// normalised is -1.0 .. +1.0; map to 0 .. 255 (0 = full negative)
	const auto scaled = (normalised + 1.0) * 127.5;
	return static_cast<uint8_t>(std::clamp(scaled, 0.0, 255.0));
}
} // namespace

CSerialIForce::CSerialIForce(const uint8_t id, CommandLine* cmd)
        : CSerial(id, cmd),
          port_num(id + 1)
{
	CSerial::Init_Registers();
	setRI(false);
	setDSR(true);  // device is present and powered
	setCD(true);
	setCTS(true);

	// Optional 'ffb:' parameter selects the force-feedback output mode:
	//   off      - no output
	//   rumble   - gamepad vibration motors (default); Jolt + Vibration only
	//   wheel    - true force feedback, steering (X) axis only
	//   joystick - true force feedback on both X and Y axes
	std::string ffb_value = {};
	if (cmd && cmd->FindStringBegin("ffb:", ffb_value, false)) {
		if (iequals(ffb_value, "off")) {
			ffb_mode = JoyFfb::Mode::Off;
		} else if (iequals(ffb_value, "wheel")) {
			ffb_mode = JoyFfb::Mode::Wheel;
		} else if (iequals(ffb_value, "joystick")) {
			ffb_mode = JoyFfb::Mode::Joystick;
		} else if (iequals(ffb_value, "rumble") ||
		           iequals(ffb_value, "gamepad")) {
			ffb_mode = JoyFfb::Mode::Rumble;
		} else {
			LOG_WARNING("SERIAL: I-FORCE COM%d invalid 'ffb' value '%s', "
			            "using 'rumble'",
			            port_num, ffb_value.c_str());
		}
	}

	// Optional 'ffbstrength:' parameter (0-100) scales the output.
	std::string strength_value = {};
	if (cmd && cmd->FindStringBegin("ffbstrength:", strength_value, false)) {
		const auto percent = atoi(strength_value.c_str());
		ffb_strength = std::clamp(percent, 0, 100) / 100.0f;
	}

	// Optional per-axis force inversion ('ffbinvertx', 'ffbinverty').
	auto parse_bool_flag = [&](const char* prefix, bool& flag) {
		std::string value = {};
		if (cmd && cmd->FindStringBegin(prefix, value, false)) {
			flag = iequals(value, "on") || iequals(value, "true") ||
			       iequals(value, "yes") || iequals(value, "1");
		}
	};
	parse_bool_flag("ffbinvertx:", ffb_invert_x);
	parse_bool_flag("ffbinverty:", ffb_invert_y);

	ScheduleStatus();

	const char* mode_name = (ffb_mode == JoyFfb::Mode::Off)      ? "off"
	                      : (ffb_mode == JoyFfb::Mode::Wheel)    ? "wheel"
	                      : (ffb_mode == JoyFfb::Mode::Joystick) ? "joystick"
	                                                             : "rumble";
	LOG_MSG("SERIAL: Port %d, Immersion I-FORCE joystick emulation enabled "
	        "(ffb: %s)",
	        port_num, mode_name);

	InstallationSuccessful = true;
}

CSerialIForce::~CSerialIForce()
{
	removeEvent(SERIAL_TX_EVENT);
	removeEvent(SERIAL_RX_EVENT);
	removeEvent(IForceStatusEvent);
	removeEvent(IForcePositionEvent);
	removeEvent(IForceButtonPollEvent);

	if (ffb_mode != JoyFfb::Mode::Off) {
		JoyFfb::StopAll();
	}
}

void CSerialIForce::ResetEffects()
{
	effect_flags        = 0;
	reflex_flags        = 0;
	forces_enabled      = false;
	position_reporting  = false;
	position_period_ms  = 0;
	removeEvent(IForcePositionEvent);
	for (auto& r : reflexes) {
		r = {};
	}
	button_poll_active = false;
	removeEvent(IForceButtonPollEvent);
	StopFfb();
}

uint8_t CSerialIForce::StatusByte0() const
{
	uint8_t byte0 = StatusDeadmanActive | StatusPowerConnected;
	if (forces_enabled) {
		byte0 |= StatusForcesSoftware;
	}
	if (dtr_asserted) {
		byte0 |= StatusForcesHardware;
	}
	if (!copyright_requested) {
		byte0 |= StatusNoCopyrightYet;
	}
	if (position_reporting) {
		byte0 |= StatusPositionActive;
	}
	return byte0;
}

// ---------------------------------------------------------------------------
// Real force-feedback output (rumble)
// ---------------------------------------------------------------------------

void CSerialIForce::EnsureFfbOpen()
{
	if (ffb_mode != JoyFfb::Mode::Off && !ffb_opened) {
		ffb_opened = JoyFfb::Open(ffb_mode, ffb_strength);
	}
}

void CSerialIForce::PlayJolt(const int mag_x, const int mag_y,
                             const uint32_t duration_ms)
{
	EnsureFfbOpen();
	const float fx = mag_x / 100.0f * (ffb_invert_x ? -1.0f : 1.0f);
	const float fy = mag_y / 100.0f * (ffb_invert_y ? -1.0f : 1.0f);

	if (ffb_mode == JoyFfb::Mode::Rumble) {
		JoyFfb::PlayRumble(std::min(std::hypot(fx, fy), 1.0f), duration_ms);
	} else if (ffb_mode == JoyFfb::Mode::Wheel) {
		// Steering-axis (X) only.
		JoyFfb::PlayConstantPulse(fx, 0.0f, duration_ms);
	} else if (ffb_mode == JoyFfb::Mode::Joystick) {
		// Both axes.
		JoyFfb::PlayConstantPulse(fx, fy, duration_ms);
	}
}

void CSerialIForce::ApplyVibration()
{
	EnsureFfbOpen();
	if (ffb_mode == JoyFfb::Mode::Rumble) {
		// Gamepad: Jolt + Vibration only (buffeting excluded by request).
		JoyFfb::SetSustainedRumble(std::max(ffb_x_vibration, ffb_y_vibration));
	} else if (ffb_mode == JoyFfb::Mode::Wheel ||
	           ffb_mode == JoyFfb::Mode::Joystick) {
		// Vibration and buffeting (turbulence) share one periodic effect.
		// Wheel uses the X vibration only; joystick also folds in Y.
		float level = std::max(ffb_x_vibration, ffb_buffeting);
		if (ffb_mode == JoyFfb::Mode::Joystick) {
			level = std::max(level, ffb_y_vibration);
		}
		// Vibration carries its own frequency; buffeting alone uses a
		// moderate default turbulence frequency.
		const float freq_hz = (ffb_x_vibration > 0.0f || ffb_y_vibration > 0.0f)
		                            ? ffb_vib_freq_hz
		                            : 15.0f;
		JoyFfb::SetPeriodic(level, freq_hz);
	}
}

void CSerialIForce::ApplySpring()
{
	EnsureFfbOpen();
	// Springs only apply in the true force-feedback modes. Wheel is
	// steering-axis (X) only, so its Y spring is dropped; joystick uses both.
	const bool use_y = (ffb_mode == JoyFfb::Mode::Joystick);
	JoyFfb::SetSpring(ffb_spring_center_x, ffb_spring_left_x, ffb_spring_right_x,
	                  use_y ? ffb_spring_center_y : 0.0f,
	                  use_y ? ffb_spring_left_y : 0.0f,
	                  use_y ? ffb_spring_right_y : 0.0f);
}

void CSerialIForce::StopFfb()
{
	ffb_x_vibration    = 0.0f;
	ffb_y_vibration    = 0.0f;
	ffb_buffeting      = 0.0f;
	ffb_spring_left_x  = 0.0f;
	ffb_spring_right_x = 0.0f;
	ffb_spring_left_y  = 0.0f;
	ffb_spring_right_y = 0.0f;
	if (ffb_mode != JoyFfb::Mode::Off) {
		JoyFfb::StopAll();
	}
}

// ---------------------------------------------------------------------------
// Button-reflex jolts: the device autonomously fires a jolt when a configured
// joystick button is pressed, repeating at the programmed rate while held.
// ---------------------------------------------------------------------------

bool CSerialIForce::ReadReflexButton(const int index) const
{
	// Map reflex buttons 0-3 to the game-port joystick buttons, matching the
	// position-report button order.
	switch (index) {
	case 0: return JOYSTICK_IsAccessible(0) && JOYSTICK_GetButton(0, 0);
	case 1: return JOYSTICK_IsAccessible(0) && JOYSTICK_GetButton(0, 1);
	case 2: return JOYSTICK_IsAccessible(1) && JOYSTICK_GetButton(1, 0);
	case 3: return JOYSTICK_IsAccessible(1) && JOYSTICK_GetButton(1, 1);
	default: return false;
	}
}

void CSerialIForce::StartButtonPoll()
{
	if (!button_poll_active) {
		button_poll_active = true;
		setEvent(IForceButtonPollEvent, ButtonPollIntervalMs);
	}
}

void CSerialIForce::PollButtonReflexes()
{
	bool any_active = false;

	for (int i = 0; i < num_reflex_buttons; ++i) {
		ButtonReflex& r = reflexes[i];
		if (!r.active) {
			r.prev_pressed = false;
			continue;
		}
		any_active = true;

		const bool pressed = ReadReflexButton(i);

		if (pressed != r.prev_pressed) {
			Log("ButtonReflex: button %d %s", i,
			    pressed ? "pressed" : "released");
		}

		if (pressed) {
			if (!r.prev_pressed) {
				// Rising edge: fire immediately.
				PlayJolt(r.mag_x, r.mag_y, r.duration_ms);
				r.countdown_ms = static_cast<float>(r.repeat_ms);
				Log("ButtonReflex: button %d fired (magX=%d magY=%d "
				    "duration=%d ms)",
				    i, r.mag_x, r.mag_y, r.duration_ms);
			} else {
				r.countdown_ms -= ButtonPollIntervalMs;
				if (r.countdown_ms <= 0.0f) {
					PlayJolt(r.mag_x, r.mag_y, r.duration_ms);
					r.countdown_ms += static_cast<float>(
					        std::max(r.repeat_ms,
					                 static_cast<uint32_t>(ButtonPollIntervalMs)));
				}
			}
		}
		r.prev_pressed = pressed;
	}

	button_poll_active = any_active;
	if (button_poll_active) {
		setEvent(IForceButtonPollEvent, ButtonPollIntervalMs);
	}
}

// ---------------------------------------------------------------------------
// Incoming frame parsing
// ---------------------------------------------------------------------------

void CSerialIForce::transmitByte(const uint8_t val, const bool first)
{
#if IFORCE_LOG_RAW
	Log("RX byte 0x%02x ('%c')", val,
	    (val >= 0x20 && val < 0x7f) ? static_cast<char>(val) : '.');
#endif

	ParseByte(val);

	// Signal the guest UART that the byte has been shifted out
	if (first) {
		SetEventTHR();
	} else {
		SetEventTX();
	}
}

void CSerialIForce::ParseByte(const uint8_t ch)
{
	in_csum ^= ch;

	switch (recv_state) {
	case RecvState::FindHeader:
		if (ch == StartChar) {
			recv_state  = RecvState::GetType;
			in_csum     = 0; // header is NOT part of either checksum
			in_data_len = 0;
		}
		break;

	case RecvState::GetType:
		in_type    = ch;
		recv_state = RecvState::GetLen;
		break;

	case RecvState::GetLen:
		in_len_to_go = ch + 1; // +1 accounts for the trailing checksum byte
		recv_state   = RecvState::GetData;
		break;

	case RecvState::GetData:
		if (in_len_to_go > 1) {
			// payload byte
			if (in_data_len < max_data_bytes) {
				in_data[in_data_len++] = ch;
			}
		}
		// else: this byte is the checksum
		if (--in_len_to_go <= 0) {
			recv_state = RecvState::FindHeader;
			// The checksum value identifies the dialect: the production
			// protocol XORs every byte after the header (so a valid frame
			// nets 0), while the 1996 SDK 1.05 protocol also folds the
			// '+' header into its checksum (so the same XOR nets 0x2B).
			if (in_csum == 0x00) {
				DispatchProduction(in_type, in_data, in_data_len);
			} else if (in_csum == StartChar) {
				DispatchLegacy(in_type, in_data, in_data_len);
			}
#if IFORCE_LOG_RAW
			else {
				Log("Frame checksum mismatch: type=0x%02x len=%d csum=0x%02x",
				    in_type, in_data_len, in_csum);
			}
#endif
		}
		break;
	}
}

void CSerialIForce::Log(const char* format, ...) const
{
	if (!LogIForce) {
		return;
	}
	char message[256] = {};
	va_list args;
	va_start(args, format);
	vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	LOG_MSG("SERIAL: I-FORCE COM%d %s", port_num, message);
}

// Force-effect payload bytes carry signed magnitudes
static int Signed(const uint8_t value)
{
	return static_cast<int>(static_cast<int8_t>(value));
}

void CSerialIForce::DispatchLegacy(const uint8_t type, const uint8_t* data,
                                   const int len)
{
	switch (type) {
	case QueryReset:
		ResetEffects();
		QueueMessage(QueryReset, nullptr, 0);
		Log("InitStick: reset");
		break;

	case QueryEcho:
		QueueMessage(QueryEcho, nullptr, 0);
		Log("Echo");
		break;

	case QueryVersion: {
		const uint8_t v[] = {DeviceVersionMajor,
		                     DeviceVersionMinor,
		                     DeviceVersionSub};
		QueueMessage(QueryVersion, v, sizeof(v));
		Log("Query firmware version -> %d.%d.%d",
		    DeviceVersionMajor, DeviceVersionMinor, DeviceVersionSub);
		break;
	}
	case QueryDate: {
		const uint8_t d[] = {DeviceDateMonth, DeviceDateDay, DeviceDateYear};
		QueueMessage(QueryDate, d, sizeof(d));
		Log("Query firmware date -> %02d/%02d/%02d",
		    DeviceDateMonth, DeviceDateDay, DeviceDateYear);
		break;
	}
	case QuerySerial: {
		const uint8_t n[] = {
		        static_cast<uint8_t>(DeviceSerialNumber & 0xff),
		        static_cast<uint8_t>((DeviceSerialNumber >> 8) & 0xff),
		        static_cast<uint8_t>((DeviceSerialNumber >> 16) & 0xff),
		        static_cast<uint8_t>((DeviceSerialNumber >> 24) & 0xff)};
		QueueMessage(QuerySerial, n, sizeof(n));
		Log("Query serial number -> %u", DeviceSerialNumber);
		break;
	}
	case QueryModel: {
		const uint8_t m[] = {
		        static_cast<uint8_t>(DeviceModelNumber & 0xff),
		        static_cast<uint8_t>((DeviceModelNumber >> 8) & 0xff)};
		QueueMessage(QueryModel, m, sizeof(m));
		Log("Query model number -> %u", DeviceModelNumber);
		break;
	}
	case QueryAxes: {
		// A force-feedback wheel is single-axis (steering); reporting 2 axes
		// makes the game also send Y-axis forces that rotate the wheel.
		const uint8_t a = (ffb_mode == JoyFfb::Mode::Wheel)
		                        ? uint8_t{1}
		                        : DeviceForceAxes;
		QueueMessage(QueryAxes, &a, 1);
		Log("Query force axes -> %d", a);
		break;
	}
	case QueryPosition: {
		const uint8_t p = DevicePositionAxes;
		QueueMessage(QueryPosition, &p, 1);
		Log("Query position axes -> %d", DevicePositionAxes);
		break;
	}

	case CmdInit:
		ResetEffects();
		Log("InitStick: initialise");
		break;

	case CmdEnableForces:
		forces_enabled = true;
		EnsureFfbOpen();
		Log("EnableForces");
		break;
	case CmdDisableForces:
		forces_enabled = false;
		StopFfb();
		Log("DisableForces");
		break;

	case CmdClearForces:
		effect_flags = 0;
		reflex_flags = 0;
		for (auto& r : reflexes) {
			r = {};
		}
		StopFfb();
		Log("ClearForces");
		break;

	case CmdEnablePosition:
		if (len >= 1) {
			position_period_ms = std::max(data[0], MinPositionPeriodMs);
			position_reporting = true;
			SchedulePosition();
			Log("EnablePositionReporting: every %d ms",
			    position_period_ms);
		}
		break;

	case CmdDisablePosition:
		position_reporting = false;
		removeEvent(IForcePositionEvent);
		Log("DisablePositionReporting");
		break;

	case CmdJolt:
		if (len >= 3) {
			const auto duration_ms = static_cast<uint32_t>(data[2]) * 10;
			PlayJolt(Signed(data[0]), Signed(data[1]), duration_ms);
			Log("Jolt: magX=%d magY=%d duration=%d ms",
			    Signed(data[0]), Signed(data[1]), data[2] * 10);
		}
		break; // transient effect, no persistent status

	case CmdButtonJolt:
		if (len >= 5 && data[0] < num_reflex_buttons) {
			reflex_flags |= static_cast<uint8_t>(1 << data[0]);
			// duration and repeat-delay are stored in units of 10 ms; the
			// repeat period is duration + repeat-delay.
			const auto duration_ms = static_cast<uint32_t>(data[3]) * 10;
			const auto repeat_delay = static_cast<uint32_t>(data[4]) * 10;
			ButtonReflex& r = reflexes[data[0]];
			r.active       = true;
			r.mag_x        = Signed(data[1]);
			r.mag_y        = Signed(data[2]);
			r.duration_ms  = duration_ms;
			r.repeat_ms    = duration_ms + repeat_delay;
			r.prev_pressed = false;
			r.countdown_ms = 0.0f;
			StartButtonPoll();
			Log("ButtonReflexJolt: button=%d magX=%d magY=%d "
			    "duration=%d ms repeat=%d ms",
			    data[0], r.mag_x, r.mag_y, r.duration_ms, r.repeat_ms);
		}
		break;
	case CmdButtonClear:
		if (len >= 1 && data[0] < num_reflex_buttons) {
			reflex_flags &= static_cast<uint8_t>(~(1 << data[0]));
			reflexes[data[0]] = {};
			Log("ButtonReflexClear: button=%d", data[0]);
		}
		break;

	case CmdXVibration:
		if (len >= 3 && (data[0] || data[1])) {
			effect_flags |= EffectXVibration;
			ffb_x_vibration = std::max(data[0], data[1]) / 100.0f;
			ffb_vib_freq_hz = static_cast<float>(data[2]);
			Log("XVibration: left=%d%% right=%d%% freq=%d Hz",
			    data[0], data[1], data[2]);
		} else {
			effect_flags &= static_cast<uint8_t>(~EffectXVibration);
			ffb_x_vibration = 0.0f;
			Log("XVibration: cleared");
		}
		ApplyVibration();
		break;
	case CmdYVibration:
		if (len >= 3 && (data[0] || data[1])) {
			effect_flags |= EffectYVibration;
			ffb_y_vibration = std::max(data[0], data[1]) / 100.0f;
			ffb_vib_freq_hz = static_cast<float>(data[2]);
			Log("YVibration: down=%d%% up=%d%% freq=%d Hz",
			    data[0], data[1], data[2]);
		} else {
			effect_flags &= static_cast<uint8_t>(~EffectYVibration);
			ffb_y_vibration = 0.0f;
			Log("YVibration: cleared");
		}
		ApplyVibration();
		break;
	case CmdBuffeting:
		if (len >= 1 && data[0]) {
			effect_flags |= EffectBuffeting;
			ffb_buffeting = data[0] / 100.0f;
			Log("Buffeting: magnitude=%d%%", data[0]);
		} else {
			effect_flags &= static_cast<uint8_t>(~EffectBuffeting);
			ffb_buffeting = 0.0f;
			Log("Buffeting: cleared");
		}
		ApplyVibration();
		break;
	case CmdVectorForce:
		// Constant force is the core driving effect (g-forces, crosswind,
		// pushing into walls). Wheel uses X only; joystick uses X and Y.
		if (len >= 2 && (data[0] || data[1])) {
			effect_flags |= EffectVector;
			if (ffb_mode == JoyFfb::Mode::Wheel ||
			    ffb_mode == JoyFfb::Mode::Joystick) {
				EnsureFfbOpen();
				const float fx = Signed(data[0]) / 100.0f *
				                 (ffb_invert_x ? -1.0f : 1.0f);
				const float fy = (ffb_mode == JoyFfb::Mode::Joystick)
				                       ? Signed(data[1]) / 100.0f *
				                                 (ffb_invert_y ? -1.0f : 1.0f)
				                       : 0.0f;
				JoyFfb::SetConstantForce(fx, fy);
			}
			Log("VectorForce: magX=%d magY=%d",
			    Signed(data[0]), Signed(data[1]));
		} else {
			effect_flags &= static_cast<uint8_t>(~EffectVector);
			if (ffb_mode == JoyFfb::Mode::Wheel ||
			    ffb_mode == JoyFfb::Mode::Joystick) {
				JoyFfb::SetConstantForce(0.0f, 0.0f);
			}
			Log("VectorForce: cleared");
		}
		break;
	case CmdXSpring:
		if (len >= 3 && (data[1] || data[2])) {
			effect_flags |= EffectXSpring;
			ffb_spring_center_x = Signed(data[0]) / 100.0f;
			ffb_spring_left_x   = data[1] / 100.0f;
			ffb_spring_right_x  = data[2] / 100.0f;
			Log("XSpring: origin=%d left-stiffness=%d right-stiffness=%d",
			    Signed(data[0]), data[1], data[2]);
		} else {
			effect_flags &= static_cast<uint8_t>(~EffectXSpring);
			ffb_spring_left_x  = 0.0f;
			ffb_spring_right_x = 0.0f;
			Log("XSpring: cleared");
		}
		ApplySpring();
		break;

	// 0xA9 is overloaded: a zero-length payload is a copyright request,
	// otherwise it is the YSpring effect command.
	case CmdYSpring:
		if (len == 0) {
			copyright_requested = true;
			const uint8_t msg[] = {'I', 'H', 'I', 'C', '9', '6'};
			QueueMessage(MsgCopyright, msg, sizeof(msg));
			Log("Authenticate: sent copyright \"IHIC96\"");
		} else if (len >= 3 && (data[1] || data[2])) {
			// Down-stiffness -> negative (left) side, up -> positive (right).
			// Applied only in joystick mode (wheel drops Y).
			effect_flags |= EffectYSpring;
			ffb_spring_center_y = Signed(data[0]) / 100.0f;
			ffb_spring_left_y   = data[1] / 100.0f;
			ffb_spring_right_y  = data[2] / 100.0f;
			Log("YSpring: origin=%d down-stiffness=%d up-stiffness=%d",
			    Signed(data[0]), data[1], data[2]);
			ApplySpring();
		} else if (len >= 3) {
			effect_flags &= static_cast<uint8_t>(~EffectYSpring);
			ffb_spring_left_y  = 0.0f;
			ffb_spring_right_y = 0.0f;
			Log("YSpring: cleared");
			ApplySpring();
		}
		break;

	default:
		Log("Unknown legacy command 0x%02x (%d bytes)", type, len);
		break;
	}
}

// ---------------------------------------------------------------------------
// Production / DirectInput-era protocol (opcodes FF, 01, 02, 40-43)
//
// Detection hinges on answering the 'O' (open) query: the host sends it up to
// 20 times and gives up if it never gets a reply. All query replies use opcode
// 0xFF and carry the queried letter as the first payload byte, mirroring what
// the host matches against. The host does not verify the reply checksum.
// ---------------------------------------------------------------------------

void CSerialIForce::DispatchProduction(const uint8_t type, const uint8_t* data,
                                       const int len)
{
	// Opcode 0xFF: device query
	if (type == 0xFF) {
		if (len < 1) {
			return;
		}
		const uint8_t query = data[0];
		switch (query) {
		case 'O': // open device
			production_opened = true;
			QueueProductionQuery(query, nullptr, 0);
			Log("Query OPEN -> device ready");
			break;
		case 'C': // close device
			production_opened = false;
			QueueProductionQuery(query, nullptr, 0);
			Log("Query CLOSE");
			break;
		case 'M': { // vendor id (little-endian uint16)
			const uint8_t v[] = {
			        static_cast<uint8_t>(DeviceVendorId & 0xff),
			        static_cast<uint8_t>((DeviceVendorId >> 8) & 0xff)};
			QueueProductionQuery(query, v, sizeof(v));
			Log("Query vendor id -> 0x%04x", DeviceVendorId);
			break;
		}
		case 'P': { // product id (little-endian uint16)
			const uint8_t v[] = {
			        static_cast<uint8_t>(DeviceProductId & 0xff),
			        static_cast<uint8_t>((DeviceProductId >> 8) & 0xff)};
			QueueProductionQuery(query, v, sizeof(v));
			Log("Query product id -> 0x%04x", DeviceProductId);
			break;
		}
		case 'B': { // effect-memory size (little-endian uint16)
			const uint8_t v[] = {
			        static_cast<uint8_t>(DeviceMemoryBytes & 0xff),
			        static_cast<uint8_t>((DeviceMemoryBytes >> 8) & 0xff)};
			QueueProductionQuery(query, v, sizeof(v));
			Log("Query memory size -> %d bytes", DeviceMemoryBytes);
			break;
		}
		case 'N': { // number of simultaneous effects
			const uint8_t v[] = {DeviceNumEffects};
			QueueProductionQuery(query, v, sizeof(v));
			Log("Query effect count -> %d", DeviceNumEffects);
			break;
		}
		case 'V': { // firmware version (major, minor, subminor)
			const uint8_t v[] = {DeviceVersionMajor,
			                     DeviceVersionMinor,
			                     DeviceVersionSub};
			QueueProductionQuery(query, v, sizeof(v));
			Log("Query firmware version -> %d.%d.%d",
			    DeviceVersionMajor, DeviceVersionMinor, DeviceVersionSub);
			break;
		}
		case 'E': { // is this effect type supported? (nonzero = yes)
			const uint8_t v[] = {0x01, 0x00};
			QueueProductionQuery(query, v, sizeof(v));
			Log("Query effect support -> supported");
			break;
		}
		default:
			// Reply with just the queried letter so the host doesn't stall
			// waiting on an answer it can still match.
			QueueProductionQuery(query, nullptr, 0);
			Log("Query '%c' (0x%02x) -> acknowledged",
			    (query >= 0x20 && query < 0x7f) ? static_cast<char>(query) : '.',
			    query);
			break;
		}
		return;
	}

	// Effect / control opcodes: accepted and logged (no haptic output).
	switch (type) {
	case 0x40: Log("Set control parameters (%d bytes)", len); break;
	case 0x41: Log("Effect control: start/stop (%d bytes)", len); break;
	case 0x42: Log("Set effect state (%d bytes)", len); break;
	case 0x43: Log("Set overall gain (%d bytes)", len); break;
	case 0x01: Log("Upload force effect (%d bytes)", len); break;
	case 0x02: Log("Set device effect state (%d bytes)", len); break;
	default:
		Log("Unknown production command 0x%02x (%d bytes)", type, len);
		break;
	}
}

// ---------------------------------------------------------------------------
// Outgoing messages
// ---------------------------------------------------------------------------

void CSerialIForce::QueueMessage(const uint8_t type, const uint8_t* data,
                                 const int len)
{
	uint8_t csum = 0;

	auto push = [&](const uint8_t b) {
		csum ^= b;
		tx_queue.push_back(b);
	};

	push(StartChar);
	push(type);
	push(static_cast<uint8_t>(len));
	for (int i = 0; i < len; ++i) {
		push(data[i]);
	}
	tx_queue.push_back(csum);

	SetEventRX();
}

void CSerialIForce::QueueProductionQuery(const uint8_t query,
                                         const uint8_t* value, const int value_len)
{
	// Reply payload is the queried letter followed by its value bytes; the
	// whole thing is sent under opcode 0xFF.
	uint8_t payload[1 + max_data_bytes] = {query};
	const int payload_len = 1 + value_len;
	for (int i = 0; i < value_len && i < max_data_bytes; ++i) {
		payload[1 + i] = value[i];
	}
	QueueMessage(0xFF, payload, payload_len);
}

void CSerialIForce::SendStatus()
{
	const uint8_t status[] = {StatusByte0(), effect_flags, reflex_flags, 0};
	QueueMessage(MsgStatus, status, sizeof(status));
}

void CSerialIForce::SendPosition()
{
	uint8_t x        = 128;
	uint8_t y        = 128;
	uint16_t buttons = 0;

	if (JOYSTICK_IsAccessible(0)) {
		x = AxisToByte(JOYSTICK_GetMove_X(0));
		y = AxisToByte(JOYSTICK_GetMove_Y(0));
		if (JOYSTICK_GetButton(0, 0)) {
			buttons |= 0x01;
		}
		if (JOYSTICK_GetButton(0, 1)) {
			buttons |= 0x02;
		}
	}
	if (JOYSTICK_IsAccessible(1)) {
		if (JOYSTICK_GetButton(1, 0)) {
			buttons |= 0x04;
		}
		if (JOYSTICK_GetButton(1, 1)) {
			buttons |= 0x08;
		}
	}

	const uint8_t pos[] = {x,
	                       y,
	                       static_cast<uint8_t>(buttons & 0xff),
	                       static_cast<uint8_t>((buttons >> 8) & 0xff)};
	QueueMessage(MsgPosition, pos, sizeof(pos));
}

// ---------------------------------------------------------------------------
// Event scheduling
// ---------------------------------------------------------------------------

void CSerialIForce::SetEventTX()
{
	setEvent(SERIAL_TX_EVENT, bytetime);
}

void CSerialIForce::SetEventRX()
{
	setEvent(SERIAL_RX_EVENT, bytetime);
}

void CSerialIForce::SetEventTHR()
{
	setEvent(SERIAL_THR_EVENT, bytetime / 10);
}

void CSerialIForce::ScheduleStatus()
{
	setEvent(IForceStatusEvent, StatusPeriodMs);
}

void CSerialIForce::SchedulePosition()
{
	if (position_reporting) {
		setEvent(IForcePositionEvent,
		         static_cast<float>(position_period_ms));
	}
}

void CSerialIForce::handleUpperEvent(const uint16_t event_type)
{
	switch (event_type) {
	case SERIAL_TX_EVENT: ByteTransmitted(); break;

	case SERIAL_THR_EVENT:
		ByteTransmitting();
		SetEventTX();
		break;

	case SERIAL_RX_EVENT:
		if (!tx_queue.empty()) {
			if (CSerial::CanReceiveByte()) {
				CSerial::receiveByte(tx_queue.front());
				tx_queue.pop_front();
			}
			if (!tx_queue.empty()) {
				SetEventRX();
			}
		}
		break;

	case IForceStatusEvent:
		SendStatus();
		ScheduleStatus();
		break;

	case IForcePositionEvent:
		if (position_reporting) {
			SendPosition();
			SchedulePosition();
		}
		break;

	case IForceButtonPollEvent:
		PollButtonReflexes();
		break;
	}
}

// ---------------------------------------------------------------------------
// Control lines / port configuration
// ---------------------------------------------------------------------------

void CSerialIForce::setRTSDTR(const bool rts, const bool dtr)
{
	setRTS(rts);
	setDTR(dtr);
}

void CSerialIForce::setRTS([[maybe_unused]] const bool val)
{
	// A real I-FORCE device is always powered and ready, so it keeps DSR,
	// CTS and CD asserted regardless of the host's RTS/DTR lines (these were
	// raised in the constructor). We deliberately do NOT mirror RTS onto CTS
	// here: detection routines that poll the modem status lines for device
	// presence must always see the device as present.
}

void CSerialIForce::setDTR(const bool val)
{
	// The I-FORCE device treats an asserted DTR line as the hardware
	// force-enable signal, but its presence lines stay asserted either way.
	dtr_asserted = val;
}

void CSerialIForce::updatePortConfig([[maybe_unused]] const uint16_t divider,
                                     [[maybe_unused]] const uint8_t lcr)
{
	// The device runs at a fixed 9600 baud, 8N1; nothing to reconfigure.
}

void CSerialIForce::updateMSR() {}

void CSerialIForce::setBreak([[maybe_unused]] const bool value) {}
