// SPDX-FileCopyrightText:  2002-2026 The DOSBox Staging Team
// SPDX-License-Identifier: GPL-2.0-or-later

// Immersion I-FORCE force-feedback joystick emulation.
//
// I-FORCE joysticks (e.g. the CH Products Force-FX) used a dual interface: the
// standard game port for legacy position/buttons, plus a 9600-baud serial link
// over which the host sends high-level force-effect commands and queries the
// device. This class emulates the serial side of such a device, so that games
// and tools written against the Immersion I-FORCE API detect a connected
// joystick and operate normally.
//
// The wire protocol (host <-> device) frames every message as:
//   '+' <type> <len> <data[0..len-1]> <checksum>
// where checksum is the XOR of all preceding bytes of the frame.
//
// This driver also contains some support for I-FORCE 2.00 API.

#ifndef DOSBOX_SERIALIFORCE_H
#define DOSBOX_SERIALIFORCE_H

#include "serialport.h"

#include <deque>

#include "hardware/input/joystick_ffb.h"

class CSerialIForce final : public CSerial {
public:
	CSerialIForce(const uint8_t id, CommandLine* cmd);
	~CSerialIForce() override;

	void setRTSDTR(const bool rts, const bool dtr) override;
	void setRTS(const bool val) override;
	void setDTR(const bool val) override;

	void updatePortConfig(const uint16_t divider, const uint8_t lcr) override;
	void updateMSR() override;
	void transmitByte(const uint8_t val, const bool first) override;
	void setBreak(const bool value) override;
	void handleUpperEvent(const uint16_t event_type) override;

private:
	// Incoming frame parser (mirrors the device-side serial state machine)
	void ParseByte(const uint8_t ch);
	// SDK 1.05 dialect ('R'/'V'/0x81/0xA1, checksum includes the header)
	void DispatchLegacy(const uint8_t type, const uint8_t* data, const int len);
	// Production/DirectInput dialect (FF queries, 01/02/40-43 effects)
	void DispatchProduction(const uint8_t type, const uint8_t* data, const int len);

	// Console logging of decoded commands (no-op when logging is disabled)
	void Log(const char* format, ...) const
	        GCC_ATTRIBUTE(__format__(__printf__, 2, 3));

	// Outgoing messages to the host
	void QueueMessage(const uint8_t type, const uint8_t* data, const int len);
	void QueueProductionQuery(const uint8_t query, const uint8_t* value,
	                          const int value_len);
	void QueueByte(const uint8_t value);
	void SendStatus();
	void SendPosition();

	// Device state helpers
	void ResetEffects();
	uint8_t StatusByte0() const;

	// Real force-feedback output helpers
	void EnsureFfbOpen();
	void PlayJolt(const int mag_x, const int mag_y, const uint32_t duration_ms);
	void ApplyVibration();
	void ApplySpring();
	void StopFfb();

	// Button-reflex helpers (autonomous button-triggered jolts)
	bool ReadReflexButton(const int index) const;
	void StartButtonPoll();
	void PollButtonReflexes();

	// Scheduling helpers
	void SetEventTX();
	void SetEventRX();
	void SetEventTHR();
	void ScheduleStatus();
	void SchedulePosition();

	const int port_num = 0; // for logging purposes

	// --- Incoming parser state ---
	enum class RecvState { FindHeader, GetType, GetLen, GetData };
	RecvState recv_state = RecvState::FindHeader;
	uint8_t in_type      = 0;
	uint8_t in_csum      = 0;
	int in_len_to_go     = 0;
	int in_data_len      = 0;
	static constexpr int max_data_bytes = 16;
	uint8_t in_data[max_data_bytes] = {};

	// --- Outgoing byte queue (delivered to host one byte per bytetime) ---
	std::deque<uint8_t> tx_queue = {};

	// --- Emulated device state ---
	bool forces_enabled      = false; // set by EnableForces / DisableForces
	bool dtr_asserted        = false; // hardware force enable line
	bool position_reporting  = false;
	bool copyright_requested = false;
	bool production_opened   = false; // production protocol 'O'/'C' state
	uint8_t position_period_ms = 0;

	uint8_t effect_flags = 0; // status byte 1: active continuous effects
	uint8_t reflex_flags = 0; // status byte 2: programmed button reflexes

	// --- Real force-feedback output ---
	JoyFfb::Mode ffb_mode = JoyFfb::Mode::Rumble;
	float ffb_strength    = 1.0f;  // overall scale 0.0..1.0
	bool ffb_opened       = false; // device opened lazily on first enable
	bool ffb_invert_x     = false; // flip steering-axis force direction
	bool ffb_invert_y     = false; // flip Y-axis force direction

	// Continuous effect state (0.0..1.0 magnitudes)
	float ffb_x_vibration   = 0.0f;
	float ffb_y_vibration   = 0.0f;
	float ffb_buffeting     = 0.0f;
	float ffb_vib_freq_hz   = 0.0f;
	// Per-direction spring stiffness. X = steering axis (wheel + joystick);
	// Y = second axis (joystick mode only).
	float ffb_spring_center_x = 0.0f;
	float ffb_spring_left_x   = 0.0f;
	float ffb_spring_right_x  = 0.0f;
	float ffb_spring_center_y = 0.0f;
	float ffb_spring_left_y   = 0.0f;
	float ffb_spring_right_y  = 0.0f;

	// --- Button-reflex jolts (I-FORCE ButtonReflexJolt) ---
	struct ButtonReflex {
		bool active           = false;
		int mag_x             = 0; // signed -100..100
		int mag_y             = 0;
		uint32_t duration_ms  = 0;
		uint32_t repeat_ms    = 0; // time between successive jolts when held
		bool prev_pressed     = false;
		float countdown_ms    = 0.0f; // until the next repeat while held
	};
	static constexpr int num_reflex_buttons = 4;
	ButtonReflex reflexes[num_reflex_buttons] = {};
	bool button_poll_active = false;
};

#endif // DOSBOX_SERIALIFORCE_H
