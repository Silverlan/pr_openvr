// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include <openvr.h>

export module pragma.modules.openvr:controller_state;

import pragma.platform;

export namespace openvr {
	struct ControllerState {
		ControllerState() = default;
		void UpdateState(const vr::VRControllerState_t &state);
		void OnStateChanged(uint32_t key, pragma::platform::KeyState state);
		void SetStateChangeCallback(const std::function<void(uint32_t, pragma::platform::KeyState)> &f);
	  private:
		vr::VRControllerState_t m_vrState = {};
		std::function<void(uint32_t, pragma::platform::KeyState)> m_stateChangeCallback = nullptr;
	};
};
