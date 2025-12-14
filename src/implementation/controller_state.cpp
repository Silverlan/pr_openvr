// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include <openvr.h>

module pragma.modules.openvr;

import :controller_state;

void openvr::ControllerState::SetStateChangeCallback(const std::function<void(uint32_t, pragma::platform::KeyState)> &f) { m_stateChangeCallback = f; }
void openvr::ControllerState::OnStateChanged(uint32_t key, pragma::platform::KeyState state)
{
	if(m_stateChangeCallback == nullptr)
		return;
	m_stateChangeCallback(key, state);
}
void openvr::ControllerState::UpdateState(const vr::VRControllerState_t &state)
{
	if(state.ulButtonPressed != m_vrState.ulButtonPressed) {
		auto v = pragma::math::max(pragma::math::get_highest_bit(state.ulButtonPressed), pragma::math::get_highest_bit(m_vrState.ulButtonPressed));
		while(v > 0) {
			if(state.ulButtonPressed & v) {
				if(!(m_vrState.ulButtonPressed & v))
					OnStateChanged(pragma::math::get_number_of_times_dividable_by_x(v, 2), pragma::platform::KeyState::Press);
			}
			else if(m_vrState.ulButtonPressed & v)
				OnStateChanged(pragma::math::get_number_of_times_dividable_by_x(v, 2), pragma::platform::KeyState::Release);
			v >>= 1u;
		}
	}
	if(state.ulButtonTouched != m_vrState.ulButtonTouched) {
		auto v = pragma::math::max(pragma::math::get_highest_bit(state.ulButtonTouched), pragma::math::get_highest_bit(m_vrState.ulButtonTouched));
		while(v > 0) {
			if(state.ulButtonTouched & v) {
				if(!(m_vrState.ulButtonTouched & v))
					; // std::cout << "Button " << v << " has been touched!" << std::endl; // Touched
			}
			else if(m_vrState.ulButtonTouched & v)
				; // std::cout << "Button " << v << " has been detouched!" << std::endl; // Detouched
			v >>= 1u;
		}
	}
	auto axisId = 0u;
	for(auto &axis : state.rAxis) {
		auto &axisOther = m_vrState.rAxis[axisId++];
		if(axis.x == axisOther.x && axis.y == axisOther.y)
			continue;
	}
	m_vrState = state;
}
