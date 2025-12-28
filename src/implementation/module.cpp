// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module pragma.modules.openvr;

import :controller_state;
import pragma.string;

extern "C" {
void PR_EXPORT pragma_initialize_lua(Lua::Interface &l)
{
	if(pragma::string::compare<std::string>(l.GetIdentifier(), "cl") == false)
		return;
	Lua::openvr::register_lua_library(l);
}
void PR_EXPORT pragma_detach() { Lua::openvr::close(); }
void PR_EXPORT preinitialize_openvr() { ::openvr::preinitialize_openvr(); }
bool PR_EXPORT is_hmd_present() { return ::openvr::is_hmd_present(); }
/*
	void PR_EXPORT InitializeLua(Lua::Interface &l)
	{
		if(l.GetIdentifier() != "cl")
			return;
		if(IState::is_game_active() == true)
			Lua::openvr::register_lua_library(l.GetState());
		cbGameStarted = IState::add_callback(IState::Callback::OnGameStart,FunctionCallback<void,CGame*>::Create([](pragma::CGame *game) {
			if(cbGameInitialized.IsValid())
				cbGameInitialized.Remove();
			cbGameInitialized = IState::add_callback(IState::Callback::OnGameInitialized,FunctionCallback<void,Game*>::Create([](Game *game) {
				if(s_vrInstance != nullptr)
					s_vrInstance->InitializeScene();
			}));

			if(cbLuaInitialized.IsValid())
				cbLuaInitialized.Remove();
			cbLuaInitialized = IState::add_callback(IState::Callback::OnLuaInitialized,FunctionCallback<void,lua_State*>::Create([](lua_State *l) {
				Lua::openvr::register_lua_library(l);
			}));
		}));
		cbGameEnd = IState::add_callback(IState::Callback::EndGame,FunctionCallback<void,CGame*>::Create([](pragma::CGame *game) {
			if(s_vrInstance != nullptr)
				s_vrInstance->ClearScene();
			clear_callbacks();
		}));
	}*/
};
