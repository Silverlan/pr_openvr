// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

export module pragma.modules.openvr:lua_bindings;

export import pragma.lua;

export namespace Lua {
	namespace openvr {
		void register_lua_library(Lua::Interface &l);
		void close();
	};
};
