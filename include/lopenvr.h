// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

#ifndef __LOPENVR_H__
#define __LOPENVR_H__

struct lua_State;
namespace Lua {
	class Interface;
	namespace openvr {
		void register_lua_library(Lua::Interface &l);
		void close();
	};
};

#endif