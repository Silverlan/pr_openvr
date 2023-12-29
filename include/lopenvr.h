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