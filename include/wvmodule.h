// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

#ifndef __WVMODULE_H__
#define __WVMODULE_H__

#define LOPENVR_VERBOSE 0

#ifdef MODULE_SERVER
#include "pragma/iserver.h"
#define IState iserver
#pragma comment(lib, "IServer.lib")
#else
#define IState iclient
#pragma comment(lib, "IClient.lib")
#endif

#endif