// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

#include "stdafx_openvr.h"
#include "vr_eye.hpp"
#include "vr_instance.hpp"
#include "wvmodule.h"
#include <pragma/c_engine.h>
#include <pragma/game/c_game.h>
#include <pragma/entities/environment/c_env_camera.h>
#include <pragma/entities/entity_iterator.hpp>
#include <pragma/entities/c_world.h>
#include <prosper_context.hpp>
#include <image/prosper_image.hpp>
#include <prosper_util.hpp>
#include <image/prosper_sampler.hpp>
#include <image/prosper_render_target.hpp>
#include <prosper_fence.hpp>
#include <prosper_command_buffer.hpp>
#include <pragma/entities/entity_component_system_t.hpp>
#include <glm/ext/matrix_clip_space.hpp>

extern DLLCLIENT CEngine *c_engine;
extern DLLCLIENT CGame *c_game;

openvr::Eye::Eye(Instance &instance, vr::EVREye eye) : m_eye {eye}, m_instance {instance} {}

openvr::Eye::~Eye() {}

void openvr::Eye::ClearImage() { m_image = nullptr; }
void openvr::Eye::SetImage(prosper::IImage &img)
{
	m_image = img.shared_from_this();
	switch(m_instance.GetRenderAPI()) {
	case RenderAPI::Vulkan:
		{
			auto &renderContext = c_engine->GetRenderContext();
			auto &vrTextureData = (m_vrVkTextureData = vr::VRVulkanTextureData_t {});
			openvr::initialize_vulkan_texture_data(*vrTextureData, img);
			m_vrTexture = {&vrTextureData /* handle */, vr::TextureType_Vulkan, vr::EColorSpace::ColorSpace_Auto};
			break;
		}
	case RenderAPI::OpenGL:
		m_vrTexture = {const_cast<void *>(img.GetInternalHandle()), vr::TextureType_OpenGL, vr::EColorSpace::ColorSpace_Auto}; // ColorSpace_Gamma ?
		break;
	}
}

Mat4 openvr::Eye::GetEyeViewMatrix(pragma::CCameraComponent &cam) const
{
	auto *vrInterface = m_instance.GetSystemInterface();
	auto &matView = cam.GetViewMatrix();

	auto eyeTransform = vrInterface->GetEyeToHeadTransform(m_eye);
	Mat4 m(eyeTransform.m[0][0], eyeTransform.m[1][0], eyeTransform.m[2][0], 0.f, eyeTransform.m[0][1], eyeTransform.m[1][1], eyeTransform.m[2][1], 0.f, eyeTransform.m[0][2], eyeTransform.m[1][2], eyeTransform.m[2][2], 0.f, eyeTransform.m[0][3], eyeTransform.m[1][3], eyeTransform.m[2][3],
	  1.f);
	return matView * glm::inverse(m);
}
vr::EVREye &openvr::Eye::GetVREye() { return m_eye; }
vr::Texture_t &openvr::Eye::GetVRTexture() { return m_vrTexture; }
Mat4 openvr::Eye::GetEyeProjectionMatrix(float nearZ, float farZ) const
{
	auto *vrInterface = m_instance.GetSystemInterface();

	// Note: The documentation recommends using GetProjectionMatrix instead of
	// GetProjectionRaw, but I have no idea what format it's in and couldn't get it
	// to work with glm.
	// auto eyeProj = vrInterface->GetProjectionMatrix(eye,nearZ,farZ);
	// return Mat4{
	//	eyeProj.m[0][0],eyeProj.m[1][0],eyeProj.m[2][0],eyeProj.m[3][0],
	//	eyeProj.m[0][1],eyeProj.m[1][1],eyeProj.m[2][1],eyeProj.m[3][1],
	//	eyeProj.m[0][2],eyeProj.m[1][2],eyeProj.m[2][2],eyeProj.m[3][2],
	//	eyeProj.m[0][3],eyeProj.m[1][3],eyeProj.m[2][3],eyeProj.m[3][3]
	// };

	float left, right, top, bottom;
	vrInterface->GetProjectionRaw(m_eye, &left, &right, &top, &bottom);

	return glm::frustumRH(left, right, bottom, top, nearZ, farZ);
}
