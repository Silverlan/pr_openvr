// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include <openvr.h>

export module pragma.modules.openvr:eye;

export import pragma.client;

export namespace openvr {
	class Instance;
	struct Eye {
	  public:
		Eye(Instance &instance, vr::EVREye eye);
		~Eye();

		void SetImage(prosper::IImage &src);
		void ClearImage();
		Mat4 GetEyeViewMatrix(pragma::CCameraComponent &cam) const;
		Mat4 GetEyeProjectionMatrix(float nearZ, float farZ) const;
		vr::EVREye &GetVREye();
		vr::Texture_t &GetVRTexture();
	  private:
		Instance &m_instance;
		std::shared_ptr<prosper::IImage> m_image = nullptr;
		std::optional<vr::VRVulkanTextureData_t> m_vrVkTextureData {};
		vr::Texture_t m_vrTexture;

		vr::EVREye m_eye;
		util::WeakHandle<pragma::CCameraComponent> m_camera {};
	};
};
