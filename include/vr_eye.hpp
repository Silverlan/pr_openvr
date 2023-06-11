#ifndef __VR_EYE_HPP__
#define __VR_EYE_HPP__

#include <openvr.h>
#include <mathutil/umat.h>
#include <pragma/iscene.h>
#include <sharedutils/util_weak_handle.hpp>
#include <optional>

namespace prosper {
	class RenderTarget;
	class IImage;
	class IPrimaryCommandBuffer;
	class IFence;
};
namespace pragma {
	class CCameraComponent;
	namespace rendering {
		class BaseRenderer;
	};
};
namespace openvr {
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
		std::shared_ptr<pragma::rendering::BaseRenderer> m_renderer = nullptr;
		util::WeakHandle<pragma::CCameraComponent> m_camera {};
	};
};

#endif
