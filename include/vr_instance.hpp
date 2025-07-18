// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

#ifndef __VR_INSTANCE_HPP__
#define __VR_INSTANCE_HPP__

#include <openvr.h>
#include <string>
#include <cinttypes>
#include <sharedutils/functioncallback.h>
#include <sharedutils/util_weak_handle.hpp>
#include <unordered_map>
#include <deque>
#include <chrono>
#include <optional>
#include <mathutil/color.h>
#include <mathutil/umat.h>
#include <mathutil/uvec.h>
#include <mathutil/transform.hpp>
#include "vr_eye.hpp"

import pragma.platform;

struct lua_State;
namespace Lua {
	class Interface;
};
namespace prosper {
	class Shader;
	class IImage;
	class IPrimaryCommandBuffer;
	class IFence;
};

namespace openvr {
	enum class RenderAPI : uint8_t { OpenGL = 0, Vulkan };
	struct ControllerState;
	struct Eye;
	std::string to_string(uint32_t ev);
	std::string to_string(vr::VRCompositorError err);
	std::string to_string(vr::ETrackedPropertyError err);
	std::string to_string(vr::EVRInitError err);
	Mat4 steam_vr_matrix_to_engine_matrix(const vr::HmdMatrix34_t &matPose);
	bool is_hmd_present();
	void preinitialize_openvr();
	void initialize_vulkan_texture_data(vr::VRVulkanTextureData_t &vrTextureData, prosper::IImage &img);

	class IInstance {};

	class Instance {
	  public:
		~Instance();
		static std::unique_ptr<Instance> Create(vr::EVRInitError *err, std::vector<std::string> &reqInstanceExtensions, std::vector<std::string> &reqDeviceExtensions);
		Eye &GetLeftEye();
		const Eye &GetLeftEye() const;
		Eye &GetRightEye();
		const Eye &GetRightEye() const;

		void SetControllerStateCallback(const std::function<void(uint32_t, uint32_t, pragma::platform::KeyState)> &callback);
		std::string GetTrackedDeviceString(vr::TrackedDeviceIndex_t idx, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetTrackedDeviceString(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = nullptr) const;
		bool GetTrackedDeviceBool(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = nullptr) const;
		float GetTrackedDeviceFloat(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetTrackedDeviceInt32(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetTrackedDeviceUInt64(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = nullptr) const;
		Mat3x4 GetTrackedDeviceMatrix34(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = nullptr) const;
		vr::ETrackedControllerRole GetTrackedDeviceRole(uint32_t deviceIdx) const;
		vr::EDeviceActivityLevel GetTrackedDeviceActivityLevel(uint32_t deviceIdx) const;
		std::optional<std::string> GetTrackedDeviceSerialNumber(uint32_t deviceIdx) const;
		std::optional<std::string> GetTrackedDeviceType(uint32_t deviceIdx) const;
		std::optional<std::string> GetTrackedDeviceTrackingSystemName(uint32_t deviceIdx) const;
		std::optional<std::string> GetTrackedDeviceModelNumber(uint32_t deviceIdx) const;
		std::optional<std::string> GetTrackedDeviceRenderModelName(uint32_t deviceIdx) const;

		bool GetPoseTransform(uint32_t deviceIdx, vr::TrackedDevicePose_t &pose, Mat4 &m) const;
		std::string GetTrackingSystemName(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetModelNumber(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetSerialNumber(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetRenderModelName(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetManufacturerName(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetTrackingFirmwareVersion(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetHardwareRevision(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetAllWirelessDongleDescriptions(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetConnectedWirelessDongle(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetFirmwareManualUpdateURL(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetFirmwareProgrammingTarget(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetDisplayMCImageLeft(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetDisplayMCImageRight(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetDisplayGCImage(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetCameraFirmwareDescription(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetAttachedDeviceId(vr::TrackedPropertyError *peError = nullptr) const;
		std::string GetModelLabel(vr::TrackedPropertyError *peError = nullptr) const;

		bool WillDriftInYaw(vr::TrackedPropertyError *peError = nullptr) const;
		bool DeviceIsWireless(vr::TrackedPropertyError *peError = nullptr) const;
		bool DeviceIsCharging(vr::TrackedPropertyError *peError = nullptr) const;
		bool FirmwareUpdateAvailable(vr::TrackedPropertyError *peError = nullptr) const;
		bool FirmwareManualUpdate(vr::TrackedPropertyError *peError = nullptr) const;
		bool BlockServerShutdown(vr::TrackedPropertyError *peError = nullptr) const;
		bool CanUnifyCoordinateSystemWithHmd(vr::TrackedPropertyError *peError = nullptr) const;
		bool ContainsProximitySensor(vr::TrackedPropertyError *peError = nullptr) const;
		bool DeviceProvidesBatteryStatus(vr::TrackedPropertyError *peError = nullptr) const;
		bool DeviceCanPowerOff(vr::TrackedPropertyError *peError = nullptr) const;
		bool HasCamera(vr::TrackedPropertyError *peError = nullptr) const;
		bool ReportsTimeSinceVSync(vr::TrackedPropertyError *peError = nullptr) const;
		bool IsOnDesktop(vr::TrackedPropertyError *peError = nullptr) const;

		float GetDeviceBatteryPercentage(vr::TrackedPropertyError *peError = nullptr) const;
		float GetSecondsFromVsyncToPhotons(vr::TrackedPropertyError *peError = nullptr) const;
		float GetDisplayFrequency(vr::TrackedPropertyError *peError = nullptr) const;
		float GetUserIpdMeters(vr::TrackedPropertyError *peError = nullptr) const;
		float GetDisplayMCOffset(vr::TrackedPropertyError *peError = nullptr) const;
		float GetDisplayMCScale(vr::TrackedPropertyError *peError = nullptr) const;
		float GetDisplayGCBlackClamp(vr::TrackedPropertyError *peError = nullptr) const;
		float GetDisplayGCOffset(vr::TrackedPropertyError *peError = nullptr) const;
		float GetDisplayGCScale(vr::TrackedPropertyError *peError = nullptr) const;
		float GetDisplayGCPrescale(vr::TrackedPropertyError *peError = nullptr) const;
		float GetLensCenterLeftU(vr::TrackedPropertyError *peError = nullptr) const;
		float GetLensCenterLeftV(vr::TrackedPropertyError *peError = nullptr) const;
		Vector2 GetLensCenterLeftUV(vr::TrackedPropertyError *peError = nullptr) const;
		float GetLensCenterRightU(vr::TrackedPropertyError *peError = nullptr) const;
		float GetLensCenterRightV(vr::TrackedPropertyError *peError = nullptr) const;
		Vector2 GetLensCenterRightUV(vr::TrackedPropertyError *peError = nullptr) const;
		float GetUserHeadToEyeDepthMeters(vr::TrackedPropertyError *peError = nullptr) const;
		float GetFieldOfViewLeftDegrees(vr::TrackedPropertyError *peError = nullptr) const;
		float GetFieldOfViewRightDegrees(vr::TrackedPropertyError *peError = nullptr) const;
		float GetFieldOfViewTopDegrees(vr::TrackedPropertyError *peError = nullptr) const;
		float GetFieldOfViewBottomDegrees(vr::TrackedPropertyError *peError = nullptr) const;
		float GetTrackingRangeMinimumMeters(vr::TrackedPropertyError *peError = nullptr) const;
		float GetTrackingRangeMaximumMeters(vr::TrackedPropertyError *peError = nullptr) const;

		Mat3x4 GetStatusDisplayTransform(vr::TrackedPropertyError *peError = nullptr) const;
		Mat3x4 GetCameraToHeadTransform(vr::TrackedPropertyError *peError = nullptr) const;

		uint64_t GetHardwareRevisionNumber(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetFirmwareVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetFPGAVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetVRCVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetRadioVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetDongleVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetCurrentUniverseId(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetPreviousUniverseId(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetDisplayFirmwareVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetCameraFirmwareVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetDisplayFPGAVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetDisplayBootloaderVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetDisplayHardwareVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetAudioFirmwareVersion(vr::TrackedPropertyError *peError = nullptr) const;
		uint64_t GetSupportedButtons(vr::TrackedPropertyError *peError = nullptr) const;

		int32_t GetDeviceClass(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetDisplayMCType(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetEdidVendorID(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetEdidProductID(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetDisplayGCType(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetCameraCompatibilityMode(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetAxis0Type(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetAxis1Type(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetAxis2Type(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetAxis3Type(vr::TrackedPropertyError *peError = nullptr) const;
		int32_t GetAxis4Type(vr::TrackedPropertyError *peError = nullptr) const;

		vr::IVRSystem *GetSystemInterface();
		vr::IVRRenderModels *GetRenderInterface();
		vr::IVRCompositor *GetCompositorInterface();
		vr::IVRChaperone *GetChaperone();

		RenderAPI GetRenderAPI() const;
		void FadeToColor(Color col, float tFade, bool bBackground = false);
		void FadeGrid(float tFade, bool bFadeIn);
		void ShowMirrorWindow();
		void HideMirrorWindow();
		bool IsMirrorWindowVisible() const;
		void SetHmdViewEnabled(bool b);
		bool IsHmdViewEnabled() const;

		bool CanRenderScene() const;
		void ClearLastSubmittedFrame() const;
		void ClearSkyboxOverride() const;
		void CompositorBringToFront() const;
		void CompositorDumpImages() const;
		void CompositorGoToBack() const;
		void ForceInterleavedReprojectionOn(bool b) const;
		void ForceReconnectProcess() const;
		float GetFrameTimeRemaining() const;
		bool IsFullscreen() const;
		bool ShouldAppRenderWithLowResources() const;
		void SuspendRendering(bool b) const;
		bool IsRenderingSuspended() const;
		vr::EVRCompositorError SetSkyboxOverride(prosper::IImage &img) const;
		vr::EVRCompositorError SetSkyboxOverride(prosper::IImage &img, prosper::IImage &img2) const;
		vr::EVRCompositorError SetSkyboxOverride(prosper::IImage &front, prosper::IImage &back, prosper::IImage &left, prosper::IImage &right, prosper::IImage &top, prosper::IImage &bottom) const;
		vr::Compositor_CumulativeStats GetCumulativeStats() const;
		vr::ETrackingUniverseOrigin GetTrackingSpace() const;
		void SetTrackingSpace(vr::ETrackingUniverseOrigin space) const;

		void UpdateHMDPoses();
		std::chrono::steady_clock::duration GetPoseWaitTime() const;
		double GetSmoothedPoseWaitTime() const;

		void SetDeviceZeroPose(uint32_t deviceIndex, const umath::Transform &pose);
		const umath::Transform *GetInverseDeviceZeroPose(uint32_t deviceIndex) const;

		const Mat4 &GetHMDPoseMatrix() const;
		const Mat4 &GetPoseMatrix(uint32_t deviceIndex) const;
		const std::vector<vr::VREvent_t> &PollEvents();
	  private:
		Instance(vr::IVRSystem *system, RenderAPI renderAPI, vr::IVRRenderModels *i, vr::IVRCompositor *compositor
#ifdef USE_OPENGL_OFFSCREEN_CONTEXT
		  ,
		  GLFWwindow *window
#endif
		);
		vr::IVRSystem *m_system;
		vr::IVRRenderModels *m_renderInterface;
		vr::IVRCompositor *m_compositor;
		vr::IVRChaperone *m_chaperone;
		util::WeakHandle<prosper::Shader> m_hShaderFlip;
		std::unordered_map<uint32_t, ControllerState> m_controllerStates;
		std::array<Mat4, vr::k_unMaxTrackedDeviceCount> m_poseTransforms;
		std::vector<vr::VREvent_t> m_events {};
		std::chrono::steady_clock::duration m_poseWaitTime = std::chrono::nanoseconds {0};
		double m_smoothedPoseWaitTime = 0.0;
#ifdef USE_OPENGL_OFFSCREEN_CONTEXT
		GLFWwindow *m_window;
#endif
		std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> m_trackedPoses {};
		std::array<vr::TrackedDeviceClass, vr::k_unMaxTrackedDeviceCount> m_trackedDeviceClasses {};
		Mat4 m_hmdPoseMatrix = umat::identity();
		std::unique_ptr<Eye> m_leftEye;
		std::unique_ptr<Eye> m_rightEye;
		std::vector<umath::Transform> m_invDeviceZeroPoses;
		std::function<void(uint32_t, uint32_t, pragma::platform::KeyState)> m_controllerStateCallback = nullptr;
		bool m_bHmdViewEnabled = false;
		mutable bool m_isRenderingSuspended = false;
		RenderAPI m_renderAPI = RenderAPI::OpenGL;

		void OnControllerStateChanged(uint32_t controllerId, uint32_t key, pragma::platform::KeyState state);
		vr::EVRCompositorError SetSkyboxOverride(const std::vector<prosper::IImage *> &images) const;
	};
};

#endif
