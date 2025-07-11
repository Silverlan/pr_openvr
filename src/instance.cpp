// SPDX-FileCopyrightText: (c) 2020 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

#include "stdafx_openvr.h"
#include "vr_instance.hpp"
#include "vr_eye.hpp"
#include "vr_controller_state.hpp"
#include "wvmodule.h"
#include "vrincludes.h"
#include <sharedutils/scope_guard.h>
#include <array>
#include <GLFW/glfw3.h>
#include <image/prosper_render_target.hpp>
#include <prosper_context.hpp>
#include <prosper_command_buffer.hpp>
#include <prosper_fence.hpp>
#include <shader/prosper_shader.hpp>
#include <prosper_util.hpp>
#include <pragma/c_engine.h>
#include <pragma/iscene.h>
#include <pragma/game/c_game.h>
#include <pragma/entities/environment/c_env_camera.h>
#include <pragma/util/steam/util_steam.hpp>
#include <sharedutils/util_string.h>
#ifdef _DEBUG
#include <iostream>
#endif

extern DLLCLIENT CEngine *c_engine;
extern DLLCLIENT CGame *c_game;

using namespace openvr;

std::string openvr::to_string(uint32_t ev)
{
	switch(ev) {
	case vr::EVREventType::VREvent_None:
		return "None";
	case vr::EVREventType::VREvent_TrackedDeviceActivated:
		return "Tracked device activated";
	case vr::EVREventType::VREvent_TrackedDeviceDeactivated:
		return "Tracked device deactivated";
	case vr::EVREventType::VREvent_TrackedDeviceUpdated:
		return "Tracked device updated";
	case vr::EVREventType::VREvent_TrackedDeviceUserInteractionStarted:
		return "Tracked device user interaction started";
	case vr::EVREventType::VREvent_TrackedDeviceUserInteractionEnded:
		return "Tracked device user interaction ended";
	case vr::EVREventType::VREvent_IpdChanged:
		return "Ipd changed";
	case vr::EVREventType::VREvent_EnterStandbyMode:
		return "Enter standby mode";
	case vr::EVREventType::VREvent_LeaveStandbyMode:
		return "Leave standby mode";
	case vr::EVREventType::VREvent_TrackedDeviceRoleChanged:
		return "Tracked device role changed";
	case vr::EVREventType::VREvent_ButtonPress:
		return "Button press";
	case vr::EVREventType::VREvent_ButtonUnpress:
		return "Button unpress";
	case vr::EVREventType::VREvent_ButtonTouch:
		return "Button touch";
	case vr::EVREventType::VREvent_ButtonUntouch:
		return "Button untouch";
	case vr::EVREventType::VREvent_MouseMove:
		return "Mouse move";
	case vr::EVREventType::VREvent_MouseButtonDown:
		return "Mouse button down";
	case vr::EVREventType::VREvent_MouseButtonUp:
		return "Mouse button up";
	case vr::EVREventType::VREvent_FocusEnter:
		return "Focus enter";
	case vr::EVREventType::VREvent_FocusLeave:
		return "Focus leave";
	case vr::EVREventType::VREvent_ScrollDiscrete:
		return "ScrollDiscrete";
	case vr::EVREventType::VREvent_ScrollSmooth:
		return "ScrollSmooth";
	case vr::EVREventType::VREvent_TouchPadMove:
		return "Touch pad move";
	case vr::EVREventType::VREvent_InputFocusCaptured:
		return "Input focus captured";
	case vr::EVREventType::VREvent_InputFocusReleased:
		return "Input focus released";
	case vr::EVREventType::VREvent_SceneApplicationChanged:
		return "Scene focus changed";
	case vr::EVREventType::VREvent_HideRenderModels:
		return "Hide render models";
	case vr::EVREventType::VREvent_ShowRenderModels:
		return "Show render models";
	case vr::EVREventType::VREvent_OverlayShown:
		return "Overlay shown";
	case vr::EVREventType::VREvent_OverlayHidden:
		return "Overlay hidden";
	case vr::EVREventType::VREvent_DashboardActivated:
		return "Dashboard activated";
	case vr::EVREventType::VREvent_DashboardDeactivated:
		return "Dashboard deactivated";
	case vr::EVREventType::VREvent_DashboardRequested:
		return "Dashboard requested";
	case vr::EVREventType::VREvent_ResetDashboard:
		return "Reset dashboard";
	// case vr::EVREventType::VREvent_RenderToast:
	// 	return "Render toast";
	case vr::EVREventType::VREvent_ImageLoaded:
		return "Image loaded";
	case vr::EVREventType::VREvent_ShowKeyboard:
		return "Show keyboard";
	case vr::EVREventType::VREvent_HideKeyboard:
		return "Hide keyboard";
	case vr::EVREventType::VREvent_OverlayGamepadFocusGained:
		return "Overlay gamepad focus gained";
	case vr::EVREventType::VREvent_OverlayGamepadFocusLost:
		return "Overlay gamepad focus lost";
	case vr::EVREventType::VREvent_OverlaySharedTextureChanged:
		return "Overlay shared texture changed";
	case vr::EVREventType::VREvent_Notification_Shown:
		return "Notification shown";
	case vr::EVREventType::VREvent_Notification_Hidden:
		return "Notification hidden";
	case vr::EVREventType::VREvent_Notification_BeginInteraction:
		return "Begin interaction";
	case vr::EVREventType::VREvent_Notification_Destroyed:
		return "Notification destroyed";
	case vr::EVREventType::VREvent_Quit:
		return "Quit";
	case vr::EVREventType::VREvent_ProcessQuit:
		return "Process quit";
	case vr::EVREventType::VREvent_QuitAcknowledged:
		return "Quit acknowledged";
	case vr::EVREventType::VREvent_ChaperoneDataHasChanged:
		return "Chaperone data has changed";
	case vr::EVREventType::VREvent_ChaperoneUniverseHasChanged:
		return "Chaperone universe has changed";
	case vr::EVREventType::VREvent_ChaperoneTempDataHasChanged:
		return "Chaperone temp data has changed";
	case vr::EVREventType::VREvent_ChaperoneSettingsHaveChanged:
		return "Chaperone settings have changed";
	case vr::EVREventType::VREvent_SeatedZeroPoseReset:
		return "Seated zero pose reset";
	case vr::EVREventType::VREvent_BackgroundSettingHasChanged:
		return "Background setting has changed";
	case vr::EVREventType::VREvent_CameraSettingsHaveChanged:
		return "Camera settings have changed";
	case vr::EVREventType::VREvent_StatusUpdate:
		return "Status update";
	case vr::EVREventType::VREvent_MCImageUpdated:
		return "MC image updated";
	case vr::EVREventType::VREvent_FirmwareUpdateStarted:
		return "Firmware update started";
	case vr::EVREventType::VREvent_FirmwareUpdateFinished:
		return "Firmware update finished";
	case vr::EVREventType::VREvent_KeyboardClosed:
		return "Keyboard closed";
	case vr::EVREventType::VREvent_KeyboardCharInput:
		return "Keyboard char input";
	case vr::EVREventType::VREvent_KeyboardDone:
		return "Keyboard done";
	case vr::EVREventType::VREvent_Compositor_ChaperoneBoundsShown:
		return "Composition chaperone bounds shown";
	case vr::EVREventType::VREvent_Compositor_ChaperoneBoundsHidden:
		return "Compositor chaperone bounds hidden";
	case vr::EVREventType::VREvent_TrackedCamera_StartVideoStream:
		return "Tracked camera start video stream";
	case vr::EVREventType::VREvent_TrackedCamera_StopVideoStream:
		return "Tracked camera stop video stream";
	case vr::EVREventType::VREvent_TrackedCamera_PauseVideoStream:
		return "Tracked camera paused video stream";
	case vr::EVREventType::VREvent_TrackedCamera_ResumeVideoStream:
		return "Tracked camera resume video stream";
	case vr::EVREventType::VREvent_PerformanceTest_EnableCapture:
		return "Performance test enable capture";
	case vr::EVREventType::VREvent_PerformanceTest_DisableCapture:
		return "Performance test disable capture";
	case vr::EVREventType::VREvent_PerformanceTest_FidelityLevel:
		return "Performance test fidelity level";
	case vr::EVREventType::VREvent_WatchdogWakeUpRequested:
		return "Watchdog wake up requested";
	case vr::EVREventType::VREvent_LensDistortionChanged:
		return "Lens distortion changed";
	case vr::EVREventType::VREvent_PropertyChanged:
		return "Property changed";
	case vr::EVREventType::VREvent_WirelessDisconnect:
		return "Wireless disconnect";
	case vr::EVREventType::VREvent_WirelessReconnect:
		return "Wireless reconnect";
	case vr::EVREventType::VREvent_Modal_Cancel:
		return "Modal cancel";
	case vr::EVREventType::VREvent_OverlayFocusChanged:
		return "Overlay focus changed";
	case vr::EVREventType::VREvent_ReloadOverlays:
		return "Reload overlays";
	case vr::EVREventType::VREvent_LockMousePosition:
		return "Lock mouse position";
	case vr::EVREventType::VREvent_UnlockMousePosition:
		return "Unlock mouse position";
	case vr::EVREventType::VREvent_InputFocusChanged:
		return "Input focus changed";
	case vr::EVREventType::VREvent_SceneApplicationUsingWrongGraphicsAdapter:
		return "Scene application using wrong graphics adapter";
	case vr::EVREventType::VREvent_ActionBindingReloaded:
		return "Action binding reloaded";
	case vr::EVREventType::VREvent_SceneApplicationStateChanged:
		return "Scene application state changed";
	case vr::EVREventType::VREvent_ConsoleOpened:
		return "Console opened";
	case vr::EVREventType::VREvent_ConsoleClosed:
		return "Console closed";
	case vr::EVREventType::VREvent_ScreenshotTriggered:
		return "Screenshot triggered";
	case vr::EVREventType::VREvent_ImageFailed:
		return "Image failed";
	case vr::EVREventType::VREvent_DashboardOverlayCreated:
		return "Dashboard overlay created";
	case vr::EVREventType::VREvent_SwitchGamepadFocus:
		return "Switch gamepad focus";
	case vr::EVREventType::VREvent_RequestScreenshot:
		return "Request screenshot";
	case vr::EVREventType::VREvent_ScreenshotTaken:
		return "Screenshot taken";
	case vr::EVREventType::VREvent_ScreenshotFailed:
		return "Screenshot failed";
	case vr::EVREventType::VREvent_SubmitScreenshotToDashboard:
		return "Submit screenshot to dashboard";
	case vr::EVREventType::VREvent_ScreenshotProgressToDashboard:
		return "Screenshot progress to dashboard";
	case vr::EVREventType::VREvent_PrimaryDashboardDeviceChanged:
		return "Primary dashboard device changed";
	case vr::EVREventType::VREvent_RoomViewShown:
		return "Room view shown";
	case vr::EVREventType::VREvent_RoomViewHidden:
		return "Room view hidden";
	case vr::EVREventType::VREvent_ShowUI:
		return "Show UI";
	case vr::EVREventType::VREvent_ShowDevTools:
		return "Show dev tools";
	case vr::EVREventType::VREvent_DesktopViewUpdating:
		return "Desktop view updating";
	case vr::EVREventType::VREvent_DesktopViewReady:
		return "Desktop view ready";
	case vr::EVREventType::VREvent_DriverRequestedQuit:
		return "Driver requested quit";
	case vr::EVREventType::VREvent_RestartRequested:
		return "Restart requested";
	case vr::EVREventType::VREvent_ChaperoneFlushCache:
		return "Chaperone flush cache";
	case vr::EVREventType::VREvent_ChaperoneRoomSetupStarting:
		return "Chaperone room setup starting";
	case vr::EVREventType::VREvent_ChaperoneRoomSetupFinished:
		return "Chaperone room setup finished";
	case vr::EVREventType::VREvent_StandingZeroPoseReset:
		return "Standing zero pose reset";
	case vr::EVREventType::VREvent_AudioSettingsHaveChanged:
		return "Audio settings have changed";
	case vr::EVREventType::VREvent_ReprojectionSettingHasChanged:
		return "Reprojection setting has changed";
	case vr::EVREventType::VREvent_ModelSkinSettingsHaveChanged:
		return "Model skin settings have changed";
	case vr::EVREventType::VREvent_EnvironmentSettingsHaveChanged:
		return "Environment settings have changed";
	case vr::EVREventType::VREvent_PowerSettingsHaveChanged:
		return "Power settings have changed";
	case vr::EVREventType::VREvent_EnableHomeAppSettingsHaveChanged:
		return "Enable home app settings have changed";
	case vr::EVREventType::VREvent_SteamVRSectionSettingChanged:
		return "SteamVR section setting changed";
	case vr::EVREventType::VREvent_LighthouseSectionSettingChanged:
		return "Lighthouse section setting changed";
	case vr::EVREventType::VREvent_NullSectionSettingChanged:
		return "Null section setting changed";
	case vr::EVREventType::VREvent_UserInterfaceSectionSettingChanged:
		return "User interface section setting changed";
	case vr::EVREventType::VREvent_NotificationsSectionSettingChanged:
		return "Notification section setting changed";
	case vr::EVREventType::VREvent_KeyboardSectionSettingChanged:
		return "Keyboard section setting changed";
	case vr::EVREventType::VREvent_PerfSectionSettingChanged:
		return "Perf section setting changed";
	case vr::EVREventType::VREvent_DashboardSectionSettingChanged:
		return "Dashboard section setting changed";
	case vr::EVREventType::VREvent_WebInterfaceSectionSettingChanged:
		return "Web interface section setting changed";
	case vr::EVREventType::VREvent_TrackersSectionSettingChanged:
		return "Trackers section setting changed";
	case vr::EVREventType::VREvent_LastKnownSectionSettingChanged:
		return "Last known section setting changed";
	case vr::EVREventType::VREvent_DismissedWarningsSectionSettingChanged:
		return "Dismissed warnings section setting changed";
	case vr::EVREventType::VREvent_GpuSpeedSectionSettingChanged:
		return "GPU speed section setting changed";
	case vr::EVREventType::VREvent_WindowsMRSectionSettingChanged:
		return "Windows MR section setting changed";
	case vr::EVREventType::VREvent_OtherSectionSettingChanged:
		return "Other section setting changed";
	case vr::EVREventType::VREvent_WebInterface_InstallDriverCompleted:
		return "Web interface install driver completed";
	case vr::EVREventType::VREvent_ApplicationListUpdated:
		return "Application list updated";
	case vr::EVREventType::VREvent_ApplicationMimeTypeLoad:
		return "Application mime type load";
	case vr::EVREventType::VREvent_ProcessConnected:
		return "Process connected";
	case vr::EVREventType::VREvent_ProcessDisconnected:
		return "Process disconnected";
	case vr::EVREventType::VREvent_Compositor_DisplayDisconnected:
		return "Compositor display connected";
	case vr::EVREventType::VREvent_Compositor_DisplayReconnected:
		return "Compositor display reconnected";
	case vr::EVREventType::VREvent_Compositor_HDCPError:
		return "Compositor HDCP error";
	case vr::EVREventType::VREvent_Compositor_ApplicationNotResponding:
		return "Compositor application not responding";
	case vr::EVREventType::VREvent_Compositor_ApplicationResumed:
		return "Compositor application resumed";
	case vr::EVREventType::VREvent_Compositor_OutOfVideoMemory:
		return "Compositor out of video memory";
	case vr::EVREventType::VREvent_Compositor_DisplayModeNotSupported:
		return "Compositor display mode not supported";
	case vr::EVREventType::VREvent_Compositor_StageOverrideReady:
		return "Compositor stage override ready";
	case vr::EVREventType::VREvent_TrackedCamera_EditingSurface:
		return "Tracked camera editing surface";
	case vr::EVREventType::VREvent_MessageOverlay_Closed:
		return "Message overlay closed";
	case vr::EVREventType::VREvent_MessageOverlayCloseRequested:
		return "Message overlay close requested";
	case vr::EVREventType::VREvent_Input_HapticVibration:
		return "Input haptic vibration";
	case vr::EVREventType::VREvent_Input_BindingLoadFailed:
		return "Input binding load failed";
	case vr::EVREventType::VREvent_Input_BindingLoadSuccessful:
		return "Input binding load successful";
	case vr::EVREventType::VREvent_Input_ActionManifestReloaded:
		return "Input action manifest reloaded";
	case vr::EVREventType::VREvent_Input_ActionManifestLoadFailed:
		return "Input action manifest load failed";
	case vr::EVREventType::VREvent_Input_ProgressUpdate:
		return "Input progress update";
	case vr::EVREventType::VREvent_Input_TrackerActivated:
		return "Input tracker activated";
	case vr::EVREventType::VREvent_Input_BindingsUpdated:
		return "Input bindings updated";
	case vr::EVREventType::VREvent_Input_BindingSubscriptionChanged:
		return "Input binding subscription changed";
	case vr::EVREventType::VREvent_SpatialAnchors_PoseUpdated:
		return "Spatial anchors pose updated";
	case vr::EVREventType::VREvent_SpatialAnchors_DescriptorUpdated:
		return "Spatial anchors descriptor updated";
	case vr::EVREventType::VREvent_SpatialAnchors_RequestPoseUpdate:
		return "Spatial anchors request pose update";
	case vr::EVREventType::VREvent_SpatialAnchors_RequestDescriptorUpdate:
		return "Spatial anchors request descriptor update";
	case vr::EVREventType::VREvent_SystemReport_Started:
		return "System report started";
	case vr::EVREventType::VREvent_Monitor_ShowHeadsetView:
		return "Monitor show headset view";
	case vr::EVREventType::VREvent_Monitor_HideHeadsetView:
		return "Monitor hide headset view";
	}
	if(ev >= vr::EVREventType::VREvent_VendorSpecific_Reserved_Start && ev <= vr::EVREventType::VREvent_VendorSpecific_Reserved_End)
		return "Unknown (Vendor specific)";
	return "Invalid";
}

std::string openvr::to_string(vr::VRCompositorError err)
{
	switch(err) {
	case vr::VRCompositorError::VRCompositorError_None:
		return "None";
	case vr::VRCompositorError::VRCompositorError_IncompatibleVersion:
		return "Incompatible version";
	case vr::VRCompositorError::VRCompositorError_DoNotHaveFocus:
		return "Do not have focus";
	case vr::VRCompositorError::VRCompositorError_InvalidTexture:
		return "Invalid texture";
	case vr::VRCompositorError::VRCompositorError_IsNotSceneApplication:
		return "Is not scene application";
	case vr::VRCompositorError::VRCompositorError_TextureIsOnWrongDevice:
		return "Texture is on wrong device";
	case vr::VRCompositorError::VRCompositorError_TextureUsesUnsupportedFormat:
		return "Texture uses unsupported format";
	case vr::VRCompositorError::VRCompositorError_SharedTexturesNotSupported:
		return "Shared textures not supported";
	case vr::VRCompositorError::VRCompositorError_IndexOutOfRange:
		return "Index out of range";
	}
	return "Invalid";
}

std::string openvr::to_string(vr::ETrackedPropertyError err)
{
	switch(err) {
	case vr::ETrackedPropertyError::TrackedProp_Success:
		return "Success";
	case vr::ETrackedPropertyError::TrackedProp_WrongDataType:
		return "Wrong data type";
	case vr::ETrackedPropertyError::TrackedProp_WrongDeviceClass:
		return "Wrong device class";
	case vr::ETrackedPropertyError::TrackedProp_BufferTooSmall:
		return "Buffer too small";
	case vr::ETrackedPropertyError::TrackedProp_UnknownProperty:
		return "Unknown property";
	case vr::ETrackedPropertyError::TrackedProp_InvalidDevice:
		return "Invalid device";
	case vr::ETrackedPropertyError::TrackedProp_CouldNotContactServer:
		return "Could not contact server";
	case vr::ETrackedPropertyError::TrackedProp_ValueNotProvidedByDevice:
		return "Value not provided by device";
	case vr::ETrackedPropertyError::TrackedProp_StringExceedsMaximumLength:
		return "String exceeds maximum length";
	case vr::ETrackedPropertyError::TrackedProp_NotYetAvailable:
		return "Not yet available";
	}
	return "Invalid";
}

std::string openvr::to_string(vr::EVRInitError err) { return vr::VR_GetVRInitErrorAsEnglishDescription(err); }

//////////////////////////////////

struct OpenVrInitializer {
	OpenVrInitializer();
	~OpenVrInitializer();
	void Initialize(bool wait = false);

	vr::IVRSystem *GetSystem()
	{
		Initialize(true);
		return m_ivrSystem;
	}
	vr::EVRInitError &GetError()
	{
		Initialize(true);
		return m_error;
	}
  private:
	enum class State : uint8_t { Initial = 0, Initializing, InitializationComplete };
	std::thread m_thread;
	std::atomic<State> m_state = State::Initial;
	vr::IVRSystem *m_ivrSystem = nullptr;
	vr::EVRInitError m_error = {};
};

OpenVrInitializer::OpenVrInitializer() {}
void OpenVrInitializer::Initialize(bool wait)
{
	if(m_state == State::Initial) {
		m_state = State::Initializing;

#ifdef __linux__
		// If the application was not launched through steam, and SteamVR is not already running,
		// initializing SteamVR will fail because it's unable to find the qt binaries.
		// As a workaround, we try to find the steam installation location and add the binary path
		// to the path lookup env variable.
		// (Alternatively the application can be added as a non-steam game in Steam and then started through steam.)
		auto curLibPath = util::get_env_variable("LD_LIBRARY_PATH");
		if(curLibPath.has_value()) {
			auto steamPaths = util::steam::find_steam_root_paths();
			std::unordered_map<std::string, bool> libPaths {
				{"steamapps/common/SteamVR/bin/linux64/qt/lib", false},
				{"steamapps/common/SteamVR/bin/linux64", false}
			};
			for(auto &steamPath : steamPaths) {
				for(auto &[relLibPath, b] : libPaths) {
					if(b)
						continue;
					auto libPath = util::DirPath(steamPath, relLibPath);
					if(filemanager::exists(libPath.GetString())) {
						auto newLibPath = *curLibPath + ":" +libPath.GetString();
						util::set_env_variable("LD_LIBRARY_PATH", newLibPath);
						b = true;
					}
				}
			}
		}
#endif

		m_thread = std::thread {[this]() {
			m_ivrSystem = vr::VR_Init(&m_error, vr::EVRApplicationType::VRApplication_Scene);
			m_state = State::InitializationComplete;
		}};
		util::set_thread_name(m_thread, "openvr_init");
	}
	if(wait && m_thread.joinable())
		m_thread.join();
}
OpenVrInitializer::~OpenVrInitializer()
{
	if(m_state != State::Initial && m_thread.joinable())
		m_thread.join();
	if(m_ivrSystem)
		vr::VR_Shutdown();
}

void openvr::initialize_vulkan_texture_data(vr::VRVulkanTextureData_t &vrTextureData, prosper::IImage &img)
{
	auto &renderContext = c_engine->GetRenderContext();
	vrTextureData.m_nImage = reinterpret_cast<uint64_t>(img.GetInternalHandle());
	vrTextureData.m_pDevice = static_cast<VkDevice_T *>(renderContext.GetInternalDevice());
	vrTextureData.m_pPhysicalDevice = static_cast<VkPhysicalDevice_T *>(renderContext.GetInternalPhysicalDevice());
	vrTextureData.m_pInstance = static_cast<VkInstance_T *>(renderContext.GetInternalInstance());
	vrTextureData.m_pQueue = static_cast<VkQueue_T *>(renderContext.GetInternalUniversalQueue());
	vrTextureData.m_nQueueFamilyIndex = renderContext.GetUniversalQueueFamilyIndex();
	vrTextureData.m_nWidth = img.GetWidth();
	vrTextureData.m_nHeight = img.GetHeight();
	vrTextureData.m_nFormat = umath::to_integral(img.GetFormat());
	vrTextureData.m_nSampleCount = umath::to_integral(img.GetSampleCount());
}

static std::unique_ptr<OpenVrInitializer> g_openVrInitializer = nullptr;

void openvr::preinitialize_openvr()
{
	if(g_openVrInitializer)
		return;
	g_openVrInitializer = std::make_unique<OpenVrInitializer>();
	g_openVrInitializer->Initialize();
}

static vr::IVRSystem *initialize_openvr(vr::EVRInitError *err)
{
	openvr::preinitialize_openvr();
	if(err)
		*err = g_openVrInitializer->GetError();
	return g_openVrInitializer->GetSystem();
}

bool openvr::is_hmd_present() { return vr::VR_IsHmdPresent(); }

/////////////

Instance::Instance(vr::IVRSystem *system, RenderAPI renderAPI, vr::IVRRenderModels *i, vr::IVRCompositor *compositor) : m_system {system}, m_renderInterface {i}, m_compositor {compositor}, m_renderAPI {renderAPI}, m_chaperone {vr::VRChaperone()}
{
	m_leftEye = std::make_unique<Eye>(*this, vr::EVREye::Eye_Left);
	m_rightEye = std::make_unique<Eye>(*this, vr::EVREye::Eye_Right);
	auto *shaderFlip = IState::get_shader("screen_flip_y");
	m_hShaderFlip = (shaderFlip != nullptr) ? shaderFlip->GetHandle() : util::WeakHandle<prosper::Shader> {};
#ifdef _DEBUG
	m_compositor->ShowMirrorWindow();
#endif
}
Instance::~Instance() { vr::VR_Shutdown(); }
const std::vector<vr::VREvent_t> &Instance::PollEvents()
{
	m_events.clear();
	vr::VREvent_t event;
	while(m_system->PollNextEvent(&event, sizeof(event)))
		m_events.push_back(std::move(event));

	vr::VRControllerState_t state {};
	auto *sys = GetSystemInterface();
	for(auto i = decltype(vr::k_unMaxTrackedDeviceCount) {0}; i < vr::k_unMaxTrackedDeviceCount; ++i) {
		if(sys->GetControllerState(i, &state, sizeof(vr::VRControllerState_t)) == false)
			continue;
		auto it = m_controllerStates.find(i);
		if(it == m_controllerStates.end()) {
			it = m_controllerStates.insert(std::make_pair(i, ControllerState {})).first;
			it->second.SetStateChangeCallback(std::bind(&Instance::OnControllerStateChanged, this, i, std::placeholders::_1, std::placeholders::_2));
		}
		it->second.UpdateState(state);
	}
	return m_events;
}
void Instance::OnControllerStateChanged(uint32_t controllerId, uint32_t key, pragma::platform::KeyState state)
{
	if(m_controllerStateCallback == nullptr)
		return;
	m_controllerStateCallback(controllerId, key, state);
}
void Instance::SetControllerStateCallback(const std::function<void(uint32_t, uint32_t, pragma::platform::KeyState)> &callback) { m_controllerStateCallback = callback; }
/*bool Instance::InitializeScene()
{
	auto &context = IState::get_render_context();
#if LOPENVR_VERBOSE == 1
		std::cout<<"[VR] Initializing eyes..."<<std::endl;
#endif
	return (m_leftEye->Initialize(*this) == true && m_rightEye->Initialize(*this) == true) ? true : false;
}*/
openvr::Eye &Instance::GetLeftEye() { return *m_leftEye; }
const openvr::Eye &Instance::GetLeftEye() const { return const_cast<Instance *>(this)->GetLeftEye(); }
openvr::Eye &Instance::GetRightEye() { return *m_rightEye; }
const openvr::Eye &Instance::GetRightEye() const { return const_cast<Instance *>(this)->GetRightEye(); }

Mat4 openvr::steam_vr_matrix_to_engine_matrix(const vr::HmdMatrix34_t &matPose)
{
	return Mat4(matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.f, matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.f, matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.f, matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.f);
}

const Mat4 &Instance::GetHMDPoseMatrix() const { return m_hmdPoseMatrix; }
const Mat4 &Instance::GetPoseMatrix(uint32_t deviceIndex) const { return m_poseTransforms.at(deviceIndex); }

void Instance::SetDeviceZeroPose(uint32_t deviceIndex, const umath::Transform &pose)
{
	if(deviceIndex >= m_invDeviceZeroPoses.size())
		m_invDeviceZeroPoses.resize(deviceIndex + 1);
	m_invDeviceZeroPoses[deviceIndex] = pose.GetInverse();
}
const umath::Transform *Instance::GetInverseDeviceZeroPose(uint32_t deviceIndex) const
{
	if(deviceIndex >= m_invDeviceZeroPoses.size())
		return nullptr;
	return &m_invDeviceZeroPoses[deviceIndex];
}

std::chrono::steady_clock::duration Instance::GetPoseWaitTime() const { return m_poseWaitTime; }
double Instance::GetSmoothedPoseWaitTime() const { return m_smoothedPoseWaitTime; }

void Instance::UpdateHMDPoses()
{
	auto t = std::chrono::steady_clock::now();
	m_compositor->WaitGetPoses(m_trackedPoses.data(), m_trackedPoses.size(), nullptr, 0);
	m_poseWaitTime = std::chrono::steady_clock::now() - t;
	constexpr float weightRatio = 0.8f;
	auto poseWaitTimeMs = m_poseWaitTime.count() / 1'000'000.0;
	m_smoothedPoseWaitTime = poseWaitTimeMs * (1.0 - weightRatio) + m_smoothedPoseWaitTime * weightRatio;

	auto validPoseCount = 0;
	auto nDevice = 0u;
	for(auto &pose : m_trackedPoses) {
		if(pose.bPoseIsValid == false) {
			++nDevice;
			continue;
		}
		validPoseCount++;
		m_poseTransforms.at(nDevice) = steam_vr_matrix_to_engine_matrix(pose.mDeviceToAbsoluteTracking);
		if(m_trackedDeviceClasses.at(nDevice) == 0)
			m_trackedDeviceClasses.at(nDevice) = m_system->GetTrackedDeviceClass(nDevice);
		++nDevice;
	}
	auto &hmdPose = m_trackedPoses.at(vr::k_unTrackedDeviceIndex_Hmd);
	if(hmdPose.bPoseIsValid == true) {
		m_hmdPoseMatrix = m_poseTransforms.at(vr::k_unTrackedDeviceIndex_Hmd);
		//m_hmdPoseMatrix = glm::inverse(m_hmdPoseMatrix);
	}
}

static bool check_error(vr::EVRCompositorError err)
{
	if(err == vr::EVRCompositorError::VRCompositorError_None)
		return true;
	std::cout << "[VR] Eye submit error";
	std::cout << ": " << openvr::to_string(err) << std::endl;
	return false;
};

vr::IVRSystem *Instance::GetSystemInterface() { return m_system; }
vr::IVRRenderModels *Instance::GetRenderInterface() { return m_renderInterface; }
vr::IVRCompositor *Instance::GetCompositorInterface() { return m_compositor; }
vr::IVRChaperone *Instance::GetChaperone() { return m_chaperone; }

RenderAPI Instance::GetRenderAPI() const { return m_renderAPI; }
void Instance::FadeToColor(Color col, float tFade, bool bBackground) { m_compositor->FadeToColor(tFade, static_cast<float>(col.r) / 255.f, static_cast<float>(col.g) / 255.f, static_cast<float>(col.b) / 255.f, static_cast<float>(col.a) / 255.f, bBackground); }
void Instance::FadeGrid(float tFade, bool bFadeIn) { m_compositor->FadeGrid(tFade, bFadeIn); }
void Instance::ShowMirrorWindow() { m_compositor->ShowMirrorWindow(); }
void Instance::HideMirrorWindow() { m_compositor->HideMirrorWindow(); }
bool Instance::IsMirrorWindowVisible() const { return m_compositor->IsMirrorWindowVisible(); }
void Instance::SetHmdViewEnabled(bool b) { m_bHmdViewEnabled = b; }
bool Instance::IsHmdViewEnabled() const { return m_bHmdViewEnabled; }

std::string Instance::GetTrackedDeviceString(vr::TrackedDeviceIndex_t idx, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError) const
{
	auto unRequiredBufferLen = m_system->GetStringTrackedDeviceProperty(idx, prop, nullptr, 0, peError);
	if(unRequiredBufferLen == 0)
		return "";
	std::vector<char> r(unRequiredBufferLen);
	unRequiredBufferLen = m_system->GetStringTrackedDeviceProperty(idx, prop, r.data(), unRequiredBufferLen, peError);
	return std::string {r.data(), r.size() - 1};
}
std::string Instance::GetTrackedDeviceString(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd, prop, peError); }
bool Instance::GetTrackedDeviceBool(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError) const { return m_system->GetBoolTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, prop, peError); }
float Instance::GetTrackedDeviceFloat(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError) const { return m_system->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, prop, peError); }
int32_t Instance::GetTrackedDeviceInt32(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError) const { return m_system->GetInt32TrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, prop, peError); }
uint64_t Instance::GetTrackedDeviceUInt64(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError) const { return m_system->GetUint64TrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, prop, peError); }
glm::mat3x4 Instance::GetTrackedDeviceMatrix34(vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError) const
{
	auto m = m_system->GetMatrix34TrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, prop, peError);
	glm::mat3x4 r {};
	for(int32_t i = 0; i < 3; ++i) {
		for(int32_t j = 0; j < 4; ++j)
			r[i][j] = m.m[i][j];
	}
	return r;
}
std::unique_ptr<Instance> Instance::Create(vr::EVRInitError *err, std::vector<std::string> &reqInstanceExtensions, std::vector<std::string> &reqDeviceExtensions)
{
	if(vr::VR_IsHmdPresent() == false) {
		if(err != nullptr)
			*err = vr::EVRInitError::VRInitError_Init_HmdNotFound;
		return nullptr;
	}
	auto *ivr = initialize_openvr(err);
	if(ivr == nullptr)
		return nullptr;
	util::ScopeGuard guard {[]() { vr::VR_Shutdown(); }};
	auto *pRenderModels = static_cast<vr::IVRRenderModels *>(vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, err));
	if(pRenderModels == nullptr)
		return nullptr;
	auto *pCompositor = vr::VRCompositor();
	if(pCompositor == nullptr) {
		if(err != nullptr)
			*err = vr::EVRInitError::VRInitError_Compositor_Failed;
		return nullptr;
	}

	auto renderAPI = c_engine->GetRenderContext().GetAPIIdentifier();
	RenderAPI eRenderAPI;
	if(ustring::compare<std::string>(renderAPI, "OpenGL"))
		eRenderAPI = RenderAPI::OpenGL;
	else if(ustring::compare<std::string>(renderAPI, "Vulkan"))
		eRenderAPI = RenderAPI::Vulkan;
	else
		return nullptr;

	if(eRenderAPI == RenderAPI::Vulkan) {
		auto instanceExtLen = pCompositor->GetVulkanInstanceExtensionsRequired(nullptr, 0);
		std::string instanceExt;
		if(instanceExtLen > 0) {
			instanceExt.resize(instanceExtLen);
			pCompositor->GetVulkanInstanceExtensionsRequired(const_cast<char *>(instanceExt.data()), 0);
		}
		auto &vkContext = IState::get_render_context();
		auto *vkDevice = static_cast<VkPhysicalDevice_T *>(vkContext.GetInternalPhysicalDevice());
		auto deviceExtLen = pCompositor->GetVulkanDeviceExtensionsRequired(vkDevice, nullptr, 0);
		std::string deviceExt;
		if(deviceExtLen > 0) {
			deviceExt.resize(deviceExtLen);
			pCompositor->GetVulkanDeviceExtensionsRequired(vkDevice, const_cast<char *>(deviceExt.data()), 0);
		}
		pCompositor->SetExplicitTimingMode(vr::EVRCompositorTimingMode::VRCompositorTimingMode_Explicit_RuntimePerformsPostPresentHandoff);

		ustring::explode(instanceExt, " ", reqInstanceExtensions);
		ustring::explode(deviceExt, " ", reqDeviceExtensions);
		std::cout << "[VR] Instance Extensions Required: " << instanceExt << std::endl;
		std::cout << "[VR] Device Extensions Required: " << deviceExt << std::endl;

		for(auto &ext : reqInstanceExtensions) {
			if(vkContext.IsInstanceExtensionEnabled(ext) == false) {
				Con::cerr << "[VR] ERROR: Required instance extension '" << ext << "' is not enabled!" << Con::endl;
				break;
			}
		}
		for(auto &ext : reqDeviceExtensions) {
			if(vkContext.IsDeviceExtensionEnabled(ext) == false) {
				Con::cerr << "[VR] ERROR: Required device extension '" << ext << "' is not enabled!" << Con::endl;
				break;
			}
		}
	}

	guard.dismiss();
	auto instance = std::unique_ptr<Instance> {new Instance {ivr, eRenderAPI, pRenderModels, pCompositor}};
	return instance;
}

std::string Instance::GetTrackingSystemName(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_TrackingSystemName_String, peError); }
std::string Instance::GetModelNumber(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_ModelNumber_String, peError); }
std::string Instance::GetSerialNumber(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_SerialNumber_String, peError); }
std::string Instance::GetRenderModelName(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_RenderModelName_String, peError); }
std::string Instance::GetManufacturerName(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_ManufacturerName_String, peError); }
std::string Instance::GetTrackingFirmwareVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_TrackingFirmwareVersion_String, peError); }
std::string Instance::GetHardwareRevision(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_HardwareRevision_String, peError); }
std::string Instance::GetAllWirelessDongleDescriptions(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_AllWirelessDongleDescriptions_String, peError); }
std::string Instance::GetConnectedWirelessDongle(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_ConnectedWirelessDongle_String, peError); }
std::string Instance::GetFirmwareManualUpdateURL(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_Firmware_ManualUpdateURL_String, peError); }
std::string Instance::GetFirmwareProgrammingTarget(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_Firmware_ProgrammingTarget_String, peError); }
std::string Instance::GetDisplayMCImageLeft(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_DisplayMCImageLeft_String, peError); }
std::string Instance::GetDisplayMCImageRight(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_DisplayMCImageRight_String, peError); }
std::string Instance::GetDisplayGCImage(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_DisplayGCImage_String, peError); }
std::string Instance::GetCameraFirmwareDescription(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_CameraFirmwareDescription_String, peError); }
std::string Instance::GetAttachedDeviceId(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_AttachedDeviceId_String, peError); }
std::string Instance::GetModelLabel(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceString(vr::TrackedDeviceProperty::Prop_ModeLabel_String, peError); }

bool Instance::WillDriftInYaw(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_WillDriftInYaw_Bool, peError); }
bool Instance::DeviceIsWireless(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_DeviceIsWireless_Bool, peError); }
bool Instance::DeviceIsCharging(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_DeviceIsCharging_Bool, peError); }
bool Instance::FirmwareUpdateAvailable(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_Firmware_UpdateAvailable_Bool, peError); }
bool Instance::FirmwareManualUpdate(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_Firmware_ManualUpdate_Bool, peError); }
bool Instance::BlockServerShutdown(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_BlockServerShutdown_Bool, peError); }
bool Instance::CanUnifyCoordinateSystemWithHmd(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_CanUnifyCoordinateSystemWithHmd_Bool, peError); }
bool Instance::ContainsProximitySensor(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_ContainsProximitySensor_Bool, peError); }
bool Instance::DeviceProvidesBatteryStatus(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_DeviceProvidesBatteryStatus_Bool, peError); }
bool Instance::DeviceCanPowerOff(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_DeviceCanPowerOff_Bool, peError); }
bool Instance::HasCamera(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_HasCamera_Bool, peError); }
bool Instance::ReportsTimeSinceVSync(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_ReportsTimeSinceVSync_Bool, peError); }
bool Instance::IsOnDesktop(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceBool(vr::TrackedDeviceProperty::Prop_IsOnDesktop_Bool, peError); }

float Instance::GetDeviceBatteryPercentage(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_DeviceBatteryPercentage_Float, peError); }
float Instance::GetSecondsFromVsyncToPhotons(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_SecondsFromVsyncToPhotons_Float, peError); }
float Instance::GetDisplayFrequency(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_DisplayFrequency_Float, peError); }
float Instance::GetUserIpdMeters(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_UserIpdMeters_Float, peError); }
float Instance::GetDisplayMCOffset(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_DisplayMCOffset_Float, peError); }
float Instance::GetDisplayMCScale(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_DisplayMCScale_Float, peError); }
float Instance::GetDisplayGCBlackClamp(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_DisplayGCBlackClamp_Float, peError); }
float Instance::GetDisplayGCOffset(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_DisplayGCOffset_Float, peError); }
float Instance::GetDisplayGCScale(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_DisplayGCScale_Float, peError); }
float Instance::GetDisplayGCPrescale(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_DisplayGCPrescale_Float, peError); }
float Instance::GetLensCenterLeftU(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_LensCenterLeftU_Float, peError); }
float Instance::GetLensCenterLeftV(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_LensCenterLeftV_Float, peError); }
glm::vec2 Instance::GetLensCenterLeftUV(vr::TrackedPropertyError *peError) const
{
	glm::vec2 uv {};
	uv.x = GetLensCenterLeftU(peError);
	if(peError != nullptr && *peError != vr::TrackedPropertyError::TrackedProp_Success)
		return uv;
	uv.y = GetLensCenterLeftV(peError);
	return uv;
}
float Instance::GetLensCenterRightU(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_LensCenterRightU_Float, peError); }
float Instance::GetLensCenterRightV(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_LensCenterRightV_Float, peError); }
glm::vec2 Instance::GetLensCenterRightUV(vr::TrackedPropertyError *peError) const
{
	glm::vec2 uv {};
	uv.x = GetLensCenterRightU(peError);
	if(peError != nullptr && *peError != vr::TrackedPropertyError::TrackedProp_Success)
		return uv;
	uv.y = GetLensCenterRightV(peError);
	return uv;
}
float Instance::GetUserHeadToEyeDepthMeters(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_UserHeadToEyeDepthMeters_Float, peError); }
float Instance::GetFieldOfViewLeftDegrees(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_FieldOfViewLeftDegrees_Float, peError); }
float Instance::GetFieldOfViewRightDegrees(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_FieldOfViewRightDegrees_Float, peError); }
float Instance::GetFieldOfViewTopDegrees(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_FieldOfViewTopDegrees_Float, peError); }
float Instance::GetFieldOfViewBottomDegrees(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_FieldOfViewBottomDegrees_Float, peError); }
float Instance::GetTrackingRangeMinimumMeters(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_TrackingRangeMinimumMeters_Float, peError); }
float Instance::GetTrackingRangeMaximumMeters(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceFloat(vr::TrackedDeviceProperty::Prop_TrackingRangeMaximumMeters_Float, peError); }

glm::mat3x4 Instance::GetStatusDisplayTransform(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceMatrix34(vr::TrackedDeviceProperty::Prop_StatusDisplayTransform_Matrix34, peError); }
glm::mat3x4 Instance::GetCameraToHeadTransform(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceMatrix34(vr::TrackedDeviceProperty::Prop_CameraToHeadTransform_Matrix34, peError); }

uint64_t Instance::GetHardwareRevisionNumber(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_HardwareRevision_Uint64, peError); }
uint64_t Instance::GetFirmwareVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_FirmwareVersion_Uint64, peError); }
uint64_t Instance::GetFPGAVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_FPGAVersion_Uint64, peError); }
uint64_t Instance::GetVRCVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_VRCVersion_Uint64, peError); }
uint64_t Instance::GetRadioVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_RadioVersion_Uint64, peError); }
uint64_t Instance::GetDongleVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_DongleVersion_Uint64, peError); }
uint64_t Instance::GetCurrentUniverseId(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_CurrentUniverseId_Uint64, peError); }
uint64_t Instance::GetPreviousUniverseId(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_PreviousUniverseId_Uint64, peError); }
uint64_t Instance::GetDisplayFirmwareVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_DisplayFirmwareVersion_Uint64, peError); }
uint64_t Instance::GetCameraFirmwareVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_CameraFirmwareVersion_Uint64, peError); }
uint64_t Instance::GetDisplayFPGAVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_DisplayFPGAVersion_Uint64, peError); }
uint64_t Instance::GetDisplayBootloaderVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_DisplayBootloaderVersion_Uint64, peError); }
uint64_t Instance::GetDisplayHardwareVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_DisplayHardwareVersion_Uint64, peError); }
uint64_t Instance::GetAudioFirmwareVersion(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_AudioFirmwareVersion_Uint64, peError); }
uint64_t Instance::GetSupportedButtons(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceUInt64(vr::TrackedDeviceProperty::Prop_SupportedButtons_Uint64, peError); }

int32_t Instance::GetDeviceClass(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_DeviceClass_Int32, peError); }
int32_t Instance::GetDisplayMCType(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_DisplayMCType_Int32, peError); }
int32_t Instance::GetEdidVendorID(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_EdidVendorID_Int32, peError); }
int32_t Instance::GetEdidProductID(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_EdidProductID_Int32, peError); }
int32_t Instance::GetDisplayGCType(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_DisplayGCType_Int32, peError); }
int32_t Instance::GetCameraCompatibilityMode(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_CameraCompatibilityMode_Int32, peError); }
int32_t Instance::GetAxis0Type(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_Axis0Type_Int32, peError); }
int32_t Instance::GetAxis1Type(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_Axis1Type_Int32, peError); }
int32_t Instance::GetAxis2Type(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_Axis2Type_Int32, peError); }
int32_t Instance::GetAxis3Type(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_Axis3Type_Int32, peError); }
int32_t Instance::GetAxis4Type(vr::TrackedPropertyError *peError) const { return GetTrackedDeviceInt32(vr::TrackedDeviceProperty::Prop_Axis4Type_Int32, peError); }

bool Instance::CanRenderScene() const { return m_compositor->CanRenderScene(); }
void Instance::ClearLastSubmittedFrame() const { m_compositor->ClearLastSubmittedFrame(); }
void Instance::ClearSkyboxOverride() const { m_compositor->ClearSkyboxOverride(); }
vr::ETrackingUniverseOrigin Instance::GetTrackingSpace() const { return m_compositor->GetTrackingSpace(); }
void Instance::SetTrackingSpace(vr::ETrackingUniverseOrigin space) const { m_compositor->SetTrackingSpace(space); }
vr::Compositor_CumulativeStats Instance::GetCumulativeStats() const
{
	vr::Compositor_CumulativeStats stats;
	m_compositor->GetCumulativeStats(&stats, sizeof(stats));
	return stats;
}
vr::EVRCompositorError Instance::SetSkyboxOverride(const std::vector<prosper::IImage *> &images) const
{
	vr::ETextureType texType;
	std::vector<vr::VRVulkanTextureData_t> vkHandles;
	std::vector<void *> glHandles;
	switch(m_renderAPI) {
	case RenderAPI::OpenGL:
		{
			glHandles.reserve(images.size());
			for(auto *img : images) {
				glHandles.push_back({});
				glHandles.back() = const_cast<void *>(img->GetInternalHandle());
			}
			texType = vr::ETextureType::TextureType_OpenGL;
			break;
		}
	case RenderAPI::Vulkan:
		{
			vkHandles.reserve(images.size());
			for(auto *img : images) {
				vkHandles.push_back({});
				auto &vkData = vkHandles.back();
				openvr::initialize_vulkan_texture_data(vkData, *img);
			}
			texType = vr::ETextureType::TextureType_Vulkan;
			break;
		}
	}

	auto renderAPI = c_engine->GetRenderContext().GetAPIIdentifier();
	std::vector<vr::Texture_t> imgTexData(images.size());
	for(auto i = decltype(images.size()) {0}; i < images.size(); ++i) {
		auto &img = images.at(i);
		auto &texData = imgTexData.at(i);
		texData.eColorSpace = vr::EColorSpace::ColorSpace_Auto;
		texData.handle = (texType == vr::ETextureType::TextureType_Vulkan) ? &(vkHandles[i]) : glHandles[i];
		texData.eType = texType;
	}
	return m_compositor->SetSkyboxOverride(imgTexData.data(), imgTexData.size());
}
vr::EVRCompositorError Instance::SetSkyboxOverride(prosper::IImage &img) const { return SetSkyboxOverride({&img}); }
vr::EVRCompositorError Instance::SetSkyboxOverride(prosper::IImage &img, prosper::IImage &img2) const { return SetSkyboxOverride({&img, &img2}); }
vr::EVRCompositorError Instance::SetSkyboxOverride(prosper::IImage &front, prosper::IImage &back, prosper::IImage &left, prosper::IImage &right, prosper::IImage &top, prosper::IImage &bottom) const { return SetSkyboxOverride({&front, &back, &left, &right, &top, &bottom}); }
void Instance::CompositorBringToFront() const { m_compositor->CompositorBringToFront(); }
void Instance::CompositorDumpImages() const { m_compositor->CompositorDumpImages(); }
void Instance::CompositorGoToBack() const { m_compositor->CompositorGoToBack(); }
void Instance::ForceInterleavedReprojectionOn(bool b) const { m_compositor->ForceInterleavedReprojectionOn(b); }
void Instance::ForceReconnectProcess() const { m_compositor->ForceReconnectProcess(); }
float Instance::GetFrameTimeRemaining() const { return m_compositor->GetFrameTimeRemaining(); }
bool Instance::IsFullscreen() const { return m_compositor->IsFullscreen(); }
bool Instance::ShouldAppRenderWithLowResources() const { return m_compositor->ShouldAppRenderWithLowResources(); }
void Instance::SuspendRendering(bool b) const
{
	m_isRenderingSuspended = b;
	m_compositor->SuspendRendering(b);
}
bool Instance::IsRenderingSuspended() const { return m_isRenderingSuspended; }
//#include <chrono>
vr::ETrackedControllerRole Instance::GetTrackedDeviceRole(uint32_t deviceIdx) const
{
	vr::ETrackedPropertyError err;
	auto controllerRole = m_system->GetInt32TrackedDeviceProperty(deviceIdx, vr::ETrackedDeviceProperty::Prop_ControllerRoleHint_Int32, &err);
	if(err != vr::TrackedProp_Success)
		return vr::ETrackedControllerRole::TrackedControllerRole_Invalid;
	return static_cast<vr::ETrackedControllerRole>(controllerRole);
}
vr::EDeviceActivityLevel Instance::GetTrackedDeviceActivityLevel(uint32_t deviceIdx) const { return m_system->GetTrackedDeviceActivityLevel(deviceIdx); }
std::optional<std::string> Instance::GetTrackedDeviceSerialNumber(uint32_t deviceIdx) const
{
	vr::TrackedPropertyError err;
	auto serialNumber = GetTrackedDeviceString(deviceIdx, vr::ETrackedDeviceProperty::Prop_SerialNumber_String, &err);
	if(err != vr::TrackedProp_Success)
		return {};
	return serialNumber;
}
std::optional<std::string> Instance::GetTrackedDeviceType(uint32_t deviceIdx) const
{
	vr::TrackedPropertyError err;
	auto serialNumber = GetTrackedDeviceString(deviceIdx, vr::ETrackedDeviceProperty::Prop_ControllerType_String, &err);
	if(err != vr::TrackedProp_Success)
		return {};
	return serialNumber;
}
std::optional<std::string> Instance::GetTrackedDeviceTrackingSystemName(uint32_t deviceIdx) const
{
	vr::TrackedPropertyError err;
	auto trackingSystem = GetTrackedDeviceString(deviceIdx, vr::ETrackedDeviceProperty::Prop_TrackingSystemName_String, &err);
	if(err != vr::TrackedProp_Success)
		return {};
	return trackingSystem;
}
std::optional<std::string> Instance::GetTrackedDeviceModelNumber(uint32_t deviceIdx) const
{
	vr::TrackedPropertyError err;
	auto modelNumber = GetTrackedDeviceString(deviceIdx, vr::ETrackedDeviceProperty::Prop_ModelNumber_String, &err);
	if(err != vr::TrackedProp_Success)
		return {};
	return modelNumber;
}
std::optional<std::string> Instance::GetTrackedDeviceRenderModelName(uint32_t deviceIdx) const
{
	vr::TrackedPropertyError err;
	auto renderModelName = GetTrackedDeviceString(deviceIdx, vr::ETrackedDeviceProperty::Prop_RenderModelName_String, &err);
	if(err != vr::TrackedProp_Success)
		return {};
	return renderModelName;
}
bool Instance::GetPoseTransform(uint32_t deviceIdx, vr::TrackedDevicePose_t &pose, Mat4 &m) const
{
	float fSecondsSinceLastVsync;
	m_system->GetTimeSinceLastVsync(&fSecondsSinceLastVsync, NULL);

	auto fDisplayFrequency = m_system->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
	auto fFrameDuration = 1.f / fDisplayFrequency;
	auto fVsyncToPhotons = m_system->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SecondsFromVsyncToPhotons_Float);

	auto fPredictedSecondsFromNow = fFrameDuration - fSecondsSinceLastVsync + fVsyncToPhotons;
	static std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> poseTransforms;
	m_system->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseSeated, fPredictedSecondsFromNow, poseTransforms.data(), poseTransforms.size());
	if(deviceIdx >= poseTransforms.size())
		return false;
	pose = poseTransforms.at(deviceIdx);
	if(pose.bDeviceIsConnected == false || pose.bPoseIsValid == false || pose.eTrackingResult != vr::ETrackingResult::TrackingResult_Running_OK)
		return false;
	m = steam_vr_matrix_to_engine_matrix(pose.mDeviceToAbsoluteTracking);
	if(std::isnan(m[0][0]))
		return false;
	// m = glm::inverse(m);
	return true;
}
