#include <prosper_command_buffer.hpp>
#include <prosper_fence.hpp>
#include "stdafx_openvr.h"
#include <optional>
#include <unordered_map>
#include <pragma/lua/classes/ldef_color.h>
#include <sharedutils/datastream.h>
#include <pragma/lua/libraries/c_lua_vulkan.h>
#include <pragma/lua/lua_entity_component.hpp>
#include <pragma/audio/e_alstate.h>
#include <pragma/game/c_game.h>
#include <pragma/physics/physobj.h>
#include <pragma/physics/collision_object.hpp>
#include <pragma/lua/c_lentity_handles.hpp>
#include <pragma/lua/converters/game_type_converters_t.hpp>
#include <pragma/lua/converters/optional_converter_t.hpp>
#include <pragma/lua/converters/pair_converter_t.hpp>
#include <pragma/lua/converters/vector_converter_t.hpp>
#include <pragma/lua/policies/default_parameter_policy.hpp>
#include <pragma/entities/environment/c_env_camera.h>
#include <pragma/entities/components/c_scene_component.hpp>
#include <pragma/c_engine.h>
#include <luainterface.hpp>
#include <sharedutils/functioncallback.h>
#include <sharedutils/util_pragma.hpp>
#include <pragma/pragma_module.hpp>
#include <pragma/util/util_game.hpp>
#include "vr_instance.hpp"
#include "vr_eye.hpp"
#include "lopenvr.h"
#include "wvmodule.h"
#include <luabind/copy_policy.hpp>

import glm;

std::unique_ptr<openvr::Instance> s_vrInstance = nullptr;

namespace luabind {
	// Instance
	namespace detail {
		PRAGMA_EXPORT openvr::Instance *get_openvr_instance(lua_State *l);
		template<typename T>
		T get_openvr_instance(lua_State *l)
		{
			if constexpr(std::is_pointer_v<T>)
				return static_cast<T>(get_openvr_instance(l));
			else {
				auto *instance = get_openvr_instance(l);
				if(!instance)
					Lua::Error(l, "openvr has not been initialized!");
				return *instance;
			}
		}
	};
	template<typename T>
	    requires(is_type_or_derived<T, openvr::Instance>)
	struct default_converter<T> : parameter_emplacement_converter<T, detail::get_openvr_instance<T>> {};

	// System
	namespace detail {
		PRAGMA_EXPORT vr::IVRSystem *get_openvr_system(lua_State *l);
		template<typename T>
		T get_openvr_system(lua_State *l)
		{
			if constexpr(std::is_pointer_v<T>)
				return static_cast<T>(get_openvr_system(l));
			else {
				auto *instance = get_openvr_system(l);
				if(!instance)
					Lua::Error(l, "openvr has not been initialized!");
				return *instance;
			}
		}
	};
	template<typename T>
	    requires(is_type_or_derived<T, vr::IVRSystem>)
	struct default_converter<T> : parameter_emplacement_converter<T, detail::get_openvr_system<T>> {};

	// RenderModels
	namespace detail {
		PRAGMA_EXPORT vr::IVRRenderModels *get_openvr_render_models(lua_State *l);
		template<typename T>
		T get_openvr_render_models(lua_State *l)
		{
			if constexpr(std::is_pointer_v<T>)
				return static_cast<T>(get_openvr_render_models(l));
			else {
				auto *instance = get_openvr_render_models(l);
				if(!instance)
					Lua::Error(l, "openvr has not been initialized!");
				return *instance;
			}
		}
	};
	template<typename T>
	    requires(is_type_or_derived<T, vr::IVRRenderModels>)
	struct default_converter<T> : parameter_emplacement_converter<T, detail::get_openvr_render_models<T>> {};

	// Compositor
	namespace detail {
		PRAGMA_EXPORT vr::IVRCompositor *get_openvr_compositor(lua_State *l);
		template<typename T>
		T get_openvr_compositor(lua_State *l)
		{
			if constexpr(std::is_pointer_v<T>)
				return static_cast<T>(get_openvr_compositor(l));
			else {
				auto *instance = get_openvr_compositor(l);
				if(!instance)
					Lua::Error(l, "openvr has not been initialized!");
				return *instance;
			}
		}
	};
	template<typename T>
	    requires(is_type_or_derived<T, vr::IVRCompositor>)
	struct default_converter<T> : parameter_emplacement_converter<T, detail::get_openvr_compositor<T>> {};

	// Chaperone
	namespace detail {
		PRAGMA_EXPORT vr::IVRChaperone *get_openvr_chaperone(lua_State *l);
		template<typename T>
		T get_openvr_chaperone(lua_State *l)
		{
			if constexpr(std::is_pointer_v<T>)
				return static_cast<T>(get_openvr_chaperone(l));
			else {
				auto *instance = get_openvr_chaperone(l);
				if(!instance)
					Lua::Error(l, "openvr has not been initialized!");
				return *instance;
			}
		}
	};
	template<typename T>
	    requires(is_type_or_derived<T, vr::IVRChaperone>)
	struct default_converter<T> : parameter_emplacement_converter<T, detail::get_openvr_chaperone<T>> {};
}

openvr::Instance *luabind::detail::get_openvr_instance(lua_State *l) { return s_vrInstance.get(); }
vr::IVRSystem *luabind::detail::get_openvr_system(lua_State *l) { return s_vrInstance ? s_vrInstance->GetSystemInterface() : nullptr; }
vr::IVRRenderModels *luabind::detail::get_openvr_render_models(lua_State *l) { return s_vrInstance ? s_vrInstance->GetRenderInterface() : nullptr; }
vr::IVRCompositor *luabind::detail::get_openvr_compositor(lua_State *l) { return s_vrInstance ? s_vrInstance->GetCompositorInterface() : nullptr; }
vr::IVRChaperone *luabind::detail::get_openvr_chaperone(lua_State *l) { return s_vrInstance ? s_vrInstance->GetChaperone() : nullptr; }

static openvr::Eye &get_eye(vr::EVREye eyeId)
{
	switch(eyeId) {
	case vr::EVREye::Eye_Left:
		return s_vrInstance->GetLeftEye();
	default:
		return s_vrInstance->GetRightEye();
	}
}

extern "C" {
PRAGMA_EXPORT void openvr_set_hmd_view_enabled(bool b)
{
	if(s_vrInstance == nullptr)
		return;
	s_vrInstance->SetHmdViewEnabled(b);
}
PRAGMA_EXPORT void openvr_set_controller_state_callback(const std::function<void(uint32_t, uint32_t, GLFW::KeyState)> &f)
{
	if(s_vrInstance == nullptr)
		return;
	s_vrInstance->SetControllerStateCallback(f);
}
PRAGMA_EXPORT void openvr_set_mirror_window_enabled(bool b)
{
	if(s_vrInstance == nullptr)
		return;
	if(b == true)
		s_vrInstance->ShowMirrorWindow();
	else
		s_vrInstance->HideMirrorWindow();
}

PRAGMA_EXPORT bool openvr_initialize(std::string &strErr, std::vector<std::string> &reqInstanceExtensions, std::vector<std::string> &reqDeviceExtensions)
{
	if(s_vrInstance != nullptr)
		return true;
	vr::EVRInitError err;
#if LOPENVR_VERBOSE == 1
	std::cout << "[VR] Creating vr instance..." << std::endl;
#endif
	s_vrInstance = openvr::Instance::Create(&err, reqInstanceExtensions, reqDeviceExtensions);
	strErr = openvr::to_string(err);
	if(s_vrInstance == nullptr)
		return false;
#if LOPENVR_VERBOSE == 1
	std::cout << "[VR] Initializing vr instance..." << std::endl;
#endif
	auto r = (err == vr::EVRInitError::VRInitError_None) ? true : false;
	if(r == true)
		s_vrInstance->HideMirrorWindow();
#if LOPENVR_VERBOSE == 1
	std::cout << "[VR] Initialization complete!" << std::endl;
#endif
	return r;
}
};

int run_openxr_demo(int argc, char *argv[]);
#include <prosper_window.hpp>
GLFW::Window *get_glfw_window() { return &*pragma::get_cengine()->GetRenderContext().GetWindow(); }

//#include "openxr/pvr_openxr_instance.hpp"

namespace Lua {
	namespace openvr {
		std::string property_error_to_string(vr::ETrackedPropertyError err) { return ::openvr::to_string(err); }

		std::string init_error_to_string(vr::EVRInitError err) { return ::openvr::to_string(err); }

		std::string compositor_error_to_string(vr::VRCompositorError err) { return ::openvr::to_string(err); }

		std::optional<std::string> button_id_to_string(vr::EVRButtonId buttonId)
		{
			if(s_vrInstance == nullptr)
				return {};
			auto *sys = s_vrInstance->GetSystemInterface();
			return sys->GetButtonIdNameFromEnum(buttonId);
		}

		std::optional<std::string> event_type_to_string(vr::EVREventType evType)
		{
			if(s_vrInstance == nullptr)
				return {};
			auto *sys = s_vrInstance->GetSystemInterface();
			return ::openvr::to_string(evType);
		}

		std::optional<std::string> controller_axis_type_to_string(vr::EVRControllerAxisType controllerAxisType)
		{
			if(s_vrInstance == nullptr)
				return {};
			auto *sys = s_vrInstance->GetSystemInterface();
			return sys->GetControllerAxisTypeNameFromEnum(controllerAxisType);
		}
	};
};

template<class T>
using TProperty = std::variant<vr::ETrackedPropertyError, std::pair<vr::ETrackedPropertyError, T>>;

// Return type of getter member function pointer
template<auto T>
using TPropertyBaseType = std::invoke_result_t<decltype(T), openvr::Instance const *, vr::TrackedPropertyError *>;

template<auto T> // T = Member function pointer to property getter-function (e.g. &openvr::Instance::GetTrackingSystemName)
TProperty<TPropertyBaseType<T>> get_property(openvr::Instance *instance)
{
	if(!instance)
		return vr::ETrackedPropertyError::TrackedProp_InvalidDevice;
	vr::TrackedPropertyError err;
	auto val = std::invoke(T, instance, &err);
	return std::pair<vr::ETrackedPropertyError, TPropertyBaseType<T>> {err, std::move(val)};
}

namespace Lua::openvr {
	Lua::map<std::string, uint32_t> get_cumulative_stats(lua_State *l, ::openvr::Instance &instance)
	{
		auto stats = s_vrInstance->GetCumulativeStats();
		auto t = luabind::newtable(l);

		const auto fAddAttribute = [l, &t, &stats](const std::string id, uint32_t val) { t[id] = val; };
		fAddAttribute("numFramePresents", stats.m_nNumFramePresents);
		fAddAttribute("numDroppedFrames", stats.m_nNumDroppedFrames);
		fAddAttribute("numReprojectedFrames", stats.m_nNumReprojectedFrames);
		fAddAttribute("numFramePresentsOnStartup", stats.m_nNumFramePresentsOnStartup);
		fAddAttribute("numDroppedFramesOnStartup", stats.m_nNumDroppedFramesOnStartup);
		fAddAttribute("numReprojectedFramesOnStartup", stats.m_nNumReprojectedFramesOnStartup);
		fAddAttribute("numLoading", stats.m_nNumLoading);
		fAddAttribute("numFramePresentsLoading", stats.m_nNumFramePresentsLoading);
		fAddAttribute("numDroppedFramesLoading", stats.m_nNumDroppedFramesLoading);
		fAddAttribute("numReprojectedFramesLoading", stats.m_nNumReprojectedFramesLoading);
		fAddAttribute("numTimedOut", stats.m_nNumTimedOut);
		fAddAttribute("numFramePresentsTimedOut", stats.m_nNumFramePresentsTimedOut);
		fAddAttribute("numDroppedFramesTimedOut", stats.m_nNumDroppedFramesTimedOut);
		fAddAttribute("numReprojectedFramesTimedOut", stats.m_nNumReprojectedFramesTimedOut);
		return t;
	}

	std::pair<uint32_t, uint32_t> get_recommended_render_target_size(vr::IVRSystem &sys)
	{
		auto width = 0u;
		auto height = 0u;
		sys.GetRecommendedRenderTargetSize(&width, &height);
		return {width, height};
	}

	Mat4 get_projection_matrix(vr::IVRSystem &sys, vr::EVREye eye, float nearZ, float farZ)
	{
		auto vrMat = sys.GetProjectionMatrix(eye, nearZ, farZ);
		auto m = glm::transpose(reinterpret_cast<Mat4 &>(vrMat.m));
		return glm::gtx::scale(m, Vector3(1.f, -1.f, 1.f));
	}

	std::tuple<float, float, float, float> get_projection_raw(vr::IVRSystem &sys, vr::EVREye eye)
	{
		auto left = 0.f;
		auto right = 0.f;
		auto top = 0.f;
		auto bottom = 0.f;
		sys.GetProjectionRaw(eye, &left, &right, &top, &bottom);
		return {left, right, top, bottom};
	}

	std::variant<bool, std::tuple<bool, Vector2, Vector2, Vector2>> compute_distortion(vr::IVRSystem &sys, vr::EVREye eye, float fu, float fv)
	{
		vr::DistortionCoordinates_t distortion {};
		auto b = sys.ComputeDistortion(static_cast<vr::EVREye>(eye), fu, fv, &distortion);
		if(b == true) {
			Vector2 vred {distortion.rfRed[0], distortion.rfRed[1]};
			Vector2 vgreen {distortion.rfGreen[0], distortion.rfGreen[1]};
			Vector2 vblue {distortion.rfBlue[0], distortion.rfBlue[1]};
			return std::tuple<bool, Vector2, Vector2, Vector2> {true, vred, vgreen, vblue};
		}
		return std::variant<bool, std::tuple<bool, Vector2, Vector2, Vector2>> {true};
	}

	Mat4 get_eye_to_head_transform(vr::IVRSystem &sys, vr::EVREye eEye, pragma::CCameraComponent &cam)
	{
		auto &eye = (eEye == vr::EVREye::Eye_Left) ? s_vrInstance->GetLeftEye() : s_vrInstance->GetRightEye();
		return eye.GetEyeViewMatrix(cam);
	}

	std::tuple<bool, float, uint64_t> get_time_since_last_vsync(vr::IVRSystem &sys)
	{
		float secondsSinceLasyVsync = 0.f;
		uint64_t pullFrameCounter = 0ull;
		auto r = sys.GetTimeSinceLastVsync(&secondsSinceLasyVsync, &pullFrameCounter);
		return std::tuple<bool, float, uint64_t> {r, secondsSinceLasyVsync, pullFrameCounter};
	}

	std::vector<vr::TrackedDevicePose_t> get_device_to_absolute_tracking_pose(lua_State *l, vr::IVRSystem &sys, vr::ETrackingUniverseOrigin origin, float predictedSecondsToPhotonsFromNow)
	{
		static std::vector<vr::TrackedDevicePose_t> trackedDevicePoseArray(vr::k_unMaxTrackedDeviceCount);
		sys.GetDeviceToAbsoluteTrackingPose(origin, predictedSecondsToPhotonsFromNow, trackedDevicePoseArray.data(), trackedDevicePoseArray.size());
		return trackedDevicePoseArray;
	}

	float compute_seconds_to_photons(vr::IVRSystem &sys)
	{
		float fSecondsSinceLastVsync;
		sys.GetTimeSinceLastVsync(&fSecondsSinceLastVsync, nullptr);

		auto fDisplayFrequency = sys.GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
		auto fFrameDuration = 1.f / fDisplayFrequency;
		auto fVsyncToPhotons = sys.GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SecondsFromVsyncToPhotons_Float);

		auto fPredictedSecondsFromNow = fFrameDuration - fSecondsSinceLastVsync + fVsyncToPhotons;
		return fPredictedSecondsFromNow;
	}

	Mat3x4 get_seated_zero_pose_to_standing_absolute_tracking_pose(vr::IVRSystem &sys)
	{
		auto m = sys.GetSeatedZeroPoseToStandingAbsoluteTrackingPose();
		return reinterpret_cast<Mat3x4 &>(m);
	}

	void trigger_haptic_pulse(vr::IVRSystem &sys, vr::TrackedDeviceIndex_t devIndex, uint32_t axisId, float duration) { sys.TriggerHapticPulse(devIndex, axisId, duration * 1'000'000); }

	struct LuaVRControllerState {
		// If packet num matches that on your prior call, then the controller state hasn't been changed since
		// your last call and there is no need to process it
		uint32_t unPacketNum;

		// bit flags for each of the buttons. Use ButtonMaskFromId to turn an ID into a mask
		uint32_t ulButtonPressed;
		uint32_t ulButtonTouched;

		// Axis data for the controller's analog inputs
		Vector2 rAxis0;
		Vector2 rAxis1;
		Vector2 rAxis2;
		Vector2 rAxis3;
		Vector2 rAxis4;
	};

	static LuaVRControllerState vr_controller_state_to_lua_controller_state(const vr::VRControllerState_t &in)
	{
		auto r = LuaVRControllerState {};
		r.unPacketNum = in.unPacketNum;
		r.ulButtonPressed = in.ulButtonPressed;
		r.ulButtonTouched = in.ulButtonTouched;
		r.rAxis0 = {in.rAxis[0].x, in.rAxis[0].y};
		r.rAxis1 = {in.rAxis[1].x, in.rAxis[1].y};
		r.rAxis2 = {in.rAxis[2].x, in.rAxis[2].y};
		r.rAxis3 = {in.rAxis[3].x, in.rAxis[3].y};
		r.rAxis4 = {in.rAxis[4].x, in.rAxis[4].y};
		return r;
	}

	Lua::tb<LuaVRControllerState> get_controller_states(lua_State *l, vr::IVRSystem &sys)
	{
		auto t = luabind::newtable(l);

		vr::VRControllerState_t state {};
		for(auto i = decltype(vr::k_unMaxTrackedDeviceCount) {0}; i < vr::k_unMaxTrackedDeviceCount; ++i) {
			vr::VRControllerState_t state;
			if(sys.GetControllerState(i, &state, sizeof(vr::VRControllerState_t)))
				t[i] = vr_controller_state_to_lua_controller_state(state);
		}
		return t;
	}

	std::optional<LuaVRControllerState> get_controller_state(vr::IVRSystem &sys, vr::TrackedDeviceIndex_t devIndex)
	{
		vr::VRControllerState_t state {};
		auto r = sys.GetControllerState(devIndex, &state, sizeof(vr::VRControllerState_t));
		if(r == true)
			return vr_controller_state_to_lua_controller_state(state);
		return {};
	}

	std::optional<std::pair<LuaVRControllerState, vr::TrackedDevicePose_t>> get_controller_state_with_pose(vr::IVRSystem &sys, vr::TrackingUniverseOrigin origin, vr::TrackedDeviceIndex_t devIndex)
	{
		vr::VRControllerState_t state {};
		vr::TrackedDevicePose_t devPose {};
		auto r = sys.GetControllerStateWithPose(origin, devIndex, &state, sizeof(vr::VRControllerState_t), &devPose);
		if(r == true)
			return std::pair<LuaVRControllerState, vr::TrackedDevicePose_t> {vr_controller_state_to_lua_controller_state(state), devPose};
		return {};
	}

	std::optional<std::pair<Mat4, Vector3>> get_pose_transform(::openvr::Instance &instance, uint32_t deviceIdx)
	{
		vr::TrackedDevicePose_t pose {};
		Mat4 m {};
		if(instance.GetPoseTransform(deviceIdx, pose, m) == false)
			return {};
		return std::pair<Mat4, Vector3> {m, Vector3(pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2]) * static_cast<float>(::util::pragma::metres_to_units(1.f))};
	}

	static umath::Transform openvr_matrix_to_pragma_pose(const Mat4 &poseMatrix)
	{
		Vector3 scale;
		Vector3 skew;
		::Vector4 perspective;
		Vector3 pos;
		Quat rot;
		glm::gtx::decompose(poseMatrix, scale, rot, pos, skew, perspective);
		rot = glm::gtx::conjugate(rot);

		static auto openVrToPragmaPoseTransform = uquat::create(EulerAngles(0.f, 180.f, 0.f));
		rot = rot * openVrToPragmaPoseTransform;
		pos *= static_cast<float>(::util::pragma::metres_to_units(1.f));
		return umath::Transform {pos, rot};
	}

	std::pair<umath::Transform, Vector3> get_raw_pose(::openvr::Instance &instance, uint32_t deviceIdx)
	{
		vr::TrackedDevicePose_t pose {};
		Mat4 m {};
		//if(s_vrInstance->GetPoseTransform(deviceIdx,pose,m) == false)
		//	return 0;
		m = instance.GetPoseMatrix(deviceIdx);

		m = glm::inverse(m);
		auto mpose = openvr_matrix_to_pragma_pose(m);

		// For some reason the position from GetPoseMatrix (which comes from WaitGetPoses)
		// is incorrect, but the rotation is correct, while for GetPoseTransform it's the other way around.
		// We're probably doing something wrong somewhere, but for now this will do as a work-around.
		// TODO: FIXME
		vr::TrackedDevicePose_t pose2 {};
		Mat4 m2 {};
		if(s_vrInstance->GetPoseTransform(deviceIdx, pose2, m2)) {
			auto &pos = mpose.GetOrigin();
			pos = {m2[3][0], m2[3][1], m2[3][2]};
			pos *= static_cast<float>(::util::pragma::metres_to_units(1.f));
		}

		static auto applyRotCorrection = true;
		if(applyRotCorrection) {
			static auto correctionRotation = uquat::create(EulerAngles {0.f, 180.f, 0.f});
			mpose.RotateGlobal(correctionRotation);
		}

		return std::pair<umath::Transform, Vector3> {mpose, Vector3(pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2]) * static_cast<float>(::util::pragma::metres_to_units(1.f))};
	}

	std::pair<umath::Transform, Vector3> get_pose(::openvr::Instance &instance, uint32_t deviceIdx)
	{
		auto [pose, vel] = get_raw_pose(instance, deviceIdx);

		auto *invPose = instance.GetInverseDeviceZeroPose(deviceIdx);
		if(invPose)
			pose = *invPose * pose;

		return {pose, vel};
	}
};

static void add_event_data(const vr::VREvent_t &ev, luabind::object &t)
{
	switch(ev.eventType) {
	case vr::VREvent_ButtonPress:
	case vr::VREvent_ButtonUnpress:
	case vr::VREvent_ButtonTouch:
	case vr::VREvent_ButtonUntouch:
		{
			t["button"] = ev.data.controller.button;
			break;
		}
	case vr::VREvent_MouseMove:
	case vr::VREvent_MouseButtonDown:
	case vr::VREvent_MouseButtonUp:
	case vr::VREvent_TouchPadMove:
		{
			t["button"] = ev.data.mouse.button;
			t["x"] = ev.data.mouse.x;
			t["y"] = ev.data.mouse.y;
			break;
		}
	case vr::VREvent_InputFocusCaptured:
	case vr::VREvent_InputFocusReleased:
	case vr::VREvent_SceneApplicationChanged:
	case vr::VREvent_InputFocusChanged:
	case vr::VREvent_SceneApplicationUsingWrongGraphicsAdapter:
	case vr::VREvent_ActionBindingReloaded:
	case vr::VREvent_Quit:
	case vr::VREvent_ProcessQuit:
	case vr::VREvent_QuitAcknowledged:
	case vr::VREvent_Monitor_ShowHeadsetView:
	case vr::VREvent_Monitor_HideHeadsetView:
		{
			t["connectionLost"] = ev.data.process.bConnectionLost;
			t["forced"] = ev.data.process.bForced;
			t["oldPid"] = ev.data.process.oldPid;
			t["pid"] = ev.data.process.pid;
			break;
		}
	case vr::VREvent_FocusEnter:
	case vr::VREvent_FocusLeave:
	case vr::VREvent_OverlayFocusChanged:
	case vr::VREvent_DashboardRequested:
		{
			//auto &overlay = ev.data.overlay
			break;
		}
	case vr::VREvent_ScrollDiscrete:
	case vr::VREvent_ScrollSmooth:
		{
			t["viewportScale"] = ev.data.scroll.viewportscale;
			t["xdelta"] = ev.data.scroll.xdelta;
			t["ydelta"] = ev.data.scroll.ydelta;
			break;
		}
	case vr::VREvent_ShowUI:
		{
			t["type"] = ev.data.showUi.eType;
			break;
		}
	case vr::VREvent_ShowDevTools:
		{
			t["browserIdentifier"] = ev.data.showDevTools.nBrowserIdentifier;
			break;
		}
	case vr::VREvent_Compositor_HDCPError:
		{
			t["code"] = ev.data.hdcpError.eCode;
			break;
		}
	case vr::VREvent_Input_HapticVibration:
		{
			t["amplitude"] = ev.data.hapticVibration.fAmplitude;
			t["durationSeconds"] = ev.data.hapticVibration.fDurationSeconds;
			t["frequency"] = ev.data.hapticVibration.fFrequency;
			break;
		}
	case vr::VREvent_Input_BindingLoadFailed:
	case vr::VREvent_Input_BindingLoadSuccessful:
		{
			t["pathControllerType"] = ev.data.inputBinding.pathControllerType;
			break;
		}
	case vr::VREvent_Input_ActionManifestLoadFailed:
		{
			//auto &actionManifest = ev.data.actionManifest
			break;
		}
	case vr::VREvent_Input_ProgressUpdate:
		{
			t["progress"] = ev.data.progressUpdate.fProgress;
			t["pathProgressAction"] = ev.data.progressUpdate.pathProgressAction;
			break;
		}
	case vr::VREvent_SpatialAnchors_PoseUpdated:
		{
			//auto &spatialAnchor = ev.data.spatialAnchor
			break;
		}
	}
}

void Lua::openvr::close() { s_vrInstance = nullptr; }

void Lua::openvr::register_lua_library(Lua::Interface &l)
{
	auto *lua = l.GetState();

	auto &modVr = l.RegisterLibrary("openvr");
	modVr[luabind::def(
	  "initialize", +[]() -> vr::EVRInitError {
		  //static auto instance = pvr::XrInstance::Create();
		  //for(;;)
		  //	instance->Render();
		  //return 0;
		  vr::EVRInitError err;
		  if(s_vrInstance != nullptr)
			  err = vr::EVRInitError::VRInitError_None;
		  else {
			  std::vector<std::string> reqInstanceExtensions;
			  std::vector<std::string> reqDeviceExtensions;
			  s_vrInstance = ::openvr::Instance::Create(&err, reqInstanceExtensions, reqDeviceExtensions);
		  }
		  return err;
	  })];
	modVr[luabind::def("close", &Lua::openvr::close)];
	modVr[luabind::def("preinitialize", &::openvr::preinitialize_openvr)];
	modVr[luabind::def("is_hmd_present", &::openvr::is_hmd_present)];
	modVr[luabind::def("get_tracked_device_serial_number", &::openvr::Instance::GetTrackedDeviceSerialNumber)];
	modVr[luabind::def("get_tracked_device_activity_level", &::openvr::Instance::GetTrackedDeviceActivityLevel)];
	modVr[luabind::def("get_tracked_device_type", &::openvr::Instance::GetTrackedDeviceType)];
	modVr[luabind::def("update_poses", &::openvr::Instance::UpdateHMDPoses)];
	modVr[luabind::def("get_hmd_pose_matrix", &::openvr::Instance::GetHMDPoseMatrix, luabind::copy_policy<0> {})];
	modVr[luabind::def(
	  "get_hmd_pose", +[]() -> umath::Transform {
		  auto &hmdPoseMatrix = s_vrInstance->GetHMDPoseMatrix();
		  return openvr_matrix_to_pragma_pose(hmdPoseMatrix);
	  })];
	modVr[luabind::def("get_eye", &get_eye)];
	modVr[luabind::def(
	  "submit_eye", +[](::openvr::Instance &instance, vr::EVREye eyeId) -> vr::EVRCompositorError {
		  auto &eye = get_eye(eyeId);
		  return instance.GetCompositorInterface()->Submit(eye.GetVREye(), &eye.GetVRTexture());
	  })];
	modVr[luabind::def(
	  "set_eye_image", +[](::openvr::Instance &instance, vr::EVREye eyeIndex, prosper::IImage &img) {
		  auto &eye = (eyeIndex == vr::EVREye::Eye_Left) ? s_vrInstance->GetLeftEye() : s_vrInstance->GetRightEye();
		  eye.SetImage(img);
	  })];
	modVr[luabind::def(
	  "poll_events", +[](lua_State *l, ::openvr::Instance &instance) -> Lua::map<std::string, luabind::object> {
		  auto t = luabind::newtable(l);
		  int32_t idx = 1;
		  for(auto &ev : s_vrInstance->PollEvents()) {
			  auto tEv = luabind::newtable(l);
			  tEv["type"] = ev.eventType;
			  auto data = luabind::newtable(l);
			  tEv["data"] = data;
			  tEv["trackedDeviceIndex"] = ev.trackedDeviceIndex;
			  add_event_data(ev, data);
			  t[idx++] = tEv;
		  }
		  return t;
	  })];
	modVr[luabind::def(
	  "is_instance_valid", +[](::openvr::Instance *instance) { return instance != nullptr; })];
	/*modVr[luabind::def(
	  "run_openxr_demo", +[](::openvr::Instance *&instance) {
		  std::vector<char *> args = {"", "-g", "Vulkan2", "-ff", "Hmd", "-vc", "Stereo", "-v"};
		  run_openxr_demo(args.size(), args.data());
		  return 0;
	  })];*/

	modVr[luabind::def("fade_to_color", &::openvr::Instance::FadeToColor), luabind::def("fade_to_color", &::openvr::Instance::FadeToColor, luabind::default_parameter_policy<4, false> {})];

	modVr[luabind::def("get_tracking_system_name", &get_property<&::openvr::Instance::GetTrackingSystemName>)];
	modVr[luabind::def("get_model_number", &get_property<&::openvr::Instance::GetModelNumber>)];
	modVr[luabind::def("get_serial_number", &get_property<&::openvr::Instance::GetSerialNumber>)];
	modVr[luabind::def("get_render_model_name", &get_property<&::openvr::Instance::GetRenderModelName>)];
	modVr[luabind::def("get_manufacturer_name", &get_property<&::openvr::Instance::GetManufacturerName>)];
	modVr[luabind::def("get_tracking_firmware_version", &get_property<&::openvr::Instance::GetTrackingFirmwareVersion>)];
	modVr[luabind::def("get_hardware_revision", &get_property<&::openvr::Instance::GetHardwareRevision>)];
	modVr[luabind::def("get_all_wireless_dongle_descriptions", &get_property<&::openvr::Instance::GetAllWirelessDongleDescriptions>)];
	modVr[luabind::def("get_connected_wireless_dongle", &get_property<&::openvr::Instance::GetConnectedWirelessDongle>)];
	modVr[luabind::def("get_firmware_manual_update_url", &get_property<&::openvr::Instance::GetFirmwareManualUpdateURL>)];
	modVr[luabind::def("get_firmware_programming_target", &get_property<&::openvr::Instance::GetFirmwareProgrammingTarget>)];
	modVr[luabind::def("get_display_mc_image_left", &get_property<&::openvr::Instance::GetDisplayMCImageLeft>)];
	modVr[luabind::def("get_display_mc_image_right", &get_property<&::openvr::Instance::GetDisplayMCImageRight>)];
	modVr[luabind::def("get_display_gc_image", &get_property<&::openvr::Instance::GetDisplayGCImage>)];
	modVr[luabind::def("get_camera_firmware_description", &get_property<&::openvr::Instance::GetCameraFirmwareDescription>)];
	modVr[luabind::def("get_attached_device_id", &get_property<&::openvr::Instance::GetAttachedDeviceId>)];
	modVr[luabind::def("get_model_label", &get_property<&::openvr::Instance::GetModelLabel>)];
	modVr[luabind::def("will_drift_in_yaw", &get_property<&::openvr::Instance::WillDriftInYaw>)];
	modVr[luabind::def("device_is_wireless", &get_property<&::openvr::Instance::DeviceIsWireless>)];
	modVr[luabind::def("device_is_charging", &get_property<&::openvr::Instance::DeviceIsCharging>)];
	modVr[luabind::def("firmware_update_available", &get_property<&::openvr::Instance::FirmwareUpdateAvailable>)];
	modVr[luabind::def("firmware_manual_update", &get_property<&::openvr::Instance::FirmwareManualUpdate>)];
	modVr[luabind::def("block_server_shutdown", &get_property<&::openvr::Instance::BlockServerShutdown>)];
	modVr[luabind::def("can_unify_coordinate_system_with_hmd", &get_property<&::openvr::Instance::CanUnifyCoordinateSystemWithHmd>)];
	modVr[luabind::def("contains_proximity_sensor", &get_property<&::openvr::Instance::ContainsProximitySensor>)];
	modVr[luabind::def("device_provides_battery_status", &get_property<&::openvr::Instance::DeviceProvidesBatteryStatus>)];
	modVr[luabind::def("reports_time_since_vsync", &get_property<&::openvr::Instance::ReportsTimeSinceVSync>)];
	modVr[luabind::def("device_can_power_off", &get_property<&::openvr::Instance::DeviceCanPowerOff>)];
	modVr[luabind::def("has_camera", &get_property<&::openvr::Instance::HasCamera>)];
	modVr[luabind::def("is_on_desktop", &get_property<&::openvr::Instance::IsOnDesktop>)];
	modVr[luabind::def("get_device_battery_percentage", &get_property<&::openvr::Instance::GetDeviceBatteryPercentage>)];
	modVr[luabind::def("get_seconds_from_vsync_to_photons", &get_property<&::openvr::Instance::GetSecondsFromVsyncToPhotons>)];
	modVr[luabind::def("get_display_frequency", &get_property<&::openvr::Instance::GetDisplayFrequency>)];
	modVr[luabind::def("get_user_ipd_meters", &get_property<&::openvr::Instance::GetUserIpdMeters>)];
	modVr[luabind::def("get_display_mc_offset", &get_property<&::openvr::Instance::GetDisplayMCOffset>)];
	modVr[luabind::def("get_display_mc_scale", &get_property<&::openvr::Instance::GetDisplayMCScale>)];
	modVr[luabind::def("get_display_gc_black_clamp", &get_property<&::openvr::Instance::GetDisplayGCBlackClamp>)];
	modVr[luabind::def("get_display_gc_offset", &get_property<&::openvr::Instance::GetDisplayGCOffset>)];
	modVr[luabind::def("get_display_gc_scale", &get_property<&::openvr::Instance::GetDisplayGCScale>)];
	modVr[luabind::def("get_display_gc_prescale", &get_property<&::openvr::Instance::GetDisplayGCPrescale>)];
	modVr[luabind::def("get_lens_center_left_u", &get_property<&::openvr::Instance::GetLensCenterLeftU>)];
	modVr[luabind::def("get_lens_center_left_v", &get_property<&::openvr::Instance::GetLensCenterLeftV>)];
	modVr[luabind::def("get_lens_center_left_uv", &get_property<&::openvr::Instance::GetLensCenterLeftUV>)];
	modVr[luabind::def("get_lens_center_right_u", &get_property<&::openvr::Instance::GetLensCenterRightU>)];
	modVr[luabind::def("get_lens_center_right_v", &get_property<&::openvr::Instance::GetLensCenterRightV>)];
	modVr[luabind::def("get_lens_center_right_uv", &get_property<&::openvr::Instance::GetLensCenterRightUV>)];
	modVr[luabind::def("get_user_head_to_eye_depth_meters", &get_property<&::openvr::Instance::GetUserHeadToEyeDepthMeters>)];
	modVr[luabind::def("get_field_of_view_left_degrees", &get_property<&::openvr::Instance::GetFieldOfViewLeftDegrees>)];
	modVr[luabind::def("get_field_of_view_right_degrees", &get_property<&::openvr::Instance::GetFieldOfViewRightDegrees>)];
	modVr[luabind::def("get_field_of_view_top_degrees", &get_property<&::openvr::Instance::GetFieldOfViewTopDegrees>)];
	modVr[luabind::def("get_field_of_view_bottom_degrees", &get_property<&::openvr::Instance::GetFieldOfViewBottomDegrees>)];
	modVr[luabind::def("get_tracking_range_minimum_meters", &get_property<&::openvr::Instance::GetTrackingRangeMinimumMeters>)];
	modVr[luabind::def("get_tracking_range_maximum_meters", &get_property<&::openvr::Instance::GetTrackingRangeMaximumMeters>)];
	modVr[luabind::def("get_status_display_transform", &get_property<&::openvr::Instance::GetStatusDisplayTransform>)];
	modVr[luabind::def("get_camera_to_head_transform", &get_property<&::openvr::Instance::GetCameraToHeadTransform>)];
	modVr[luabind::def("get_hardware_revision_number", &get_property<&::openvr::Instance::GetHardwareRevisionNumber>)];
	modVr[luabind::def("get_firmware_version", &get_property<&::openvr::Instance::GetFirmwareVersion>)];
	modVr[luabind::def("get_fpga_version", &get_property<&::openvr::Instance::GetFPGAVersion>)];
	modVr[luabind::def("get_vrc_version", &get_property<&::openvr::Instance::GetVRCVersion>)];
	modVr[luabind::def("get_radio_version", &get_property<&::openvr::Instance::GetRadioVersion>)];
	modVr[luabind::def("get_dongle_version", &get_property<&::openvr::Instance::GetDongleVersion>)];
	modVr[luabind::def("get_current_universe_id", &get_property<&::openvr::Instance::GetCurrentUniverseId>)];
	modVr[luabind::def("get_previous_universe_id", &get_property<&::openvr::Instance::GetPreviousUniverseId>)];
	modVr[luabind::def("get_display_firmware_version", &get_property<&::openvr::Instance::GetDisplayFirmwareVersion>)];
	modVr[luabind::def("get_camera_firmware_version", &get_property<&::openvr::Instance::GetCameraFirmwareVersion>)];
	modVr[luabind::def("get_display_fpga_version", &get_property<&::openvr::Instance::GetDisplayFPGAVersion>)];
	modVr[luabind::def("get_display_bootloader_version", &get_property<&::openvr::Instance::GetDisplayBootloaderVersion>)];
	modVr[luabind::def("get_display_hardware_version", &get_property<&::openvr::Instance::GetDisplayHardwareVersion>)];
	modVr[luabind::def("get_audio_firmware_version", &get_property<&::openvr::Instance::GetAudioFirmwareVersion>)];
	modVr[luabind::def("get_supported_buttons", &get_property<&::openvr::Instance::GetSupportedButtons>)];
	modVr[luabind::def("get_device_class", &get_property<&::openvr::Instance::GetDeviceClass>)];
	modVr[luabind::def("get_display_mc_type", &get_property<&::openvr::Instance::GetDisplayMCType>)];
	modVr[luabind::def("get_edid_vendor_id", &get_property<&::openvr::Instance::GetEdidVendorID>)];
	modVr[luabind::def("get_edid_product_id", &get_property<&::openvr::Instance::GetEdidProductID>)];
	modVr[luabind::def("get_display_gc_type", &get_property<&::openvr::Instance::GetDisplayGCType>)];
	modVr[luabind::def("get_camera_compatibility_mode", &get_property<&::openvr::Instance::GetCameraCompatibilityMode>)];
	modVr[luabind::def("get_axis0_type", &get_property<&::openvr::Instance::GetAxis0Type>)];
	modVr[luabind::def("get_axis1_type", &get_property<&::openvr::Instance::GetAxis1Type>)];
	modVr[luabind::def("get_axis2_type", &get_property<&::openvr::Instance::GetAxis2Type>)];
	modVr[luabind::def("get_axis3_type", &get_property<&::openvr::Instance::GetAxis3Type>)];
	modVr[luabind::def("get_axis4_type", &get_property<&::openvr::Instance::GetAxis4Type>)];

	modVr[luabind::def("fade_grid", &::openvr::Instance::FadeGrid)];
	modVr[luabind::def("show_mirror_window", &::openvr::Instance::ShowMirrorWindow)];
	modVr[luabind::def("hide_mirror_window", &::openvr::Instance::HideMirrorWindow)];
	modVr[luabind::def("is_mirror_window_visible", &::openvr::Instance::IsMirrorWindowVisible)];
	modVr[luabind::def("set_hmd_view_enabled", &::openvr::Instance::SetHmdViewEnabled)];
	modVr[luabind::def("is_hmd_view_enabled", &::openvr::Instance::IsHmdViewEnabled)];
	modVr[luabind::def("can_render_scene", &::openvr::Instance::CanRenderScene)];
	modVr[luabind::def("clear_last_submitted_frame", &::openvr::Instance::ClearLastSubmittedFrame)];
	modVr[luabind::def("clear_skybox_override", &::openvr::Instance::ClearSkyboxOverride)];
	modVr[luabind::def("compositor_bring_to_front", &::openvr::Instance::CompositorBringToFront)];
	modVr[luabind::def("compositor_dump_images", &::openvr::Instance::CompositorDumpImages)];
	modVr[luabind::def("compositor_go_to_back", &::openvr::Instance::CompositorGoToBack)];
	modVr[luabind::def("force_interleaved_reprojection_on", &::openvr::Instance::ForceInterleavedReprojectionOn)];
	modVr[luabind::def("force_reconnect_process", &::openvr::Instance::ForceReconnectProcess)];
	modVr[luabind::def("get_frame_time_remaining", &::openvr::Instance::GetFrameTimeRemaining)];
	modVr[luabind::def("is_fullscreen", &::openvr::Instance::IsFullscreen)];
	modVr[luabind::def("should_app_render_with_low_resources", &::openvr::Instance::ShouldAppRenderWithLowResources)];
	modVr[luabind::def("suspend_rendering", &::openvr::Instance::SuspendRendering)];
	modVr[luabind::def("is_rendering_suspended", &::openvr::Instance::IsRenderingSuspended)];
	modVr[luabind::def("get_tracking_space", &::openvr::Instance::GetTrackingSpace)];
	modVr[luabind::def("set_tracking_space", &::openvr::Instance::SetTrackingSpace)];
	modVr[luabind::def("get_tracked_device_class", &vr::IVRSystem::GetTrackedDeviceClass)];
	modVr[luabind::def("get_controller_role", &::openvr::Instance::GetTrackedDeviceRole)];
	modVr[luabind::def("is_tracked_device_connected", &vr::IVRSystem::IsTrackedDeviceConnected)];
	modVr[luabind::def("set_skybox_override", static_cast<vr::EVRCompositorError (::openvr::Instance::*)(prosper::IImage &, prosper::IImage &, prosper::IImage &, prosper::IImage &, prosper::IImage &, prosper::IImage &) const>(&::openvr::Instance::SetSkyboxOverride))];
	modVr[luabind::def("set_skybox_override", static_cast<vr::EVRCompositorError (::openvr::Instance::*)(prosper::IImage &, prosper::IImage &) const>(&::openvr::Instance::SetSkyboxOverride))];
	modVr[luabind::def("set_skybox_override", static_cast<vr::EVRCompositorError (::openvr::Instance::*)(prosper::IImage &) const>(&::openvr::Instance::SetSkyboxOverride))];

	modVr[luabind::def(
	  "is_initialized", +[](lua_State *l) { return luabind::detail::get_openvr_instance(l) != nullptr; })];
	modVr[luabind::def("get_cumulative_stats", &::openvr::Instance::GetCumulativeStats)];
	modVr[luabind::def("get_recommended_render_target_size", &get_recommended_render_target_size)];
	modVr[luabind::def("get_projection_matrix", &get_projection_matrix)];
	modVr[luabind::def("get_projection_raw", &get_projection_raw)];
	modVr[luabind::def("compute_distortion", &compute_distortion)];
	modVr[luabind::def("get_eye_to_head_transform", &get_eye_to_head_transform)];
	modVr[luabind::def("get_time_since_last_vsync", &get_time_since_last_vsync)];
	modVr[luabind::def("get_device_to_absolute_tracking_pose", &get_device_to_absolute_tracking_pose)];
	modVr[luabind::def("compute_seconds_to_photons", &compute_seconds_to_photons)];
	modVr[luabind::def("get_seated_zero_pose_to_standing_absolute_tracking_pose", &get_seated_zero_pose_to_standing_absolute_tracking_pose)];
	modVr[luabind::def("trigger_haptic_pulse", &trigger_haptic_pulse)];
	modVr[luabind::def("get_controller_state", &get_controller_state)];
	modVr[luabind::def("get_controller_states", &get_controller_states)];
	modVr[luabind::def("get_controller_state_with_pose", &get_controller_state_with_pose)];

	modVr[luabind::def("property_error_to_string", &property_error_to_string)];
	modVr[luabind::def("init_error_to_string", &init_error_to_string)];
	modVr[luabind::def("compositor_error_to_string", &compositor_error_to_string)];
	modVr[luabind::def("button_id_to_string", &button_id_to_string)];
	modVr[luabind::def("controller_axis_type_to_string", &controller_axis_type_to_string)];
	modVr[luabind::def("event_type_to_string", &event_type_to_string)];

	modVr[luabind::def("get_pose_transform", &get_pose_transform)];
	modVr[luabind::def("get_pose", &get_pose)];
	modVr[luabind::def("get_raw_pose", &get_raw_pose)];
	modVr[luabind::def("set_device_zero_pose", &::openvr::Instance::SetDeviceZeroPose)];
	modVr[luabind::def("get_inverse_device_zero_pose", &::openvr::Instance::GetInverseDeviceZeroPose)];

	std::unordered_map<std::string, lua_Integer> propErrorEnums {
	  {"TRACKED_PROPERTY_ERROR_SUCCESS", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_Success)},
	  {"TRACKED_PROPERTY_ERROR_WRONG_DATA_TYPE", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_WrongDataType)},
	  {"TRACKED_PROPERTY_ERROR_WRONG_DEVICE_CLASS", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_WrongDeviceClass)},
	  {"TRACKED_PROPERTY_ERROR_BUFFER_TOO_SMALL", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_BufferTooSmall)},
	  {"TRACKED_PROPERTY_ERROR_UNKNOWN_PROPERTY", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_UnknownProperty)},
	  {"TRACKED_PROPERTY_ERROR_INVALID_DEVICE", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_InvalidDevice)},
	  {"TRACKED_PROPERTY_ERROR_COULD_NOT_CONTACT_SERVER", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_CouldNotContactServer)},
	  {"TRACKED_PROPERTY_ERROR_VALUE_NOT_PROVIDED_BY_DEVICE", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_ValueNotProvidedByDevice)},
	  {"TRACKED_PROPERTY_ERROR_STRING_EXCEEDS_MAXIMUM_LENGTH", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_StringExceedsMaximumLength)},
	  {"TRACKED_PROPERTY_ERROR_NOT_YET_AVAILABLE", static_cast<int32_t>(vr::ETrackedPropertyError::TrackedProp_NotYetAvailable)},
	};
	Lua::RegisterLibraryEnums(lua, "openvr", propErrorEnums);

	std::unordered_map<std::string, lua_Integer> activityLevelEnums {
	  {"DEVICE_ACTIVITY_LEVEL_UNKNOWN", static_cast<int32_t>(vr::EDeviceActivityLevel::k_EDeviceActivityLevel_Unknown)},
	  {"DEVICE_ACTIVITY_LEVEL_IDLE", static_cast<int32_t>(vr::EDeviceActivityLevel::k_EDeviceActivityLevel_Idle)},
	  {"DEVICE_ACTIVITY_LEVEL_USER_INTERACTION", static_cast<int32_t>(vr::EDeviceActivityLevel::k_EDeviceActivityLevel_UserInteraction)},
	  {"DEVICE_ACTIVITY_LEVEL_USER_INTERACTION_TIMEOUT", static_cast<int32_t>(vr::EDeviceActivityLevel::k_EDeviceActivityLevel_UserInteraction_Timeout)},
	  {"DEVICE_ACTIVITY_LEVEL_STANDBY", static_cast<int32_t>(vr::EDeviceActivityLevel::k_EDeviceActivityLevel_Standby)},
	  {"DEVICE_ACTIVITY_LEVEL_IDLE_TIMEOUT", static_cast<int32_t>(vr::EDeviceActivityLevel::k_EDeviceActivityLevel_Idle_Timeout)},
	};
	Lua::RegisterLibraryEnums(lua, "openvr", activityLevelEnums);

	std::unordered_map<std::string, lua_Integer> propTrackingResults {
	  {"TRACKING_RESULT_UNINITIALIZED", umath::to_integral(vr::ETrackingResult::TrackingResult_Uninitialized)},
	  {"TRACKING_RESULT_CALIBRATING_IN_PROGRESS", umath::to_integral(vr::ETrackingResult::TrackingResult_Calibrating_InProgress)},
	  {"TRACKING_RESULT_CALIBRATING_OUT_OF_RANGE", umath::to_integral(vr::ETrackingResult::TrackingResult_Calibrating_OutOfRange)},
	  {"TRACKING_RESULT_RUNNING_OK", umath::to_integral(vr::ETrackingResult::TrackingResult_Running_OK)},
	  {"TRACKING_RESULT_RUNNING_OUT_OF_RANGE", umath::to_integral(vr::ETrackingResult::TrackingResult_Running_OutOfRange)},
	};
	Lua::RegisterLibraryEnums(lua, "openvr", propTrackingResults);

	std::unordered_map<std::string, lua_Integer> trackedControllerRoles {
	  {"TRACKED_CONTROLLER_ROLE_INVALID", umath::to_integral(vr::ETrackedControllerRole::TrackedControllerRole_Invalid)},
	  {"TRACKED_CONTROLLER_ROLE_LEFT_HAND", umath::to_integral(vr::ETrackedControllerRole::TrackedControllerRole_LeftHand)},
	  {"TRACKED_CONTROLLER_ROLE_RIGHT_HAND", umath::to_integral(vr::ETrackedControllerRole::TrackedControllerRole_RightHand)},
	  {"TRACKED_CONTROLLER_ROLE_OPT_OUT", umath::to_integral(vr::ETrackedControllerRole::TrackedControllerRole_OptOut)},
	  {"TRACKED_CONTROLLER_ROLE_TREADMILL", umath::to_integral(vr::ETrackedControllerRole::TrackedControllerRole_Treadmill)},
	  {"TRACKED_CONTROLLER_ROLE_STYLUS", umath::to_integral(vr::ETrackedControllerRole::TrackedControllerRole_Stylus)},
	  {"TRACKED_CONTROLLER_ROLE_MAX", umath::to_integral(vr::ETrackedControllerRole::TrackedControllerRole_Max)},
	};
	Lua::RegisterLibraryEnums(lua, "openvr", trackedControllerRoles);

	std::unordered_map<std::string, lua_Integer> initErrorEnums {
	  {"INIT_ERROR_NONE", static_cast<int32_t>(vr::EVRInitError::VRInitError_None)},
	  {"INIT_ERROR_UNKNOWN", static_cast<int32_t>(vr::EVRInitError::VRInitError_Unknown)},

	  {"INIT_ERROR_INIT_INSTALLATION_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_InstallationNotFound)},
	  {"INIT_ERROR_INIT_INSTALLATION_CORRUPT", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_InstallationCorrupt)},
	  {"INIT_ERROR_INIT_VR_CLIENT_DLL_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_VRClientDLLNotFound)},
	  {"INIT_ERROR_INIT_FILE_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_FileNotFound)},
	  {"INIT_ERROR_INIT_FACTORY_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_FactoryNotFound)},
	  {"INIT_ERROR_INIT_INTERFACE_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_InterfaceNotFound)},
	  {"INIT_ERROR_INIT_INVALID_INTERFACE", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_InvalidInterface)},
	  {"INIT_ERROR_INIT_USER_CONFIG_DIRECTORY_INVALID", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_UserConfigDirectoryInvalid)},
	  {"INIT_ERROR_INIT_HMD_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_HmdNotFound)},
	  {"INIT_ERROR_INIT_NOT_INITIALIZED", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_NotInitialized)},
	  {"INIT_ERROR_INIT_PATH_REGISTRY_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_PathRegistryNotFound)},
	  {"INIT_ERROR_INIT_NO_CONFIG_PATH", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_NoConfigPath)},
	  {"INIT_ERROR_INIT_NO_LOG_PATH", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_NoLogPath)},
	  {"INIT_ERROR_INIT_PATH_REGISTRY_NOT_WRITABLE", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_PathRegistryNotWritable)},
	  {"INIT_ERROR_INIT_APP_INFO_INIT_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_AppInfoInitFailed)},
	  {"INIT_ERROR_INIT_RETRY", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_Retry)},
	  {"INIT_ERROR_INIT_CANCELED_BY_USER", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_InitCanceledByUser)},
	  {"INIT_ERROR_INIT_ANOTHER_APP_LAUNCHING", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_AnotherAppLaunching)},
	  {"INIT_ERROR_INIT_SETTINGS_INIT_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_SettingsInitFailed)},
	  {"INIT_ERROR_INIT_SHUTTING_DOWN", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_ShuttingDown)},
	  {"INIT_ERROR_INIT_TOO_MANY_OBJECTS", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_TooManyObjects)},
	  {"INIT_ERROR_INIT_NO_SERVER_FOR_BACKGROUND_APP", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_NoServerForBackgroundApp)},
	  {"INIT_ERROR_INIT_NOT_SUPPORTED_WITH_COMPOSITOR", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_NotSupportedWithCompositor)},
	  {"INIT_ERROR_INIT_NOT_AVAILABLE_TO_UTILITY_APPS", static_cast<int32_t>(vr::EVRInitError::VRInitError_Init_NotAvailableToUtilityApps)},

	  {"INIT_ERROR_DRIVER_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_Failed)},
	  {"INIT_ERROR_DRIVER_UNKNOWN", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_Unknown)},
	  {"INIT_ERROR_DRIVER_HMD_UNKNOWN", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_HmdUnknown)},
	  {"INIT_ERROR_DRIVER_NOT_LOADED", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_NotLoaded)},
	  {"INIT_ERROR_DRIVER_RUNTIME_OUT_OF_DATE", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_RuntimeOutOfDate)},
	  {"INIT_ERROR_DRIVER_HMD_IN_USE", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_HmdInUse)},
	  {"INIT_ERROR_DRIVER_NOT_CALIBRATED", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_NotCalibrated)},
	  {"INIT_ERROR_DRIVER_CALIBRATION_INVALID", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_CalibrationInvalid)},
	  {"INIT_ERROR_DRIVER_HMD_DISPLAY_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Driver_HmdDisplayNotFound)},

	  {"INIT_ERROR_IPC_SERVER_INIT_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_IPC_ServerInitFailed)},
	  {"INIT_ERROR_IPC_CONNECT_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_IPC_ConnectFailed)},
	  {"INIT_ERROR_IPC_SHARED_STATE_INIT_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_IPC_SharedStateInitFailed)},
	  {"INIT_ERROR_IPC_COMPOSITOR_INIT_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_IPC_CompositorInitFailed)},
	  {"INIT_ERROR_IPC_MUTEX_INIT_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_IPC_MutexInitFailed)},
	  {"INIT_ERROR_IPC_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_IPC_Failed)},

	  {"INIT_ERROR_COMPOSITOR_FAILED", static_cast<int32_t>(vr::EVRInitError::VRInitError_Compositor_Failed)},
	  {"INIT_ERROR_COMPOSITOR_D3D11_HARDWARE_REQUIRED", static_cast<int32_t>(vr::EVRInitError::VRInitError_Compositor_D3D11HardwareRequired)},

	  {"INIT_ERROR_VENDOR_SPECIFIC_UNABLE_TO_CONNECT_TO_OCULUS_RUNTIME", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_UnableToConnectToOculusRuntime)},

	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_CANT_OPEN_DEVICE", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_CantOpenDevice)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_UNABLE_TO_REQUEST_CONFIG_START", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_UnableToRequestConfigStart)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_NO_STORED_CONFIG", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_NoStoredConfig)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_CONFIG_TOO_BIG", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_ConfigTooBig)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_CONFIG_TOO_SMALL", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_ConfigTooSmall)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_UNABLE_TO_INIT_ZLIB", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_UnableToInitZLib)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_CANT_READ_FIRMWARE_VERSION", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_CantReadFirmwareVersion)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_UNABLE_TO_SEND_USER_DATA_START", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_UnableToSendUserDataStart)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_UNABLE_TO_GET_USER_DATA_START", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_UnableToGetUserDataStart)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_UNABLE_TO_GET_USER_DATA_NEXT", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_UnableToGetUserDataNext)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_USER_DATA_ADDRESS_RANGE", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_UserDataAddressRange)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_USER_DATA_ERROR", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_UserDataError)},
	  {"INIT_ERROR_VENDOR_SPECIFIC_HMD_FOUND_CONFIG_FAILED_SANITY_CHECK", static_cast<int32_t>(vr::EVRInitError::VRInitError_VendorSpecific_HmdFound_ConfigFailedSanityCheck)},

	  {"INIT_ERROR_STEAM_INSTALLATION_NOT_FOUND", static_cast<int32_t>(vr::EVRInitError::VRInitError_Steam_SteamInstallationNotFound)},

	  {"COMPOSITOR_ERROR_NONE", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_None)},
	  {"COMPOSITOR_ERROR_REQUEST_FAILED", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_RequestFailed)},
	  {"COMPOSITOR_ERROR_INCOMPATIBLE_VERSION", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_IncompatibleVersion)},
	  {"COMPOSITOR_ERROR_DO_NOT_HAVE_FOCUS", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_DoNotHaveFocus)},
	  {"COMPOSITOR_ERROR_INVALID_TEXTURE", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_InvalidTexture)},
	  {"COMPOSITOR_ERROR_IS_NOT_SCENE_APPLICATION", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_IsNotSceneApplication)},
	  {"COMPOSITOR_ERROR_TEXTURE_IS_ON_WRONG_DEVICE", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_TextureIsOnWrongDevice)},
	  {"COMPOSITOR_ERROR_TEXTURE_USES_UNSUPPORTED_FORMAT", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_TextureUsesUnsupportedFormat)},
	  {"COMPOSITOR_ERROR_SHARED_TEXTURES_NOT_SUPPORTED", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_SharedTexturesNotSupported)},
	  {"COMPOSITOR_ERROR_INDEX_OUT_OF_RANGE", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_IndexOutOfRange)},
	  {"COMPOSITOR_ERROR_ALREADY_SUBMITTED", static_cast<int32_t>(vr::EVRCompositorError::VRCompositorError_AlreadySubmitted)},

	  {"TRACKING_UNIVERSE_ORIGIN_SEATED", static_cast<int32_t>(vr::ETrackingUniverseOrigin::TrackingUniverseSeated)},
	  {"TRACKING_UNIVERSE_ORIGIN_STANDING", static_cast<int32_t>(vr::ETrackingUniverseOrigin::TrackingUniverseStanding)},
	  {"TRACKING_UNIVERSE_ORIGIN_RAW_AND_UNCALIBRATED", static_cast<int32_t>(vr::ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated)},

	  {"EYE_LEFT", umath::to_integral(vr::EVREye::Eye_Left)},
	  {"EYE_RIGHT", umath::to_integral(vr::EVREye::Eye_Right)},

	  {"TRACKED_DEVICE_CLASS_INVALID", umath::to_integral(vr::ETrackedDeviceClass::TrackedDeviceClass_Invalid)},
	  {"TRACKED_DEVICE_CLASS_HMD", umath::to_integral(vr::ETrackedDeviceClass::TrackedDeviceClass_HMD)},
	  {"TRACKED_DEVICE_CLASS_CONTROLLER", umath::to_integral(vr::ETrackedDeviceClass::TrackedDeviceClass_Controller)},
	  {"TRACKED_DEVICE_CLASS_GENERIC_TRACKER", umath::to_integral(vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker)},
	  {"TRACKED_DEVICE_CLASS_TRACKING_REFERENCE", umath::to_integral(vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference)},

	  {"BUTTON_ID_SYSTEM", static_cast<int32_t>(vr::EVRButtonId::k_EButton_System)},
	  {"BUTTON_ID_APPLICATION_MENU", static_cast<int32_t>(vr::EVRButtonId::k_EButton_ApplicationMenu)},
	  {"BUTTON_ID_GRIP", static_cast<int32_t>(vr::EVRButtonId::k_EButton_Grip)},
	  {"BUTTON_ID_DPAD_LEFT", static_cast<int32_t>(vr::EVRButtonId::k_EButton_DPad_Left)},
	  {"BUTTON_ID_DPAD_UP", static_cast<int32_t>(vr::EVRButtonId::k_EButton_DPad_Up)},
	  {"BUTTON_ID_DPAD_RIGHT", static_cast<int32_t>(vr::EVRButtonId::k_EButton_DPad_Right)},
	  {"BUTTON_ID_DPAD_DOWN", static_cast<int32_t>(vr::EVRButtonId::k_EButton_DPad_Down)},
	  {"BUTTON_ID_A", static_cast<int32_t>(vr::EVRButtonId::k_EButton_A)},
	  {"BUTTON_ID_PROXIMITY_SENSOR", static_cast<int32_t>(vr::EVRButtonId::k_EButton_ProximitySensor)},
	  {"BUTTON_ID_AXIS0", static_cast<int32_t>(vr::EVRButtonId::k_EButton_Axis0)},
	  {"BUTTON_ID_AXIS1", static_cast<int32_t>(vr::EVRButtonId::k_EButton_Axis1)},
	  {"BUTTON_ID_AXIS2", static_cast<int32_t>(vr::EVRButtonId::k_EButton_Axis2)},
	  {"BUTTON_ID_AXIS3", static_cast<int32_t>(vr::EVRButtonId::k_EButton_Axis3)},
	  {"BUTTON_ID_AXIS4", static_cast<int32_t>(vr::EVRButtonId::k_EButton_Axis4)},
	  {"BUTTON_ID_STEAMVR_TOUCHPAD", static_cast<int32_t>(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)},
	  {"BUTTON_ID_STEAMVR_TRIGGER", static_cast<int32_t>(vr::EVRButtonId::k_EButton_SteamVR_Trigger)},
	  {"BUTTON_ID_DASHBOARD_BACK", static_cast<int32_t>(vr::EVRButtonId::k_EButton_Dashboard_Back)},

	  {"CONTROLLER_AXIS_TYPE_NONE", static_cast<int32_t>(vr::EVRControllerAxisType::k_eControllerAxis_None)},
	  {"CONTROLLER_AXIS_TYPE_TRACK_PAD", static_cast<int32_t>(vr::EVRControllerAxisType::k_eControllerAxis_TrackPad)},
	  {"CONTROLLER_AXIS_TYPE_JOYSTICK", static_cast<int32_t>(vr::EVRControllerAxisType::k_eControllerAxis_Joystick)},
	  {"CONTROLLER_AXIS_TYPE_TRIGGER", static_cast<int32_t>(vr::EVRControllerAxisType::k_eControllerAxis_Trigger)},

	  {"MAX_TRACKED_DEVICE_COUNT", vr::k_unMaxTrackedDeviceCount},

	  {"EVENT_NONE", vr::VREvent_None},
	  {"EVENT_TRACKED_DEVICE_ACTIVATED", vr::VREvent_TrackedDeviceActivated},
	  {"EVENT_TRACKED_DEVICE_DEACTIVATED", vr::VREvent_TrackedDeviceDeactivated},
	  {"EVENT_TRACKED_DEVICE_UPDATED", vr::VREvent_TrackedDeviceUpdated},
	  {"EVENT_TRACKED_DEVICE_USER_INTERACTION_STARTED", vr::VREvent_TrackedDeviceUserInteractionStarted},
	  {"EVENT_TRACKED_DEVICE_USER_INTERACTION_ENDED", vr::VREvent_TrackedDeviceUserInteractionEnded},
	  {"EVENT_IPD_CHANGED", vr::VREvent_IpdChanged},
	  {"EVENT_ENTER_STANDBY_MODE", vr::VREvent_EnterStandbyMode},
	  {"EVENT_LEAVE_STANDBY_MODE", vr::VREvent_LeaveStandbyMode},
	  {"EVENT_TRACKED_DEVICE_ROLE_CHANGED", vr::VREvent_TrackedDeviceRoleChanged},
	  {"EVENT_WATCHDOG_WAKE_UP_REQUESTED", vr::VREvent_WatchdogWakeUpRequested},
	  {"EVENT_LENS_DISTORTION_CHANGED", vr::VREvent_LensDistortionChanged},
	  {"EVENT_PROPERTY_CHANGED", vr::VREvent_PropertyChanged},
	  {"EVENT_WIRELESS_DISCONNECT", vr::VREvent_WirelessDisconnect},
	  {"EVENT_WIRELESS_RECONNECT", vr::VREvent_WirelessReconnect},
	  {"EVENT_BUTTON_PRESS", vr::VREvent_ButtonPress},
	  {"EVENT_BUTTON_UNPRESS", vr::VREvent_ButtonUnpress},
	  {"EVENT_BUTTON_TOUCH", vr::VREvent_ButtonTouch},
	  {"EVENT_BUTTON_UNTOUCH", vr::VREvent_ButtonUntouch},
	  {"EVENT_MODAL_CANCEL", vr::VREvent_Modal_Cancel},
	  {"EVENT_MOUSE_MOVE", vr::VREvent_MouseMove},
	  {"EVENT_MOUSE_BUTTON_DOWN", vr::VREvent_MouseButtonDown},
	  {"EVENT_MOUSE_BUTTON_UP", vr::VREvent_MouseButtonUp},
	  {"EVENT_FOCUS_ENTER", vr::VREvent_FocusEnter},
	  {"EVENT_FOCUS_LEAVE", vr::VREvent_FocusLeave},
	  {"EVENT_SCROLL_DISCRETE", vr::VREvent_ScrollDiscrete},
	  {"EVENT_TOUCH_PAD_MOVE", vr::VREvent_TouchPadMove},
	  {"EVENT_OVERLAY_FOCUS_CHANGED", vr::VREvent_OverlayFocusChanged},
	  {"EVENT_RELOAD_OVERLAYS", vr::VREvent_ReloadOverlays},
	  {"EVENT_SCROLL_SMOOTH", vr::VREvent_ScrollSmooth},
	  {"EVENT_LOCK_MOUSE_POSITION", vr::VREvent_LockMousePosition},
	  {"EVENT_UNLOCK_MOUSE_POSITION", vr::VREvent_UnlockMousePosition},
	  {"EVENT_INPUT_FOCUS_CAPTURED", vr::VREvent_InputFocusCaptured},
	  {"EVENT_INPUT_FOCUS_RELEASED", vr::VREvent_InputFocusReleased},
	  {"EVENT_SCENE_APPLICATION_CHANGED", vr::VREvent_SceneApplicationChanged},
	  {"EVENT_INPUT_FOCUS_CHANGED", vr::VREvent_InputFocusChanged},
	  {"EVENT_SCENE_APPLICATION_USING_WRONG_GRAPHICS_ADAPTER", vr::VREvent_SceneApplicationUsingWrongGraphicsAdapter},
	  {"EVENT_ACTION_BINDING_RELOADED", vr::VREvent_ActionBindingReloaded},
	  {"EVENT_HIDE_RENDER_MODELS", vr::VREvent_HideRenderModels},
	  {"EVENT_SHOW_RENDER_MODELS", vr::VREvent_ShowRenderModels},
	  {"EVENT_SCENE_APPLICATION_STATE_CHANGED", vr::VREvent_SceneApplicationStateChanged},
	  {"EVENT_CONSOLE_OPENED", vr::VREvent_ConsoleOpened},
	  {"EVENT_CONSOLE_CLOSED", vr::VREvent_ConsoleClosed},
	  {"EVENT_OVERLAY_SHOWN", vr::VREvent_OverlayShown},
	  {"EVENT_OVERLAY_HIDDEN", vr::VREvent_OverlayHidden},
	  {"EVENT_DASHBOARD_ACTIVATED", vr::VREvent_DashboardActivated},
	  {"EVENT_DASHBOARD_DEACTIVATED", vr::VREvent_DashboardDeactivated},
	  {"EVENT_DASHBOARD_REQUESTED", vr::VREvent_DashboardRequested},
	  {"EVENT_RESET_DASHBOARD", vr::VREvent_ResetDashboard},
	  {"EVENT_IMAGE_LOADED", vr::VREvent_ImageLoaded},
	  {"EVENT_SHOW_KEYBOARD", vr::VREvent_ShowKeyboard},
	  {"EVENT_HIDE_KEYBOARD", vr::VREvent_HideKeyboard},
	  {"EVENT_OVERLAY_GAMEPAD_FOCUS_GAINED", vr::VREvent_OverlayGamepadFocusGained},
	  {"EVENT_OVERLAY_GAMEPAD_FOCUS_LOST", vr::VREvent_OverlayGamepadFocusLost},
	  {"EVENT_OVERLAY_SHARED_TEXTURE_CHANGED", vr::VREvent_OverlaySharedTextureChanged},
	  {"EVENT_SCREENSHOT_TRIGGERED", vr::VREvent_ScreenshotTriggered},
	  {"EVENT_IMAGE_FAILED", vr::VREvent_ImageFailed},
	  {"EVENT_DASHBOARD_OVERLAY_CREATED", vr::VREvent_DashboardOverlayCreated},
	  {"EVENT_SWITCH_GAMEPAD_FOCUS", vr::VREvent_SwitchGamepadFocus},
	  {"EVENT_REQUEST_SCREENSHOT", vr::VREvent_RequestScreenshot},
	  {"EVENT_SCREENSHOT_TAKEN", vr::VREvent_ScreenshotTaken},
	  {"EVENT_SCREENSHOT_FAILED", vr::VREvent_ScreenshotFailed},
	  {"EVENT_SUBMIT_SCREENSHOT_TO_DASHBOARD", vr::VREvent_SubmitScreenshotToDashboard},
	  {"EVENT_SCREENSHOT_PROGRESS_TO_DASHBOARD", vr::VREvent_ScreenshotProgressToDashboard},
	  {"EVENT_PRIMARY_DASHBOARD_DEVICE_CHANGED", vr::VREvent_PrimaryDashboardDeviceChanged},
	  {"EVENT_ROOM_VIEW_SHOWN", vr::VREvent_RoomViewShown},
	  {"EVENT_ROOM_VIEW_HIDDEN", vr::VREvent_RoomViewHidden},
	  {"EVENT_SHOW_UI", vr::VREvent_ShowUI},
	  {"EVENT_SHOW_DEV_TOOLS", vr::VREvent_ShowDevTools},
	  {"EVENT_DESKTOP_VIEW_UPDATING", vr::VREvent_DesktopViewUpdating},
	  {"EVENT_DESKTOP_VIEW_READY", vr::VREvent_DesktopViewReady},
	  {"EVENT_NOTIFICATION_SHOWN", vr::VREvent_Notification_Shown},
	  {"EVENT_NOTIFICATION_HIDDEN", vr::VREvent_Notification_Hidden},
	  {"EVENT_NOTIFICATION_BEGIN_INTERACTION", vr::VREvent_Notification_BeginInteraction},
	  {"EVENT_NOTIFICATION_DESTROYED", vr::VREvent_Notification_Destroyed},
	  {"EVENT_QUIT", vr::VREvent_Quit},
	  {"EVENT_PROCESS_QUIT", vr::VREvent_ProcessQuit},
	  {"EVENT_QUIT_ACKNOWLEDGED", vr::VREvent_QuitAcknowledged},
	  {"EVENT_DRIVER_REQUESTED_QUIT", vr::VREvent_DriverRequestedQuit},
	  {"EVENT_RESTART_REQUESTED", vr::VREvent_RestartRequested},
	  {"EVENT_CHAPERONE_DATA_HAS_CHANGED", vr::VREvent_ChaperoneDataHasChanged},
	  {"EVENT_CHAPERONE_UNIVERSE_HAS_CHANGED", vr::VREvent_ChaperoneUniverseHasChanged},
	  {"EVENT_CHAPERONE_TEMP_DATA_HAS_CHANGED", vr::VREvent_ChaperoneTempDataHasChanged},
	  {"EVENT_CHAPERONE_SETTINGS_HAVE_CHANGED", vr::VREvent_ChaperoneSettingsHaveChanged},
	  {"EVENT_SEATED_ZERO_POSE_RESET", vr::VREvent_SeatedZeroPoseReset},
	  {"EVENT_CHAPERONE_FLUSH_CACHE", vr::VREvent_ChaperoneFlushCache},
	  {"EVENT_CHAPERONE_ROOM_SETUP_STARTING", vr::VREvent_ChaperoneRoomSetupStarting},
	  {"EVENT_CHAPERONE_ROOM_SETUP_FINISHED", vr::VREvent_ChaperoneRoomSetupFinished},
	  {"EVENT_STANDING_ZERO_POSE_RESET", vr::VREvent_StandingZeroPoseReset},
	  {"EVENT_AUDIO_SETTINGS_HAVE_CHANGED", vr::VREvent_AudioSettingsHaveChanged},
	  {"EVENT_BACKGROUND_SETTING_HAS_CHANGED", vr::VREvent_BackgroundSettingHasChanged},
	  {"EVENT_CAMERA_SETTINGS_HAVE_CHANGED", vr::VREvent_CameraSettingsHaveChanged},
	  {"EVENT_REPROJECTION_SETTING_HAS_CHANGED", vr::VREvent_ReprojectionSettingHasChanged},
	  {"EVENT_MODEL_SKIN_SETTINGS_HAVE_CHANGED", vr::VREvent_ModelSkinSettingsHaveChanged},
	  {"EVENT_ENVIRONMENT_SETTINGS_HAVE_CHANGED", vr::VREvent_EnvironmentSettingsHaveChanged},
	  {"EVENT_POWER_SETTINGS_HAVE_CHANGED", vr::VREvent_PowerSettingsHaveChanged},
	  {"EVENT_ENABLE_HOME_APP_SETTINGS_HAVE_CHANGED", vr::VREvent_EnableHomeAppSettingsHaveChanged},
	  {"EVENT_STEAM_VR_SECTION_SETTING_CHANGED", vr::VREvent_SteamVRSectionSettingChanged},
	  {"EVENT_LIGHTHOUSE_SECTION_SETTING_CHANGED", vr::VREvent_LighthouseSectionSettingChanged},
	  {"EVENT_NULL_SECTION_SETTING_CHANGED", vr::VREvent_NullSectionSettingChanged},
	  {"EVENT_USER_INTERFACE_SECTION_SETTING_CHANGED", vr::VREvent_UserInterfaceSectionSettingChanged},
	  {"EVENT_NOTIFICATIONS_SECTION_SETTING_CHANGED", vr::VREvent_NotificationsSectionSettingChanged},
	  {"EVENT_KEYBOARD_SECTION_SETTING_CHANGED", vr::VREvent_KeyboardSectionSettingChanged},
	  {"EVENT_PERF_SECTION_SETTING_CHANGED", vr::VREvent_PerfSectionSettingChanged},
	  {"EVENT_DASHBOARD_SECTION_SETTING_CHANGED", vr::VREvent_DashboardSectionSettingChanged},
	  {"EVENT_WEB_INTERFACE_SECTION_SETTING_CHANGED", vr::VREvent_WebInterfaceSectionSettingChanged},
	  {"EVENT_TRACKERS_SECTION_SETTING_CHANGED", vr::VREvent_TrackersSectionSettingChanged},
	  {"EVENT_LAST_KNOWN_SECTION_SETTING_CHANGED", vr::VREvent_LastKnownSectionSettingChanged},
	  {"EVENT_DISMISSED_WARNINGS_SECTION_SETTING_CHANGED", vr::VREvent_DismissedWarningsSectionSettingChanged},
	  {"EVENT_GPU_SPEED_SECTION_SETTING_CHANGED", vr::VREvent_GpuSpeedSectionSettingChanged},
	  {"EVENT_WINDOWS_MR_SECTION_SETTING_CHANGED", vr::VREvent_WindowsMRSectionSettingChanged},
	  {"EVENT_OTHER_SECTION_SETTING_CHANGED", vr::VREvent_OtherSectionSettingChanged},
	  {"EVENT_STATUS_UPDATE", vr::VREvent_StatusUpdate},
	  {"EVENT_WEB_INTERFACE_INSTALL_DRIVER_COMPLETED", vr::VREvent_WebInterface_InstallDriverCompleted},
	  {"EVENT_MC_IMAGE_UPDATED", vr::VREvent_MCImageUpdated},
	  {"EVENT_FIRMWARE_UPDATE_STARTED", vr::VREvent_FirmwareUpdateStarted},
	  {"EVENT_FIRMWARE_UPDATE_FINISHED", vr::VREvent_FirmwareUpdateFinished},
	  {"EVENT_KEYBOARD_CLOSED", vr::VREvent_KeyboardClosed},
	  {"EVENT_KEYBOARD_CHAR_INPUT", vr::VREvent_KeyboardCharInput},
	  {"EVENT_KEYBOARD_DONE", vr::VREvent_KeyboardDone},
	  {"EVENT_APPLICATION_LIST_UPDATED", vr::VREvent_ApplicationListUpdated},
	  {"EVENT_APPLICATION_MIME_TYPE_LOAD", vr::VREvent_ApplicationMimeTypeLoad},
	  {"EVENT_PROCESS_CONNECTED", vr::VREvent_ProcessConnected},
	  {"EVENT_PROCESS_DISCONNECTED", vr::VREvent_ProcessDisconnected},
	  {"EVENT_COMPOSITOR_CHAPERONE_BOUNDS_SHOWN", vr::VREvent_Compositor_ChaperoneBoundsShown},
	  {"EVENT_COMPOSITOR_CHAPERONE_BOUNDS_HIDDEN", vr::VREvent_Compositor_ChaperoneBoundsHidden},
	  {"EVENT_COMPOSITOR_DISPLAY_DISCONNECTED", vr::VREvent_Compositor_DisplayDisconnected},
	  {"EVENT_COMPOSITOR_DISPLAY_RECONNECTED", vr::VREvent_Compositor_DisplayReconnected},
	  {"EVENT_COMPOSITOR_HDCP_ERROR", vr::VREvent_Compositor_HDCPError},
	  {"EVENT_COMPOSITOR_APPLICATION_NOT_RESPONDING", vr::VREvent_Compositor_ApplicationNotResponding},
	  {"EVENT_COMPOSITOR_APPLICATION_RESUMED", vr::VREvent_Compositor_ApplicationResumed},
	  {"EVENT_COMPOSITOR_OUT_OF_VIDEO_MEMORY", vr::VREvent_Compositor_OutOfVideoMemory},
	  {"EVENT_COMPOSITOR_DISPLAY_MODE_NOT_SUPPORTED", vr::VREvent_Compositor_DisplayModeNotSupported},
	  {"EVENT_COMPOSITOR_STAGE_OVERRIDE_READY", vr::VREvent_Compositor_StageOverrideReady},
	  {"EVENT_TRACKED_CAMERA_START_VIDEO_STREAM", vr::VREvent_TrackedCamera_StartVideoStream},
	  {"EVENT_TRACKED_CAMERA_STOP_VIDEO_STREAM", vr::VREvent_TrackedCamera_StopVideoStream},
	  {"EVENT_TRACKED_CAMERA_PAUSE_VIDEO_STREAM", vr::VREvent_TrackedCamera_PauseVideoStream},
	  {"EVENT_TRACKED_CAMERA_RESUME_VIDEO_STREAM", vr::VREvent_TrackedCamera_ResumeVideoStream},
	  {"EVENT_TRACKED_CAMERA_EDITING_SURFACE", vr::VREvent_TrackedCamera_EditingSurface},
	  {"EVENT_PERFORMANCE_TEST_ENABLE_CAPTURE", vr::VREvent_PerformanceTest_EnableCapture},
	  {"EVENT_PERFORMANCE_TEST_DISABLE_CAPTURE", vr::VREvent_PerformanceTest_DisableCapture},
	  {"EVENT_PERFORMANCE_TEST_FIDELITY_LEVEL", vr::VREvent_PerformanceTest_FidelityLevel},
	  {"EVENT_MESSAGE_OVERLAY_CLOSED", vr::VREvent_MessageOverlay_Closed},
	  {"EVENT_MESSAGE_OVERLAY_CLOSE_REQUESTED", vr::VREvent_MessageOverlayCloseRequested},
	  {"EVENT_INPUT_HAPTIC_VIBRATION", vr::VREvent_Input_HapticVibration},
	  {"EVENT_INPUT_BINDING_LOAD_FAILED", vr::VREvent_Input_BindingLoadFailed},
	  {"EVENT_INPUT_BINDING_LOAD_SUCCESSFUL", vr::VREvent_Input_BindingLoadSuccessful},
	  {"EVENT_INPUT_ACTION_MANIFEST_RELOADED", vr::VREvent_Input_ActionManifestReloaded},
	  {"EVENT_INPUT_ACTION_MANIFEST_LOAD_FAILED", vr::VREvent_Input_ActionManifestLoadFailed},
	  {"EVENT_INPUT_PROGRESS_UPDATE", vr::VREvent_Input_ProgressUpdate},
	  {"EVENT_INPUT_TRACKER_ACTIVATED", vr::VREvent_Input_TrackerActivated},
	  {"EVENT_INPUT_BINDINGS_UPDATED", vr::VREvent_Input_BindingsUpdated},
	  {"EVENT_INPUT_BINDING_SUBSCRIPTION_CHANGED", vr::VREvent_Input_BindingSubscriptionChanged},
	  {"EVENT_SPATIAL_ANCHORS_POSE_UPDATED", vr::VREvent_SpatialAnchors_PoseUpdated},
	  {"EVENT_SPATIAL_ANCHORS_DESCRIPTOR_UPDATED", vr::VREvent_SpatialAnchors_DescriptorUpdated},
	  {"EVENT_SPATIAL_ANCHORS_REQUEST_POSE_UPDATE", vr::VREvent_SpatialAnchors_RequestPoseUpdate},
	  {"EVENT_SPATIAL_ANCHORS_REQUEST_DESCRIPTOR_UPDATE", vr::VREvent_SpatialAnchors_RequestDescriptorUpdate},
	  {"EVENT_SYSTEM_REPORT_STARTED", vr::VREvent_SystemReport_Started},
	  {"EVENT_MONITOR_SHOW_HEADSET_VIEW", vr::VREvent_Monitor_ShowHeadsetView},
	  {"EVENT_MONITOR_HIDE_HEADSET_VIEW", vr::VREvent_Monitor_HideHeadsetView},
	};
	Lua::RegisterLibraryEnums(lua, "openvr", initErrorEnums);

	modVr[luabind::def(
	  "reset_zero_pose", +[](vr::ETrackingUniverseOrigin origin) {
		  if(s_vrInstance == nullptr)
			  return;
		  auto *chaperone = s_vrInstance->GetChaperone();
		  chaperone->ResetZeroPose(origin);
	  })];

	auto classDevDevicePose = luabind::class_<vr::TrackedDevicePose_t>("TrackedDevicePose")
	                            .def_readonly("deviceIsConnected", &vr::TrackedDevicePose_t::bDeviceIsConnected)
	                            .def_readonly("poseIsValid", &vr::TrackedDevicePose_t::bPoseIsValid)
	                            .def_readonly("trackingResult", &vr::TrackedDevicePose_t::eTrackingResult)
	                            .def_readonly("deviceToAbsoluteTracking", reinterpret_cast<Mat3x4 vr::TrackedDevicePose_t::*>(&vr::TrackedDevicePose_t::mDeviceToAbsoluteTracking))
	                            .def_readonly("angularVelocity", reinterpret_cast<Vector3 vr::TrackedDevicePose_t::*>(&vr::TrackedDevicePose_t::vAngularVelocity))
	                            .def_readonly("velocity", reinterpret_cast<Vector3 vr::TrackedDevicePose_t::*>(&vr::TrackedDevicePose_t::vVelocity));
	modVr[classDevDevicePose];

	auto classDefEye = luabind::class_<::openvr::Eye>("Eye");
	classDefEye.def("GetProjectionMatrix", static_cast<void (*)(lua_State *, ::openvr::Eye &, float, float)>([](lua_State *l, ::openvr::Eye &eye, float nearZ, float farZ) { Lua::Push<Mat4>(l, eye.GetEyeProjectionMatrix(nearZ, farZ)); }));
	classDefEye.def("GetViewMatrix", static_cast<void (*)(lua_State *, ::openvr::Eye &, pragma::CCameraComponent &)>([](lua_State *l, ::openvr::Eye &eye, pragma::CCameraComponent &cam) { Lua::Push<Mat4>(l, eye.GetEyeViewMatrix(cam)); }));
	modVr[classDefEye];

	auto classDefControllerState = luabind::class_<LuaVRControllerState>("ControllerState")
	                                 .def_readonly("packetNum", &LuaVRControllerState::unPacketNum)
	                                 .def_readonly("buttonPressed", &LuaVRControllerState::ulButtonPressed)
	                                 .def_readonly("buttonTouched", &LuaVRControllerState::ulButtonTouched)
	                                 .def_readonly("axis0", reinterpret_cast<Vector2 LuaVRControllerState::*>(&LuaVRControllerState::rAxis0))
	                                 .def_readonly("axis1", reinterpret_cast<Vector2 LuaVRControllerState::*>(&LuaVRControllerState::rAxis1))
	                                 .def_readonly("axis2", reinterpret_cast<Vector2 LuaVRControllerState::*>(&LuaVRControllerState::rAxis2))
	                                 .def_readonly("axis3", reinterpret_cast<Vector2 LuaVRControllerState::*>(&LuaVRControllerState::rAxis3))
	                                 .def_readonly("axis4", reinterpret_cast<Vector2 LuaVRControllerState::*>(&LuaVRControllerState::rAxis4));
	modVr[classDefControllerState];
}
