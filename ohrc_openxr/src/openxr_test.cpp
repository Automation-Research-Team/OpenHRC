
#include <ros/ros.h>

#include <cstdio>
#include <cstring>

// Required headers for OpenGL rendering, as well as for including openxr_platform
#define GL_GLEXT_PROTOTYPES
#define GL3_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>

// Required headers for windowing, as well as the XrGraphicsBindingOpenGLXlibKHR struct.
#include <GL/glx.h>
#include <X11/Xlib.h>

#define XR_USE_PLATFORM_XLIB
#define XR_USE_GRAPHICS_API_OPENGL
#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <std_msgs/Float32MultiArray.h>

#include "ohrc_msgs/BodyState.h"
#include "openxr/openxr.h"
#include "openxr/openxr_platform.h"

ros::Publisher publisher;
ros::Subscriber subscriber;

#define HAND_LEFT_INDEX 0
#define HAND_RIGHT_INDEX 1
#define HAND_COUNT 2

// true if XrResult is a success code, else print error message and return false
bool xr_check(XrInstance instance, XrResult result, const char* format, ...) {
  if (XR_SUCCEEDED(result))
    return true;

  char resultString[XR_MAX_RESULT_STRING_SIZE];
  xrResultToString(instance, result, resultString);

  char formatRes[XR_MAX_RESULT_STRING_SIZE + 1024];
  snprintf(formatRes, XR_MAX_RESULT_STRING_SIZE + 1023, "%s [%s] (%d)\n", format, resultString, result);

  va_list args;
  va_start(args, format);
  vprintf(formatRes, args);
  va_end(args);

  return false;
}

// we need an identity pose for creating spaces without offsets
static XrPosef identity_pose = { .orientation = { .x = 0, .y = 0, .z = 0, .w = 1.0 }, .position = { .x = 0, .y = 0, .z = 0 } };

// A small header with functions for OpenGL math
#define MATH_3D_IMPLEMENTATION
#include "math_3d.h"

// functions belonging to extensions must be loaded with xrGetInstanceProcAddr before use
static PFN_xrGetOpenGLGraphicsRequirementsKHR pfnGetOpenGLGraphicsRequirementsKHR = NULL;
static bool load_extension_function_pointers(XrInstance instance) {
  XrResult result = xrGetInstanceProcAddr(instance, "xrGetOpenGLGraphicsRequirementsKHR", (PFN_xrVoidFunction*)&pfnGetOpenGLGraphicsRequirementsKHR);
  if (!xr_check(instance, result, "Failed to get OpenGL graphics requirements function!"))
    return false;

  return true;
}

static SDL_Window* desktop_window;
static SDL_GLContext gl_context;

// don't need a gl loader for just one function, just load it ourselves'
PFNGLBLITNAMEDFRAMEBUFFERPROC _glBlitNamedFramebuffer;

void GLAPIENTRY MessageCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam) {
  fprintf(stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n", (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), type, severity, message);
}

bool init_sdl_window(Display** xDisplay, uint32_t* visualid, GLXFBConfig* glxFBConfig, GLXDrawable* glxDrawable, GLXContext* glxContext, int w, int h) {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("Unable to initialize SDL");
    return false;
  }

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 0);

  /* Create our window centered at half the VR resolution */
  desktop_window = SDL_CreateWindow("OpenXR Example", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, w / 2, h / 2, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
  if (!desktop_window) {
    printf("Unable to create window");
    return false;
  }

  gl_context = SDL_GL_CreateContext(desktop_window);

  glEnable(GL_DEBUG_OUTPUT);
  glDebugMessageCallback(MessageCallback, 0);

  SDL_GL_SetSwapInterval(0);

  _glBlitNamedFramebuffer = (PFNGLBLITNAMEDFRAMEBUFFERPROC)glXGetProcAddressARB((GLubyte*)"glBlitNamedFramebuffer");

  // HACK? OpenXR wants us to report these values, so "work around" SDL a
  // bit and get the underlying glx stuff. Does this still work when e.g.
  // SDL switches to xcb?
  *xDisplay = XOpenDisplay(NULL);
  *glxContext = glXGetCurrentContext();
  *glxDrawable = glXGetCurrentDrawable();

  return true;
}

std::vector<float> _haptic(HAND_COUNT, 0.0);
void cbHaptic(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  _haptic = msg->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "openxr_image_listener");
  ros::NodeHandle nh;

  publisher = nh.advertise<ohrc_msgs::BodyState>("body_state", 1);
  subscriber = nh.subscribe<std_msgs::Float32MultiArray>("haptic", 1, cbHaptic);

  // the instance handle can be thought of as the basic connection to the OpenXR runtime
  XrInstance instance = XR_NULL_HANDLE;
  // the session deals with the renderloop submitting frames to the runtime
  XrSession session = XR_NULL_HANDLE;

  int enabled_ext_count = 1;
  const char* enabled_exts[1] = { XR_KHR_OPENGL_ENABLE_EXTENSION_NAME };
  XrInstanceCreateInfo instance_create_info = {
	    .type = XR_TYPE_INSTANCE_CREATE_INFO,
	    .next = NULL,
	    .createFlags = 0,
	    .applicationInfo =
	        {
	            // some compilers have trouble with char* initialization
	            {.applicationName = ""},
	            .applicationVersion = 1,
	            {.engineName = ""},
	            .engineVersion = 0,
	            .apiVersion = XR_CURRENT_API_VERSION,
	        },
	    .enabledApiLayerCount = 0,
	    .enabledApiLayerNames = NULL,
	    .enabledExtensionCount = enabled_ext_count,
	    .enabledExtensionNames = enabled_exts,
	};
  strncpy(instance_create_info.applicationInfo.applicationName, "OpenXR OpenGL Example", XR_MAX_APPLICATION_NAME_SIZE);
  strncpy(instance_create_info.applicationInfo.engineName, "Custom", XR_MAX_ENGINE_NAME_SIZE);

  // Set the application info, including the engineName
  XrApplicationInfo appInfo;
  appInfo.applicationVersion = XR_MAKE_VERSION(1, 0, 0);
  appInfo.engineVersion = XR_MAKE_VERSION(1, 0, 0);
  appInfo.apiVersion = XR_CURRENT_API_VERSION;
  std::strcpy(appInfo.applicationName, "My OpenXR App");
  std::strcpy(appInfo.engineName, "My OpenXR Engine");  // <-- Set the engineName here
  instance_create_info.applicationInfo = appInfo;

  XrResult result = xrCreateInstance(&instance_create_info, &instance);
  if (result != XR_SUCCESS) {
    std::printf("Failed to create OpenXR instance: %d\n", result);
    return 1;
  }

  if (!load_extension_function_pointers(instance))
    return 1;

  // Get the system properties
  XrSystemGetInfo systemGetInfo = { XR_TYPE_SYSTEM_GET_INFO };
  systemGetInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
  XrSystemId systemId = XR_NULL_SYSTEM_ID;
  result = xrGetSystem(instance, &systemGetInfo, &systemId);
  if (result != XR_SUCCESS) {
    std::printf("Failed to get OpenXR system: %d\n", result);
    xrDestroyInstance(instance);
    return 1;
  }

  // Get the system properties
  XrSystemProperties systemProperties = { XR_TYPE_SYSTEM_PROPERTIES };
  result = xrGetSystemProperties(instance, systemId, &systemProperties);
  if (result != XR_SUCCESS) {
    std::printf("Failed to get OpenXR system properties: %d\n", result);
    xrDestroyInstance(instance);
    return 1;
  }

  // Print the HMD name
  std::printf("HMD Name: %s\n", systemProperties.systemName);

  // the viewconfiguration views contain information like resolution about each view
  XrViewConfigurationView* viewconfig_views = NULL;

  uint32_t view_count = 0;
  XrViewConfigurationType view_type = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
  result = xrEnumerateViewConfigurationViews(instance, systemId, view_type, 0, &view_count, NULL);
  if (!xr_check(instance, result, "Failed to get view configuration view count!"))
    return 1;

  viewconfig_views = (XrViewConfigurationView*)malloc(sizeof(XrViewConfigurationView) * view_count);
  for (uint32_t i = 0; i < view_count; i++) {
    viewconfig_views[i].type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
    viewconfig_views[i].next = NULL;
  }

  result = xrEnumerateViewConfigurationViews(instance, systemId, view_type, view_count, &view_count, viewconfig_views);
  if (!xr_check(instance, result, "Failed to enumerate view configuration views!"))
    return 1;
  // print_viewconfig_view_info(view_count, viewconfig_views);

  // OpenXR requires checking graphics requirements before creating a session.
  XrGraphicsRequirementsOpenGLKHR opengl_reqs = { .type = XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR, .next = NULL };

  // this function pointer was loaded with xrGetInstanceProcAddr
  result = pfnGetOpenGLGraphicsRequirementsKHR(instance, systemId, &opengl_reqs);
  if (!xr_check(instance, result, "Failed to get OpenGL graphics requirements!"))
    return 1;

  // --- Create session
  XrGraphicsBindingOpenGLXlibKHR graphics_binding_gl = (XrGraphicsBindingOpenGLXlibKHR){
    .type = XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR,
  };

  // create SDL window the size of the left eye & fill GL graphics binding info
  if (!init_sdl_window(&graphics_binding_gl.xDisplay, &graphics_binding_gl.visualid, &graphics_binding_gl.glxFBConfig, &graphics_binding_gl.glxDrawable,
                       &graphics_binding_gl.glxContext, viewconfig_views[0].recommendedImageRectWidth, viewconfig_views[0].recommendedImageRectHeight)) {
    printf("GLX init failed!\n");
    return 1;
  }

  XrSessionCreateInfo session_create_info = { .type = XR_TYPE_SESSION_CREATE_INFO, .next = &graphics_binding_gl, .systemId = systemId };

  result = xrCreateSession(instance, &session_create_info, &session);
  if (!xr_check(instance, result, "Failed to create session"))
    return 1;

  // --- Set up input (actions)
  XrPath hand_paths[HAND_COUNT];
  xrStringToPath(instance, "/user/hand/left", &hand_paths[HAND_LEFT_INDEX]);
  xrStringToPath(instance, "/user/hand/right", &hand_paths[HAND_RIGHT_INDEX]);

  XrActionSetCreateInfo gameplay_actionset_info = { .type = XR_TYPE_ACTION_SET_CREATE_INFO, .next = NULL, .priority = 0 };
  strcpy(gameplay_actionset_info.actionSetName, "gameplay_actionset");
  strcpy(gameplay_actionset_info.localizedActionSetName, "Gameplay Actions");

  XrActionSet gameplay_actionset;
  result = xrCreateActionSet(instance, &gameplay_actionset_info, &gameplay_actionset);
  if (!xr_check(instance, result, "failed to create actionset"))
    return 1;

  XrAction poseAction;
  {
    XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO, .next = NULL, .actionType = XR_ACTION_TYPE_POSE_INPUT, .countSubactionPaths = HAND_COUNT, .subactionPaths = hand_paths
    };
    strcpy(action_info.actionName, "handpose");
    strcpy(action_info.localizedActionName, "Hand Pose");

    result = xrCreateAction(gameplay_actionset, &action_info, &poseAction);
    if (!xr_check(instance, result, "failed to create hand pose action"))
      return 1;
  }
  // poses can't be queried directly, we need to create a space for each
  XrSpace hand_pose_spaces[HAND_COUNT];
  for (int hand = 0; hand < HAND_COUNT; hand++) {
    XrActionSpaceCreateInfo action_space_info = {
      .type = XR_TYPE_ACTION_SPACE_CREATE_INFO,
      .next = NULL,
      .action = poseAction,
      .subactionPath = hand_paths[hand],
      .poseInActionSpace = identity_pose,
    };

    result = xrCreateActionSpace(session, &action_space_info, &hand_pose_spaces[hand]);
    if (!xr_check(instance, result, "failed to create hand %d pose space", hand))
      return 1;
  }

  // Grabbing objects is not actually implemented in this demo, it only gives some  haptic feebdack.
  XrAction grab_action_float;
  {
    XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO, .next = NULL, .actionType = XR_ACTION_TYPE_FLOAT_INPUT, .countSubactionPaths = HAND_COUNT, .subactionPaths = hand_paths
    };
    strcpy(action_info.actionName, "grabobjectfloat");
    strcpy(action_info.localizedActionName, "Grab Object");

    result = xrCreateAction(gameplay_actionset, &action_info, &grab_action_float);
    if (!xr_check(instance, result, "failed to create grab action"))
      return 1;
  }

  XrAction squzAction;
  {
    XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO, .next = NULL, .actionType = XR_ACTION_TYPE_FLOAT_INPUT, .countSubactionPaths = HAND_COUNT, .subactionPaths = hand_paths
    };
    strcpy(action_info.actionName, "squeezefloat");
    strcpy(action_info.localizedActionName, "Squeeze Float Value");

    result = xrCreateAction(gameplay_actionset, &action_info, &squzAction);
    if (!xr_check(instance, result, "failed to create Squeeze action"))
      return 1;
  }

  XrAction trigAction;
  {
    XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO, .next = NULL, .actionType = XR_ACTION_TYPE_FLOAT_INPUT, .countSubactionPaths = HAND_COUNT, .subactionPaths = hand_paths
    };
    strcpy(action_info.actionName, "triggerfloat");
    strcpy(action_info.localizedActionName, "Trigger Float Value");

    result = xrCreateAction(gameplay_actionset, &action_info, &trigAction);
    if (!xr_check(instance, result, "failed to create Trigger action"))
      return 1;
  }

  XrAction stikAction;
  {
    XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO, .next = NULL, .actionType = XR_ACTION_TYPE_VECTOR2F_INPUT, .countSubactionPaths = HAND_COUNT, .subactionPaths = hand_paths
    };
    strcpy(action_info.actionName, "thumbstickfloat");
    strcpy(action_info.localizedActionName, "Thumbstick V2F Value");

    result = xrCreateAction(gameplay_actionset, &action_info, &stikAction);
    if (!xr_check(instance, result, "failed to create Thumbstick action"))
      return 1;
  }

  XrAction cliAXAction;
  {
    XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO, .next = NULL, .actionType = XR_ACTION_TYPE_BOOLEAN_INPUT, .countSubactionPaths = HAND_COUNT, .subactionPaths = hand_paths
    };
    strcpy(action_info.actionName, "axbool");
    strcpy(action_info.localizedActionName, "AX Button");

    result = xrCreateAction(gameplay_actionset, &action_info, &cliAXAction);
    if (!xr_check(instance, result, "failed to create AX Button action"))
      return 1;
  }

  XrAction cliBYAction;
  {
    XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO, .next = NULL, .actionType = XR_ACTION_TYPE_BOOLEAN_INPUT, .countSubactionPaths = HAND_COUNT, .subactionPaths = hand_paths
    };
    strcpy(action_info.actionName, "bybool");
    strcpy(action_info.localizedActionName, "BY Button");

    result = xrCreateAction(gameplay_actionset, &action_info, &cliBYAction);
    if (!xr_check(instance, result, "failed to create BY Button action"))
      return 1;
  }

  XrAction vibrAction;
  {
    XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO, .next = NULL, .actionType = XR_ACTION_TYPE_VIBRATION_OUTPUT, .countSubactionPaths = HAND_COUNT, .subactionPaths = hand_paths
    };
    strcpy(action_info.actionName, "haptic");
    strcpy(action_info.localizedActionName, "Haptic Vibration");
    result = xrCreateAction(gameplay_actionset, &action_info, &vibrAction);
    if (!xr_check(instance, result, "failed to create haptic action"))
      return 1;
  }

  {
    XrPath touchInteractionProfile{ XR_NULL_PATH };
    xrStringToPath(instance, "/interaction_profiles/oculus/touch_controller", &touchInteractionProfile);

    std::vector<XrActionSuggestedBinding> bindings{};

    XrPath inputRightGripPose{ XR_NULL_PATH }, inputLeftGripPose{ XR_NULL_PATH };
    xrStringToPath(instance, "/user/hand/right/input/grip/pose", &inputRightGripPose);
    xrStringToPath(instance, "/user/hand/left/input/grip/pose", &inputLeftGripPose);
    bindings.emplace_back(XrActionSuggestedBinding{ poseAction, inputRightGripPose });
    bindings.emplace_back(XrActionSuggestedBinding{ poseAction, inputLeftGripPose });

    XrPath inputRightSqueeze{ XR_NULL_PATH }, inputLeftSqueeze{ XR_NULL_PATH };
    xrStringToPath(instance, "/user/hand/right/input/squeeze/value", &inputRightSqueeze);
    xrStringToPath(instance, "/user/hand/left/input/squeeze/value", &inputLeftSqueeze);
    bindings.emplace_back(XrActionSuggestedBinding{ squzAction, inputRightSqueeze });
    bindings.emplace_back(XrActionSuggestedBinding{ squzAction, inputLeftSqueeze });

    XrPath inputRightTrigger{ XR_NULL_PATH }, inputLeftTrigger{ XR_NULL_PATH };
    xrStringToPath(instance, "/user/hand/right/input/trigger/value", &inputRightTrigger);
    xrStringToPath(instance, "/user/hand/left/input/trigger/value", &inputLeftTrigger);
    bindings.emplace_back(XrActionSuggestedBinding{ trigAction, inputRightTrigger });
    bindings.emplace_back(XrActionSuggestedBinding{ trigAction, inputLeftTrigger });

    XrPath inputRightThumbstick{ XR_NULL_PATH }, inputLeftThumbstick{ XR_NULL_PATH };
    xrStringToPath(instance, "/user/hand/right/input/thumbstick", &inputRightThumbstick);
    xrStringToPath(instance, "/user/hand/left/input/thumbstick", &inputLeftThumbstick);
    bindings.emplace_back(XrActionSuggestedBinding{ stikAction, inputRightThumbstick });
    bindings.emplace_back(XrActionSuggestedBinding{ stikAction, inputLeftThumbstick });

    XrPath inputRightButtonAX{ XR_NULL_PATH }, inputLeftButtonAX{ XR_NULL_PATH };
    xrStringToPath(instance, "/user/hand/right/input/a/click", &inputRightButtonAX);
    xrStringToPath(instance, "/user/hand/left/input/x/click", &inputLeftButtonAX);
    bindings.emplace_back(XrActionSuggestedBinding{ cliAXAction, inputRightButtonAX });
    bindings.emplace_back(XrActionSuggestedBinding{ cliAXAction, inputLeftButtonAX });

    XrPath inputRightButtonBY{ XR_NULL_PATH }, inputLeftButtonBY{ XR_NULL_PATH };
    xrStringToPath(instance, "/user/hand/right/input/b/click", &inputRightButtonBY);
    xrStringToPath(instance, "/user/hand/left/input/y/click", &inputLeftButtonBY);
    bindings.emplace_back(XrActionSuggestedBinding{ cliBYAction, inputRightButtonBY });
    bindings.emplace_back(XrActionSuggestedBinding{ cliBYAction, inputLeftButtonBY });

    XrPath outputRightHaptic{ XR_NULL_PATH }, outputLeftHaptic{ XR_NULL_PATH };
    xrStringToPath(instance, "/user/hand/right/output/haptic", &outputRightHaptic);
    xrStringToPath(instance, "/user/hand/left/output/haptic", &outputLeftHaptic);
    bindings.emplace_back(XrActionSuggestedBinding{ vibrAction, outputRightHaptic });
    bindings.emplace_back(XrActionSuggestedBinding{ vibrAction, outputLeftHaptic });

    XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    // These bindings are for the Meta Quest Touch interaction profile
    suggestedBindings.interactionProfile = touchInteractionProfile;
    suggestedBindings.suggestedBindings = bindings.data();
    suggestedBindings.countSuggestedBindings = bindings.size();

    // Suggest all the bindings for the Meta Quest Touch interaction profile
    xrSuggestInteractionProfileBindings(instance, &suggestedBindings);
  }

  XrReferenceSpaceType play_space_type = XR_REFERENCE_SPACE_TYPE_LOCAL;
  XrSpace play_space = XR_NULL_HANDLE;
  XrReferenceSpaceCreateInfo play_space_create_info = {
    .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO, .next = NULL, .referenceSpaceType = play_space_type, .poseInReferenceSpace = identity_pose
  };

  result = xrCreateReferenceSpace(session, &play_space_create_info, &play_space);
  if (!xr_check(instance, result, "Failed to create play space!"))
    return 1;

  while (ros::ok()) {
    XrEventDataBuffer runtime_event = { .type = XR_TYPE_EVENT_DATA_BUFFER, .next = NULL };
    XrResult poll_result = xrPollEvent(instance, &runtime_event);

    XrFrameState frame_state = { .type = XR_TYPE_FRAME_STATE, .next = NULL };
    XrFrameWaitInfo frame_wait_info = { .type = XR_TYPE_FRAME_WAIT_INFO, .next = NULL };
    result = xrWaitFrame(session, &frame_wait_info, &frame_state);
    if (!xr_check(instance, result, "xrWaitFrame() was not successful, exiting..."))
      break;

    // query each value / location with a subaction path != XR_NULL_PATH
    // resulting in individual values per hand/.
    XrActionStateFloat squeeze_value[HAND_COUNT], trigger_value[HAND_COUNT];
    XrSpaceVelocity hand_velocities[HAND_COUNT]{ XR_TYPE_SPACE_VELOCITY };
    XrSpaceLocation hand_locations[HAND_COUNT]{ XR_TYPE_SPACE_LOCATION, &hand_velocities };
    XrActionStateVector2f thumbstick_values[HAND_COUNT];
    XrActionStateBoolean AX_value[HAND_COUNT], BY_value[HAND_COUNT];

    static ohrc_msgs::BodyState prev_bodyMsg;
    ohrc_msgs::BodyState bodyMsg;
    static bool isFirst = true;

    for (int i = 0; i < HAND_COUNT; i++) {
      XrActionStatePose hand_pose_state = { .type = XR_TYPE_ACTION_STATE_POSE, .next = NULL };
      {
        XrActionStateGetInfo get_info = { .type = XR_TYPE_ACTION_STATE_GET_INFO, .next = NULL, .action = poseAction, .subactionPath = hand_paths[i] };
        result = xrGetActionStatePose(session, &get_info, &hand_pose_state);
        xr_check(instance, result, "failed to get pose value!");
      }
      // printf("Hand pose %d active: %d\n", i, poseState.isActive);

      hand_locations[i].type = XR_TYPE_SPACE_LOCATION;
      hand_locations[i].next = NULL;

      result = xrLocateSpace(hand_pose_spaces[i], play_space, frame_state.predictedDisplayTime, &hand_locations[i]);
      xr_check(instance, result, "failed to locate space %d!", i);

      geometry_msgs::Pose hand_pose;
      hand_pose.position.x = -hand_locations[i].pose.position.z;
      hand_pose.position.y = -hand_locations[i].pose.position.x;
      hand_pose.position.z = hand_locations[i].pose.position.y;
      hand_pose.orientation.x = -hand_locations[i].pose.orientation.z;
      hand_pose.orientation.y = -hand_locations[i].pose.orientation.x;
      hand_pose.orientation.z = hand_locations[i].pose.orientation.y;
      hand_pose.orientation.w = hand_locations[i].pose.orientation.w;

      geometry_msgs::Twist hand_twist;
      hand_twist.linear.x = -hand_velocities[i].linearVelocity.z;
      hand_twist.linear.y = -hand_velocities[i].linearVelocity.x;
      hand_twist.linear.z = hand_velocities[i].linearVelocity.y;
      hand_twist.angular.x = -hand_velocities[i].angularVelocity.z;
      hand_twist.angular.y = -hand_velocities[i].angularVelocity.x;
      hand_twist.angular.z = hand_velocities[i].angularVelocity.y;

      if (i == HAND_LEFT_INDEX)
        bodyMsg.left_hand.pose = hand_pose;
      else if (i == HAND_RIGHT_INDEX)
        bodyMsg.right_hand.pose = hand_pose;

      squeeze_value[i].type = XR_TYPE_ACTION_STATE_FLOAT;
      squeeze_value[i].next = NULL;
      {
        XrActionStateGetInfo get_info = { .type = XR_TYPE_ACTION_STATE_GET_INFO, .next = NULL, .action = squzAction, .subactionPath = hand_paths[i] };

        result = xrGetActionStateFloat(session, &get_info, &squeeze_value[i]);
        xr_check(instance, result, "failed to get squeeze value!");
      }

      if (i == HAND_LEFT_INDEX)
        bodyMsg.left_hand.squeeze = squeeze_value[i].currentState;
      else if (i == HAND_RIGHT_INDEX)
        bodyMsg.right_hand.squeeze = squeeze_value[i].currentState;

      trigger_value[i].type = XR_TYPE_ACTION_STATE_FLOAT;
      trigger_value[i].next = NULL;
      {
        XrActionStateGetInfo get_info = { .type = XR_TYPE_ACTION_STATE_GET_INFO, .next = NULL, .action = trigAction, .subactionPath = hand_paths[i] };

        result = xrGetActionStateFloat(session, &get_info, &trigger_value[i]);
        xr_check(instance, result, "failed to get trigger value!");
      }

      if (i == HAND_LEFT_INDEX)
        bodyMsg.left_hand.trigger = trigger_value[i].currentState;
      else if (i == HAND_RIGHT_INDEX)
        bodyMsg.right_hand.trigger = trigger_value[i].currentState;

      thumbstick_values[i].type = XR_TYPE_ACTION_STATE_VECTOR2F;
      thumbstick_values[i].next = NULL;
      {
        XrActionStateGetInfo get_info = { .type = XR_TYPE_ACTION_STATE_GET_INFO, .next = NULL, .action = stikAction, .subactionPath = hand_paths[i] };

        result = xrGetActionStateVector2f(session, &get_info, &thumbstick_values[i]);
        xr_check(instance, result, "failed to get trigger value!");
      }
      std::vector<float> axes(HAND_COUNT);
      axes[0] = thumbstick_values[i].currentState.y;
      axes[1] = -thumbstick_values[i].currentState.x;

      if (i == HAND_LEFT_INDEX)
        bodyMsg.left_hand.stick = axes;
      else if (i == HAND_RIGHT_INDEX)
        bodyMsg.right_hand.stick = axes;

      AX_value[i].type = XR_TYPE_ACTION_STATE_VECTOR2F;
      AX_value[i].next = NULL;
      {
        XrActionStateGetInfo get_info = { .type = XR_TYPE_ACTION_STATE_GET_INFO, .next = NULL, .action = cliAXAction, .subactionPath = hand_paths[i] };

        result = xrGetActionStateBoolean(session, &get_info, &AX_value[i]);
        xr_check(instance, result, "failed to get trigger value!");
      }
      if (i == HAND_LEFT_INDEX)
        bodyMsg.left_hand.button.push_back(AX_value[i].currentState);
      else if (i == HAND_RIGHT_INDEX)
        bodyMsg.right_hand.button.push_back(AX_value[i].currentState);

      BY_value[i].type = XR_TYPE_ACTION_STATE_VECTOR2F;
      BY_value[i].next = NULL;
      {
        XrActionStateGetInfo get_info = { .type = XR_TYPE_ACTION_STATE_GET_INFO, .next = NULL, .action = cliBYAction, .subactionPath = hand_paths[i] };

        result = xrGetActionStateBoolean(session, &get_info, &BY_value[i]);
        xr_check(instance, result, "failed to get trigger value!");
      }
      if (i == HAND_LEFT_INDEX)
        bodyMsg.left_hand.button.push_back(BY_value[i].currentState);
      else if (i == HAND_RIGHT_INDEX)
        bodyMsg.right_hand.button.push_back(BY_value[i].currentState);

      // printf("Grab %d active %d, current %f, changed %d\n", i,
      // grabValue[i].isActive, grabValue[i].currentState,
      // grabValue[i].changedSinceLastSync);

      {
        XrHapticVibration vibration = {
          .type = XR_TYPE_HAPTIC_VIBRATION,
          .next = NULL,
          .duration = XR_MIN_HAPTIC_DURATION,
          .frequency = XR_FREQUENCY_UNSPECIFIED,
          .amplitude = _haptic[i],
        };
        XrHapticActionInfo haptic_action_info = { .type = XR_TYPE_HAPTIC_ACTION_INFO, .next = NULL, .action = vibrAction, .subactionPath = hand_paths[i] };
        result = xrApplyHapticFeedback(session, &haptic_action_info, (const XrHapticBaseHeader*)&vibration);
        xr_check(instance, result, "failed to apply haptic feedback!");
        // printf("Sent haptic output to hand %d\n", i);
      }
    };

    if (isFirst) {
      prev_bodyMsg = bodyMsg;
      isFirst = false;
    } else {
      bodyMsg.right_hand.twist.linear.x = (bodyMsg.right_hand.pose.position.x - prev_bodyMsg.right_hand.pose.position.x) * 60.0;
      bodyMsg.right_hand.twist.linear.y = (bodyMsg.right_hand.pose.position.y - prev_bodyMsg.right_hand.pose.position.y) * 60.0;
      bodyMsg.right_hand.twist.linear.z = (bodyMsg.right_hand.pose.position.z - prev_bodyMsg.right_hand.pose.position.z) * 60.0;
      bodyMsg.left_hand.twist.linear.x = (bodyMsg.left_hand.pose.position.x - prev_bodyMsg.left_hand.pose.position.x) * 60.0;
      bodyMsg.left_hand.twist.linear.y = (bodyMsg.left_hand.pose.position.y - prev_bodyMsg.left_hand.pose.position.y) * 60.0;
      bodyMsg.left_hand.twist.linear.z = (bodyMsg.left_hand.pose.position.z - prev_bodyMsg.left_hand.pose.position.z) * 60.0;

      prev_bodyMsg = bodyMsg;
    }

    publisher.publish(bodyMsg);
    ros::spinOnce();
  }

  // Clean up
  xrDestroyInstance(instance);

  return 0;
}