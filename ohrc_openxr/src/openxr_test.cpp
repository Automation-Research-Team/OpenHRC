#include <openxr/openxr.h>

#include <cstdio>
#include <cstring>

int main() {
  XrInstance instance;
  XrInstanceCreateInfo createInfo = { XR_TYPE_INSTANCE_CREATE_INFO };
  createInfo.enabledApiLayerCount = 0;
  createInfo.enabledExtensionCount = 0;

  // Set the application info, including the engineName
  XrApplicationInfo appInfo;
  appInfo.applicationVersion = XR_MAKE_VERSION(1, 0, 0);
  appInfo.engineVersion = XR_MAKE_VERSION(1, 0, 0);
  appInfo.apiVersion = XR_CURRENT_API_VERSION;
  std::strcpy(appInfo.applicationName, "My OpenXR App");
  std::strcpy(appInfo.engineName, "My OpenXR Engine");  // <-- Set the engineName here
  createInfo.applicationInfo = appInfo;

  XrResult result = xrCreateInstance(&createInfo, &instance);
  if (result != XR_SUCCESS) {
    std::printf("Failed to create OpenXR instance: %d\n", result);
    return 1;
  }

  // Get the system properties
  XrSystemGetInfo systemGetInfo = { XR_TYPE_SYSTEM_GET_INFO };
  systemGetInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
  XrSystemId systemId;
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

  // Clean up
  xrDestroyInstance(instance);

  return 0;
}