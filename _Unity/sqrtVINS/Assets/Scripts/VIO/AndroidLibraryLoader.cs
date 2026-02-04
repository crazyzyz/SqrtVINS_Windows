using UnityEngine;

namespace SqrtVINS
{
    /// <summary>
    /// 强制加载原生库，解决依赖顺序问题
    /// </summary>
    public static class AndroidLibraryLoader
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void LoadLibraries()
        {
            if (Application.platform == RuntimePlatform.Android)
            {
                Debug.Log("[AndroidLibraryLoader] Attempting to load native libraries...");
                try
                {
                    using (var system = new AndroidJavaClass("java.lang.System"))
                    {
                        // 1. 先加载 OpenCV (libopencv_java4.so)
                        try 
                        {
                            system.CallStatic("loadLibrary", "opencv_java4");
                            Debug.Log("[AndroidLibraryLoader] Successfully loaded 'opencv_java4'");
                        }
                        catch (System.Exception ex)
                        {
                            Debug.LogError($"[AndroidLibraryLoader] Failed to load 'opencv_java4': {ex.Message}");
                        }

                        // 2. 再加载 VO 库 (libvo_unity.so)
                        try
                        {
                            system.CallStatic("loadLibrary", "vo_unity");
                            Debug.Log("[AndroidLibraryLoader] Successfully loaded 'vo_unity'");
                        }
                        catch (System.Exception ex)
                        {
                            Debug.LogError($"[AndroidLibraryLoader] Failed to load 'vo_unity': {ex.Message}");
                        }
                    }
                }
                catch (System.Exception e)
                {
                    Debug.LogError($"[AndroidLibraryLoader] Critical error accessing Java System: {e.Message}");
                }
            }
        }
    }
}
