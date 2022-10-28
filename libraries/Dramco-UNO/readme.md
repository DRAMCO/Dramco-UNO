# How to use in Visual Studio Code + PlatformIO

1. Copy Dramco-UNO library folder under the lib directory of your PlatformIO project.
2. Add the build_flags from `Dramco-UNO/boards.txt` to your platform.ini file. Example (can change, always check the file:
```cmake
build_flags =
    -DARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS 
    -DCFG_eu868=1 -DCFG_sx1276_radio=1 
    -DUSE_IDEETRON_AES 
    -DDISABLE_BEACONS 
    -DDISABLE_PING  
    -DLMIC_MAX_FRAME_LENGTH=35 
    -D__AVR -DCFG_noassert 
    -DLMIC_ENABLE_DeviceTimeReq=0
 ```

