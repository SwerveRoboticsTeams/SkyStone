
1. Connect the phone with USB cable to host running Android Studio
2. If there have been past issues, reset ADB server: adb kill-server
3. adb tcpip 5555
4. Find the phone IP address and check that it is ready for ADB: adb shell netcfg
5. adb connect xx.xx.xx.xx (such as 192.168.1.55)
6. Disconnect the USB cable and start using wireless communication