# Introduction

EnderCV is an OpenCV 3 package for the FIRST Tech Challenge that's designed to be relatively simple, and with an easier and more reliable setup.

# Installation
Installation is simple; the instructions below add the EnderCV files as Gradle dependencies of your FTC robot controller app.

1. Download this repo, either by cloning from Git or using the zip download. 
2. Pull up Android Studio, with the FTC application SDK open
3. Navigate to **File** -> **New** -> **Import Module** from the title bar.
4. When the a dialog comes up, asking for the module source directory, navigate to this repo and select the **openCVLibrary3** folder, and then hit **Finish**
5. Repeat steps 3 and 4 except instead of selecting the **openCVLibrary3** folder, select the **endercv** folder instead. If Android Studio fails to import modules because it sucks sometimes, open `settings.gradle` and add these two lines: 
```
include ':openCVLibrary3'
include ':endercv'
```
and resync the project. 
6. In the left hand side project explorer in Android Studio, right-click **TeamCode**, and click on **Open Module Settings**.
7. A **Project Struture** dialog should come up. Click the **Dependencies** tab.
8. Click the green plus sign on the right hand side, then **Module dependency**, and then **:openCVLibrary3**, then press OK.
9. Repeat step 8, except substitute **:openCVLibrary3** with **:endercv**.
10. Click **OK** to exit the **Project Structure** dialog.

And that's it! 

# Usage
The examples folder has an example of an OpenCVPipeline class and an opmode using the class, which is a nice illustrative example. That particular example shows how OpenCV can be used to filter for a certain color, as a first step for searching and analysing targets (like goal or game elements). It's been improved to be more useful for the average user.


# Updating from 1.0:
Here's a semi-hackish way to do it:

1. Close Android Studio. Download the latest version of EnderCV either from a `git pull` or using the zip download.
2. Navigate to the folder for the robot controller app project (the ftc\_app folder) where EnderCV is installed.
3. Delete the **openCVLibrary320** and **endercv** folders
4. Go into **settings.gradle** with a text editor and delete the `include ':endercv'` and `include ':openCVLibrary320'` lines
5. Restart Android Studio and follow installation steps 3-10.

# (New in 2.0) Loading libopencv\_java3.so from the phone's internal storage.
Doing this can cut down on compile times as the OpenCV native library is now extermal to the robot controller app and is not copied on every deploy.

1. Make a directory on the phone's internal storage named `EnderCV`
2. Copy `src/main/jniLibs/armeabi-v7a/libopencv_java3.so` to the `EnderCV` folder on the phone's internal storage.
3. Delete `[path to ftc_app]/endercv/src/main/jniLibs/armeabi-v7a/libopencv_java3.so` to remove it from the FTC robot controller app. Readd the file to load libopencv from the RC app itself.

(If OpenCV fails to load, you may have to edit OpenCVLoader.java to fix a hardcoded path in there idk this probably happens with Samsung S5 phones)

# Resources
OpenCV with Java is not an easy thing to get around. However, there are some resources on the net from which one can do research and find help:

[GRIP](http://wpiroboticsprojects.github.io/GRIP) is a GUI tool to prototype vision pipelines, and can even export C++ or Java code. While the generated code is not the most readable, and the software is more FRC oriented, it is useful for showing how certain OpenCV functions work, as well as tweaking parameters.

[This webpage meant for FRC](https://wpilib.screenstepslive.com/s/4485/m/24194/l/288985-identifying-and-processing-the-targets) gives a decent overview on how to identify targets with vision in general.

The unofficial [FTC Discord](https://discord.gg/c3DpWbR) and [FRC Discord](https://discord.gg/frc) can also be useful as many people in the #programming of both servers are also working on vision. 

Finally, search around and look at OpenCV documentation and examples. Object detection and tracking are widely covered subjects in many online tutorials. Some tweaking may be required to get things working in Java. As Java samples can be hard to find, consider looking at C++ samples, as those are often similar to how the Java bindings work, moreso than Python.

# Changelog:
2.0:
 - Now works with portrait mode
 - Updated OpenCV to 3.4.3
 - Now allows loading the OpenCV .so from the internal storage
 - Updated the example so its a lot more useful

1.0: 
 - Initial release
