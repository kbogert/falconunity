falconunity
===========

Library that allows for easy manipulation of Unity3D objects using a Novint Falcon.

For an example, see [here](http://www.screenr.com/baP7)


Quick Setup
-----------
- Install the Novint Falcon on your machine, make sure it works
  - Driver Download: https://www.novint.com/index.php/downloads
- Install the Visual Studio 2013 runtime from https://www.microsoft.com/en-us/download/details.aspx?id=30679
- Look in the releases section of this project, attached to the release is a binary build containing 3 files:
  - falconunity.dll
  - FalconServer.exe
  - FalconUnity.unitypackage
- Extract these files into the same folder
- Import the unitypackage into your project
- Drag the falcon prefab into your scene
- Run FalconServer
- Play your Unity project


Troubleshooting
-----------
- If the frame rate of the physics simulation drops below 500fps (and especially below 120) the falcon will become very jerky
  - Try simplifying your scene, or else run FalconServer on a more powerful machine


Using without the server
-----------
- If you have a Pro license for Unity you can use falconunity as a plugin
- Edit Assets/novint/FalconUnity.cs (in the unitypackage)
  - Comment out the first line
- Make sure that falconunity.dll is in the plugins folder of your project


Notes for Building
-----------
- This project was/is built with Visual Studio 2013
- Clone the repository to c:\ (sorry :( )
- Download the [Bullet library](http://bulletphysics.org/wordpress/) and place in c:\falconunity\external_libs\bullet
  - Apply the fix here: http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=8915
- Download [Boost](http://www.boost.org/users/download/) and place in c:\falconunity\external_libs\boost_1_58_0
- Build the solution that's in c:\falconunity\falconunity_vs2010, the falconunity.dll and FalconServer.exe files will be placed in c:\falconunity\build\Release
