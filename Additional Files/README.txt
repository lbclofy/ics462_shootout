ICS 462, AI for Games
Assignment 3 Solution
Includes a modified version of Millington's AI for Games's decision tree code (see LICENSE file for the use license for that original code)

The current version of BZFlag does not allow robots to pick up/drop flags. Download the fixes in robotFlags.zip ( http://www2.hawaii.edu/~chin/462/Assignments/robotFlags.zip ), unzip it and replace the corresponding files in your src and include directory with these files. You will have to quit bzfs and rebuild it. Likewise with bzflag.

This solution uses the YAGSBPL ( http://code.google.com/p/yagsbpl/ ) C++ template implementation of A* search.  Download yagsbpl-v2.1.zip and unzip it to your choice of location.  Then in Microsoft Visual Studio, right click on the bzflag project, select Properties and add the yagsbpl folder as a new entry to "Additional Include Directories" under Configuration Properties, C/C++, General.  Replace yagsbpl-v2.1\yagsbpl\yagsbpl_basic.h and yagsbpl-v2.1\yagsbpl\yagsbpl_basic.cpp with the yagsbpl_basic.h and yagsbpl_basic.cpp in this folder to avoid MS VS errors about some of the YAGSBPL virtual methods failing to return values and an error in the priority heap code that only shows up in Windows.  Also replace yagsbpl-v2.1\yagsbpl\planners\A_star.cpp with the version in this folder to print out an errror message to the controlPanel rather than stdout and to not call exit(1).

To compile, copy the files dectree.cxx, dectree.h, RobotPlayer.h, RobotPlayer.cxx, AStarNode.h, playing.cxx and AStarNode.cpp to your bzflags-2.4.2\src\bzflag folder (overwriting the originals of RobotPlayer.h and RobotPlayer.cxx).  Then add AStarNode.h to the bzflag project by right clicking on "Header FIles" under bzflag, selecting "Add > Existing Item" and then find AStarNode.h in your src\bzflag folder.  Likewise add AStarNode.cxx to the bzflag project by right clicking on game under "Source Files" under the bzflag project, selecting "Add > Existing Item", then find AStarNode.cxx in your src\bzflag folder. Then build bzflag in Microsoft Visual Studio as usual.

For UNIX-based systems copy the same files to your bzflags-2.4.2/src/bzflag directory.  You will have to edit src/bzflag/Makefile.am to add dectree.h, dectree.cxx AStarNode.h and AStarNode.cxx to bzflag_SOURCES and add the full path to yagsbpl to src/bzflag/Makefile.am by adding it as a -I argument to the AM_CPPFLAGS line:

AM_CPPFLAGS += $(SDL_CFLAGS) -I /full-path-to/yagsbpl

Depending on your OS and compiler, you may have to convert the line endings using a program like dos2unix. Next run autogen.sh and configure again.  Finally compile as usual.