# Getting started with the Tundrabots git repository

This document describes the steps needed to install Android Stuido and configure it to build the software used to control the robot.

## Create a GitHub account

The source code for Tundrabots resides in a `git repo`.  Git is, among other things, a souroce code management system, which is the main feature that Tundrabots uses. Within git, there are multiple `repositories`, often called `repos` for short. In order to access these git repos, you first need a GitHub account.

To create a GitHub account, go to [GitHub](https://github.com/) and click on `Signup` in the upper right. To signup, you will require an email account; if you don't already have one, you can create a gmail account by visiting [Google Sign in](https://accounts.google.com/) and creating a new account. You then enter your email account information into the prompt on GitHub and step through the process of creating the account.

## Getting access to the Tundrabots GitHub repo

Once you have your GitHub account, you should request one of the Software Mentors to add you as a `member` of the Tundrabots team in GitHub. This will allow you to be able to use the Git repository in the future. When youare are added to the Tundrabots GitHub team, an invitation will be sent to your GitHub account, which you should accept. Once done, you will be able to access the software repositories for the Tundrabots. For 2023-2024 it is [7083-2023-2024](https://github.com/Tundrabots7083/7083-2023-2024).

## Install Android Studio

In a web browser, open the site [Download Android Stuido and App Code](https://developer.android.com/studio). From here, select the latest version of Android Studio, which appears toward the top of the page. Before you download the installation file, you must agree to the license agreement for Android Studio. Once you have read through the license agreement and are ready to agree to the terms and conditions, check the box next to "I have read and agree with the above terms and conditions" and download the version of Android Studio for your platform.

Once the download completes, which can take some time as it is a large download, run the installation program. This will step you through the steps to install Android Studio. There are several licenses within Andriod Studio you must read and agree to. Once you have agreed to all licenses click `Finish`.

At ths point, Android Studio is installed.

## Configure Android Studio for git access

To access the git repository from within Android Studio, you will need to perform two steps:

- Install the git command line tools. These are used by Andriod Studio when communicating with GitHub.

- Add your GitHub account to Android Studio.

Both of these are configured in the `Settings` page (for Windows) or `Preferences` page (for MacOS). To open this, go to the `gear` in the upper right corner of the Android Studio page, click on it, and select `Settings` or `Preferences`, depending on your platform.

To install the git command line tools, you should navigate to `Version Control -> Git`. If the tools are already installed, Android Studio will point to the auto-detected git command. If this is not the case, `Test` button to the right, and then click on the `install` link on below the dialog and the tools will be installed and automatically configured for use by Android Studio.

To add your GitHub account to Android Studio, navigate to `Version Control -> GitHub`. If you are doing this right after configuring Git, it is the option directly below Git. Add an account and select `Log in with Token`. Click on `Generate` on the right-hand side of the panel, which will open a browser window. If you aren't already logged into GitHub, do so.

You will be taken to a page called `New personal access token classic` in the borowser. Change the expiration date from 30 days to `No Expiration`, and ignore the warnings. Scroll down to the bottom and click on `Generate token`. Once the token is created, copy it, go back to Android Studio, paste it into the `Token` option, and click on `Add Account`. You should see your GitHub account now being added.

## Clone the PowerPlayRobotCode git repo to your local device

The next step is to clone the git repo that contains the Tundrabots code to your local device. This creates a `copy` of the code that you can access. Later, you can use Android Studio or the git command to refresh the code (that is, pull down any new changes from GitHub to your local machine), or push your code changes from your local device to the GitHub repo.

To clone the repo, go to [PowerPlayRobotCode](https://github.com/Tundrabots7083/PowerPlayRobotCode) in a web browser. On the upper right-hand side of the page, there is a green `Code` button. Click on the two squares next to the line that starts `https://github.com/Tundrabots7083/PowerPlayRobotCode.git`, which will copy this to your clipboard.

In Android Studio, Select `Get from VCS`, click on `GitHub`, which will be your GitHub account previously adde. If the PowerCodeRobotCode isn't already present, paste the URL you copied for the clone operation into the URL; otherwise, select it. Select `Clone` and the repository will be cloned to your local device and loaded into Android Studio.

Once you have created the project, in the future you just opt to open the PowerPlayRobotCode project when starting Android Studio. On Windows devices, it may automatically open, unless you `File -> Close Project` from the file menu.

## Building the project

You can click on `Build -> Make Project` to compile the code for the project. This is a good way to verify that the setup is correct. The first time you do a build, it can take a while, but subsequent builds will be quicker as only the changed parts will need to be re-built.

After making the project, you can build the APK by selecting `Build -> Build Bundle(s)/APK(s) -> Build APK(s)`. The APK file contains the code needed to install `PowerPlayRobotCode` on the Android phone that drives the robot.

## Create your own branch in git

In git, changes are made in a `branch`. A branch diverges from the main line of development and allows you to continue to do work without messing with the main line. Once the code within the branch is completed and thoroughly tested, it can be merged back into the main line for others to use. Git supports multiple simultanous branches, allowing different developers to work independenly of each other. Developers can also use the same branch for collaberative development projects.

Code changes are made within a branch, and are then `committed` to the branch. The `commit` adds the code to the given branch, with the code residing on the local device. These commits can be `pushed` to GitHub, where the changes will then be visable to all developers who are working on a given branch.

Once the code development is completed, a `pull request` can be issued against the committed code in the branch. The pull request takes the code from the commited branch and merges it into the `master` branch.

While it is theoretically possible to develop code in the master branch and push commits to this branch, it is generally considered a bad development practice. For this reason, always create or use a branch for development, push the code commits to the branch on GitHub, and then create a pull request. One of the maintainers for the git repository will then be able to merge the code into the master branch.

## Connecting to the robot

### Setup external tools to allow connecting to the Control Hub

Two external tools are used to connect over wifi to the Control Hub: `Enable ADB over TCP` and `Connect to Control Hub over WiFi`.

Navigate to the `Settings` pannel in Android Studio by clicking on the gear icon in the upper right hand side of the screen and selecting `Settings...` (for Windows) or `Preferences...` (for MacOS). From here, open up the `Tools` option and then select `External Tools`. You won't have any present, but we will be adding two.

`Enable ADB over TCP` configures your Android Studio to be able to connect to the Control Hub over wifi. You will only need to run this command once, and it must be run while connected to the robot via a USB cable while Control Hub is powered on and the Driver Station is connected to the Control Hub. To add it, while in the `External Tools` panel for settings, click on the `+` icon. This will open up a pannel. Fill in the data with the following:

- **Name**: set to `Enable ADB over TCP`
- **Program**: set to `$ModuleSdkPath$/platform-tools/adb`
- **Arguments**: set to `tcpip 5555`
- **Working directory**: set to `$ProjectFileDir$`

`Connect to Control Hub over WiFi` is run each time you need to connect to the Control Hub and you aren't already connected. The external tool configuration is similar to `Enable ADB over TCP`, with only the arguments being different. Fill in the data with the following:

- **Name**: set to `Connect to Control Hub over WiFi`
- **Program**: set to `$ModuleSdkPath$/platform-tools/adb`
- **Arguments**: set to `connect 192.168.43.1:5555`
- **Working directory**: set to `$ProjectFileDir$`

Make sure you click `OK` when creating the external tool, as well as when you are done creating the two external tools.

On some Android Studio platforms, `$ModuleSdkPath$` will not be properly set. In that case, you must set absolute path manually and not use the `$ModuleSdkPath$` macro.  For Windows, this will be `C:\Users\username\AppData\Local\Android\Sdk\platform-tools\adb`. For MacOS, this will be `/Users/username/Library/Android/sdk/platform-tools/adb`. In both cases, replace `username` with your the name of your home directory.

To test if `$ModuleSdkPath$` is properly set on your device, try running the `Enable ADB over TCP` external tool. On the toolbar, select `Tools -> External Tools -> Enable ADB over TCP`. You should see a `run` window appear on the botton of the screen, and if the path works you will see an error message stating `adb: no devices/emulators found`.  If you see a popup saying the tool could not be found, then go back to the external tools, edit them by clicking on the tool and selecting the `pencil` icon, and change the path to an absolute path as previously described.  Then re-run the command, and now you should see the message `adb: no devices/emulators found` in the run output window.

### Connect to the robot

Now that the two commands exist and wait for the robot to start and the Driver Station to connect to the robot. If the Driver Station doesn't automatically connect, then you may need to manually enable the wifi network on the Driver Station.

Once both are connected, disable wifi on your device. Then connect the robot to your device using a USB cable. The USBC end should be connected to the robot, and the USBA end should be connected to your device. Wait until the name of the robot appears in the device list, which appear in the drop-down menu directly to the right of the one with `TeamCode` at the top of Android Studio. The name should be `REV Robotics Control Hub v1.0`; if you don't see this, then ask one of the software mentors to assist you.

At this point, you need to run the `Enable ADB over TCP` tool. On the toolbar, select `Tools -> External Tools -> Enable ADB over TCP`. If successful, you will see an exit code of 0. Once the command runs successfully, disconnect the USB cable from the robot and your device. You will now need to get the wifi information for your Control Hub so that you can connect. 

You will need the wifi network and password that were setup when configuring the Control Hub. These can be found on the Driver Station by following these steps:

- Clock on the three dots on the top right-hand of the Driver Station

- Select `Program & Manage` from the drop-down menu

- The wireless network name is the `ssid`, or network name, for the wifi network. The passphrase for the network is the password.

Go to your wifi settings, re-enable wifi, and connect to the wifi network using the name retrieved from the Driver Station. I recommend making sure you have your device to **not** auto-connect to this network. When prompted, enter the password retrieved from the Driver Station.

Once connected to the wifi network, run the external tool `Connect to Control Hub over WiFi`. If this completes successfully, you will see the robot device appear with the same device name as before (`REV Robotics Control Hub`). At this point, you are now connected to the robot over wifi.

## Building and installing code on the robot

On the top bar in Android Studio, make sure hou have selected the `REV Robotics Control Hub v1.0` device. Directly to the right of this drop-down menu is a small green triangle that represents `Play` or `Run`.  Press this green button to build the RobotController app and install it on your Control Hub.

Although the Control Hub lacks a built in screen, you can verify that the app was installed onto your Control Hub properly by looking at your Driver Station. If the Driver Station indicates that it is successfully connected to the Control Hub (after momentarily disconnecting while the update was occurring) then the app was successfully updated.
