This folder includes the source code of the Oculus Quest APK delivered in the repository. Use it if you intend to customize the app. Before following this instruction, please read the [../README.md](README.md) in the repository root folder.

In order to compile the code you need to download the Oculus Quest SDK ([link](https://developer.oculus.com/downloads/package/oculus-mobile-sdk/)) and **place this repository in the root folder of the unpacked SDK**. Please keep the repository name as 'oculus_reader' to conform to the predefined project configuration.

## Development

### Preparation

- (Mac only) Install Xcode
- Install Android development studio. On Ubuntu you can run `sudo snap install android-studio --classic`, otherwise follow: <https://developer.android.com/studio> 
- After Android studio is installed, go to *Configure* and select *SDK Manager...*:  
    ![image1](https://user-images.githubusercontent.com/14967831/106551452-244b0600-64ca-11eb-8525-0bd447fb6ea8.png)  
 Under SDK Tools tab select and install NDK
- (Windows only) Install drivers https://developer.oculus.com/downloads/package/oculus-adb-drivers/ 
- Configure NDK path. Go to configure and select *Default project structure...*

### Open the Android project for development

- Select "Open an Exisiting Project", find the repository path and open: app_source/Projects/Android/build.gradle
- The source code resides in the 'app_source/Src' folder.

## Run the code

Follow the instruction in [../scripts/README.md](scripts/README.md) to install the required packages and then follow the [../README.md](README.md) in the root folder to run the code.