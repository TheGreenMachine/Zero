# Zero
Zero is FRC Team 1816's robot for the 2023 FRC season, *FIRST Charged Up*. This software uses Java 11, the [WPILib](https://github.com/wpilibsuite/allwpilib) library, and the [CTRE Phoenix](https://github.com/CrossTheRoadElec/Phoenix-api) library.

## Prerequisites
1. You must have Java 17 installed on your system. You can [download OpenJDK 17 here](https://openjdk.java.net/projects/jdk/17/) or [download Oracle's JDK 11 here](https://www.oracle.com/technetwork/java/javase/downloads/jdk11-downloads-5066655.html). Alternatively, this will be installed through the WPILib One-Click Installer.
2. You must have an IDE of your choice installed. [Visual Studio Code](https://code.visualstudio.com/) or [IntelliJ IDEA](https://www.jetbrains.com/idea/) are recommended as official and unofficial options. VSCode can be optionally installed using the WPILib One-Click Installer.
3. You must have [Git](https://git-scm.com/) installed.
4. It is recommended that you run the [WPILib One-Click Installer](https://github.com/wpilibsuite/allwpilib/releases). This includes the VSCode plugins as well as all NI software tools. It does **NOT** include any CTRE software tools.

## Cloning

Open a new Bash shell and clone:
```bash
$ git clone https://github.com/TheGreenMachine/Zero.git
```
## Importing

### Importing into IntelliJ IDEA

1. Clone the project into your desired folder.
2. Open IntelliJ IDEA to the welcome screen.
3. **Do NOT select 'Open Project'**. Instead, select 'Import Project'.
4. Select `Import project from external model`. Then select 'Gradle'.
5. Configure your Gradle options:
    * Make sure to select the "Use gradle 'wrapper' task configuration' option.
    * Verify that the Gradle JVM is set to 'Use Project JDK' (which should be Java 11).
    * Keep other options as their defaults.
6. Select 'Finish'. Your project should load in and be set up without issues.

### Importing into Visual Studio Code

1. Make sure Visual Studio Code is properly set up with the Java Development Toolchains. Refer to [the WPILib Screensteps Guide](http://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/999999-installing-c-and-java-development-tools-for-frc) for further instructions.
2. Clone the project into your desired folder.
3. The WPILib VSCode extension should automatically detect and setup your project.

## Build and Deploy
Build the project by running the build task through the Gradle wrapper. Just issue the following command in a Bash shell:
```bash
$ ./gradlew build
```
This command isn't always necessary as it is often run by your IDE.

Deploy the project to the robot by first connecting to the robot's wi-fi network and then running the following command:
```bash
$ ./gradlew deploy
```
The deploy task will call the build task automatically.

To clear previous built binaries and minimize the possibility of bugs, it is recommended to run the clean task before deploying:
```bash
$ ./gradlew clean deploy
```
---
2023 - FRC Team 1816 The Green Machine
