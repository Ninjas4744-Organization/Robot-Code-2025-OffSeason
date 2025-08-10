# Robot-Code-2025-OffSeason
FRC Team Ninjas#4744's robot code for the 2025 FRC game **REEFSCAPE** (Offseason Edition).

## Organization
- **auto drive** – Autonomous routines in teleop, including auto-driving to the closest side of the reef and game piece output.
- **statemachine** – state machine system integrated with WPILib commands for easier and safer code design.
- **constants** – General constants for drivetrain, mechanisms, and sensors.
- **subsystems** – Each subsystem has its IO according to AdvantageKit usage.
- **NinjasLib** – Our custom library providing reusable FRC utilities, math helpers, hardware abstractions, swerve and vision.

## Notable Features
- **Cool FOM System** – A custom Field-Oriented Measurement system for odometry and vision fusion.
- **MegaTag2 Limelight Integration** – Tag-based localization with automatic camera selection and vision-based pose correction.
- **State Machine + WPILib Commands** – Combines the predictability of finite state machines with the flexibility of command-based programming.
- **AdvantageKit Logging** – Robust data logging for debugging and performance analysis.
- **Robot Physics Simulation** – Realistic simulation of drivetrain and mechanisms for testing without hardware.
- **Automated Driving** – Automatic navigation to the reef and controlled scoring/output sequences.
- **NinjasLib** – Internal utility framework used across multiple Ninjas robots.

## General Setup
1. **Clone this repo**
2. Build:
   ```bash
   ./gradlew build
3. Deploy on robot:
   ```bash
   ./gradlew deploy
4. Simulate on PC:
   ```bash
   ./gradlew simulateJavaRelease

## Development Environment
1. Visual Studio Code (Official WPILib IDE)
2. Jetbrains IntelliJ IDEA
3. AdvantageScope for debugging
4. Elastic for real-time data during games
5. Pathplanner for planning autos
