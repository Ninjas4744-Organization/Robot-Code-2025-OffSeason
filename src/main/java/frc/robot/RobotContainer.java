package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.coraldetection.CoralDetection;
import frc.robot.coraldetection.CoralDetectionIO;
import frc.robot.coraldetection.CoralDetectionIOCamera;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOController;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOController;
import frc.robot.subsystems.intakealigner.IntakeAligner;
import frc.robot.subsystems.intakealigner.IntakeAlignerIO;
import frc.robot.subsystems.intakealigner.IntakeAlignerIOController;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.intakeangle.IntakeAngleIO;
import frc.robot.subsystems.intakeangle.IntakeAngleIOController;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
//    private CommandPS5Controller driverController;
    private LoggedCommandController driverController;
//    private CommandPS5Controller operatorController;

    private static Elevator elevator;
    private static Arm arm;
    private static Intake intake;
    private static IntakeAngle intakeAngle;
    private static IntakeAligner intakeAligner;
    private static Outtake outtake;
    private static Climber climber;
    private static CoralDetection coralDetection;
    private static SwerveSubsystem swerveSubsystem;

    private STDDevCalculator stdCalculator;
    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.kRobotMode) {
            case REAL, SIM:
                arm = new Arm(false, new ArmIOController());
                elevator = new Elevator(false, new ElevatorIOController());
                intake = new Intake(true, new IntakeIOController());
                intakeAngle = new IntakeAngle(false, new IntakeAngleIOController());
                intakeAligner = new IntakeAligner(false, new IntakeAlignerIOController());
                outtake = new Outtake(false, new OuttakeIOController());
                climber = new Climber(false, new ClimberIOController());
                swerveSubsystem = new SwerveSubsystem(true);

                coralDetection = new CoralDetection(new CoralDetectionIOCamera());
                driverController = new LoggedCommandController(new LoggedCommandControllerIOPS5(Constants.kDriverControllerPort));
                break;

            case REPLAY:
                arm = new Arm(false, new ArmIO() {});
                elevator = new Elevator(false, new ElevatorIO() {});
                intake = new Intake(true, new IntakeIO() {});
                intakeAngle = new IntakeAngle(false, new IntakeAngleIO() {});
                intakeAligner = new IntakeAligner(false, new IntakeAlignerIO() {});
                outtake = new Outtake(false, new OuttakeIO() {});
                climber = new Climber(false, new ClimberIO() {});
                swerveSubsystem = new SwerveSubsystem(true);

                coralDetection = new CoralDetection(new CoralDetectionIO() {});
                driverController = new LoggedCommandController(new LoggedCommandControllerIO() {});
                break;
        }

        RobotStateBase.setInstance(new RobotState(Constants.kSwerveConstants.kinematics));
        StateMachineBase.setInstance(new StateMachine());
//        Vision.setInstance(new Vision(Constants.kVisionConstants));
        stdCalculator = new STDDevCalculator();

        if (Robot.isSimulation()) {
            for (int i = 0; i < 10; i++)
                SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(1.5, 4, Rotation2d.kZero)));
        }

        registerCommands();
        configureAuto();
        configureBindings();
    }

    private void configureAuto() {
        AutoBuilder.configure(
            () -> RobotState.getInstance().getRobotPose(), // Robot pose supplier

            pose -> {
                RobotState.getInstance().setRobotPose(pose);
//                Swerve.getInstance().getGyro().resetYaw(pose.getRotation()); //TODO RETURN
            }, // Method to reset odometry (will be called if your auto has a starting pose)

            () -> Swerve.getInstance().getChassisSpeeds(false), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

            drive -> SwerveController.getInstance().setControl(new SwerveInput(drive, false), "Auto"), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

            Constants.kAutonomyConfig, //Autonomy config
            Constants.kSwerveConstants.robotConfig, //Robot config

            () -> false
        );

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    private void registerCommands() {
        NamedCommands.registerCommand("Intake Coral", new DetachedCommand(swerveSubsystem.driveToCoral()));
        NamedCommands.registerCommand("Wait Coral", Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == States.CORAL_IN_INTAKE));
        NamedCommands.registerCommand("Drive Left Reef", changeRobotState(States.DRIVE_LEFT_REEF));
        NamedCommands.registerCommand("Drive Right Reef", changeRobotState(States.DRIVE_RIGHT_REEF));
        NamedCommands.registerCommand("Outtake", changeRobotState(States.PREPARE_CORAL_OUTTAKE_L1));
        NamedCommands.registerCommand("Set L1", setL(1));
        NamedCommands.registerCommand("Set L2", setL(2));
        NamedCommands.registerCommand("Set L3", setL(3));
        NamedCommands.registerCommand("Set L4", setL(4));
    }

    private Command changeRobotState(States state) {
        return Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(state));
    }

    private Command setL(int L) {
        return Commands.runOnce(() -> RobotState.getInstance().setL(L));
    }

    private void configureBindings() {
        StateMachine stateMachine = StateMachine.getInstance();

        //region Driver Buttons
        //region Gyro Reset
        driverController.R1().onTrue(Commands.runOnce(() -> Swerve.getInstance().getGyro().resetYaw(Rotation2d.kZero)));
        driverController.L1().onTrue(Commands.runOnce(() -> Swerve.getInstance().getGyro().resetYaw(RobotState.getInstance().getRobotPose().getRotation())));
        //endregion

        //region Auto Drive to Right Reef and score Coral High/low
//        driverController.R2().onTrue(Commands.runOnce(
//                () -> stateMachine.changeRobotState(States.DRIVE_TOWARDS_RIGHT_REEF)
//        ));
        driverController.R2().onTrue(new DetachedCommand(swerveSubsystem.driveToCoral()));
        //endregion

        //region Auto Drive to Left Reef and score Coral High/low
        driverController.L2().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.DRIVE_LEFT_REEF)
        ));
        //endregion

        //region Score Coral High/low [NO AUTO DRIVE]
        driverController.triangle().onTrue(Commands.either(
                Commands.runOnce(() -> stateMachine.changeRobotState(States.PREPARE_CORAL_OUTTAKE_L1)),
                Commands.runOnce(() -> stateMachine.changeRobotState(States.PREPARE_CORAL_OUTTAKE)),
                () -> RobotState.getInstance().getL() == 1
        ));
        //endregion

        //region Activating Coral Intake
        driverController.cross().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.INTAKE_CORAL)
        ));
        //endregion

        //region Intake Algae reef OR Output Algae to barge when ahold of it (depending on state)
        driverController.square().onTrue(Commands.either(
                Commands.runOnce(() ->  stateMachine.changeRobotState(States.PREPARE_ALGAE_OUTTAKE)),
                Commands.runOnce(() -> stateMachine.changeRobotState(States.INTAKE_ALGAE_HIGH)) ,
                () -> RobotState.getInstance().getRobotState() == States.ALGAE_IN_ARM
        ));

        driverController.circle().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.CLOSE)
        ));
        //endregion
        //endregion

        //region Operator Buttons

        RobotState robotState = RobotState.getInstance();
        //region Increment/decrement the desired L level to output coral. Also takes control of what robot part holds the coral.

        // For example, if we have increased the L level from 1 to 2, the robot will transfer the coral from the intake to the arm, and vise versa.
//        operatorController.R1().onTrue(Commands.runOnce(() -> robotState.setL(robotState.getL() + 1)));
//        operatorController.L1().onTrue(Commands.runOnce(() -> robotState.setL(robotState.getL() - 1)));

        // Have triggers that track when the robot should transfer coral from the intake to the arm, or from the arm to the intake.
        new Trigger(() -> robotState.getL() > 1 && robotState.getRobotState() == States.CORAL_IN_INTAKE)
                .onTrue(Commands.runOnce(() -> stateMachine.changeRobotState(States.TRANSFER_CORAL_FROM_INTAKE_TO_OUTTAKE)));
        new Trigger(() -> robotState.getL() == 1 && robotState.getRobotState() == States.CORAL_IN_OUTTAKE)
                .onTrue(Commands.runOnce(() -> stateMachine.changeRobotState(States.TRANSFER_CORAL_FROM_OUTTAKE_TO_INTAKE)));
        //endregion

        //region Intake Algae floor
//        operatorController.square().onTrue(Commands.runOnce(() ->
//                stateMachine.changeRobotState(States.INTAKE_ALGAE_LOW)
//        ));
        //endregion

        //region Reset
//        operatorController.povDown().onTrue(Commands.runOnce(
//                () -> stateMachine.changeRobotState(States.RESET)
//        ));
        //endregion

        //region Climb - prepare climbing for the first click, and climb for the second click.
//        operatorController.povUp().onTrue(Commands.either(
//                Commands.runOnce(() -> stateMachine.changeRobotState(States.CLIMB)),
//                Commands.runOnce(() -> stateMachine.changeRobotState(States.PREPARE_CLIMB)),
//                () -> RobotState.getInstance().getRobotState() == States.PREPARE_CLIMB
//        ));
        //endregion

        //endregion
    }

    //region Subsystem Instances
    public static Elevator getElevator() {
        return elevator;
    }

    public static Arm getArm() {
        return arm;
    }

    public static Intake getIntake() {
        return intake;
    }

    public static IntakeAngle getIntakeAngle() {
        return intakeAngle;
    }

    public static IntakeAligner getIntakeAligner() {
        return intakeAligner;
    }

    public static Outtake getOuttake() {
        return outtake;
    }

    public static Climber getClimber() {
        return climber;
    }

    public static CoralDetection getCoralDetection() {
        return coralDetection;
    }

    public static SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }
    //endregion

    public void periodic() {
        swerveSubsystem.swerveDrive(driverController);

//        VisionOutput[] estimations = Vision.getInstance().getVisionEstimations();
//        stdCalculator.update(estimations);
//        for (int i = 0; i < estimations.length; i++)
//            RobotState.getInstance().updateRobotPose(estimations[i], stdCalculator.getOdometrySTDDev(), stdCalculator.getVisionSTDDev()[i]);

        coralDetection.periodic();
        if (coralDetection.hasTargets()) {
            Pose2d robotPose = RobotState.getInstance().getRobotPose();
            Translation2d dir = coralDetection.getFieldRelativeDir();
            Logger.recordOutput("Coral Detection Dir", new Pose2d(
                    robotPose.getX() + dir.getX() / 2,
                    robotPose.getY() + dir.getY() / 2,
                    dir.getAngle()
            ));
        }

        driverController.periodic();

        if (Robot.isSimulation()) {
            Logger.recordOutput("Simulation Field/Corals", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            SimulatedArena.getInstance().simulationPeriodic();
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void reset() {
        Swerve.getInstance().resetModulesToAbsolute();
        SwerveController.getInstance().setChannel(DriverStation.isAutonomous() ? "Auto" : "Driver");
        SwerveController.getInstance().setControl(new SwerveInput(), DriverStation.isAutonomous() ? "Auto" : "Driver");
        StateMachine.getInstance().changeRobotState(States.RESET);
    }
}
