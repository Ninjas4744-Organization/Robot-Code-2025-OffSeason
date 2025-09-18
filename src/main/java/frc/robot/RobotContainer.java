package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInputIO;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInputIOReal;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInputIOSim;
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

    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.kRobotMode) {
            case REAL, SIM:
                arm = new Arm(false, new ArmIOController());
                elevator = new Elevator(false, new ElevatorIOController());
                intakeAngle = new IntakeAngle(false, new IntakeAngleIOController());
                intakeAligner = new IntakeAligner(false, new IntakeAlignerIOController());
                outtake = new Outtake(false, new OuttakeIOController());
                climber = new Climber(false, new ClimberIOController());
                swerveSubsystem = new SwerveSubsystem(true);

                if(Constants.kRobotMode == Constants.RobotMode.REAL)
                    intake = new Intake(false, new IntakeIOController(), new LoggedDigitalInputIOReal(), Constants.kIntakeBeamBreakerPort);
                else
                    intake = new Intake(false, new IntakeIOController(), new LoggedDigitalInputIOSim(() -> driverController.options().getAsBoolean()), Constants.kIntakeBeamBreakerPort);

                coralDetection = new CoralDetection(new CoralDetectionIOCamera());
                driverController = new LoggedCommandController(new LoggedCommandControllerIOPS5(Constants.kDriverControllerPort));
                break;

            case REPLAY:
                arm = new Arm(false, new ArmIO() {});
                elevator = new Elevator(false, new ElevatorIO() {});
                intake = new Intake(false, new IntakeIO() {}, new LoggedDigitalInputIO() {}, Constants.kIntakeBeamBreakerPort);
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
            },

            () -> Swerve.getInstance().getChassisSpeeds(false),

            drive -> SwerveController.getInstance().setControl(new SwerveInput(drive, false), "Auto"),

            Constants.kAutonomyConfig, //Autonomy config
            Constants.kSwerveConstants.robotConfig, //Robot config

            () -> false
        );

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    private void registerCommands() {
        NamedCommands.registerCommand("Intake Coral", new DetachedCommand(swerveSubsystem.driveToCoral()));
        NamedCommands.registerCommand("Wait Coral", Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == States.CORAL_IN_INTAKE));
        NamedCommands.registerCommand("Left Reef", changeRobotState(States.DRIVE_LEFT_REEF));
        NamedCommands.registerCommand("Right Reef", changeRobotState(States.DRIVE_RIGHT_REEF));
        NamedCommands.registerCommand("Outtake", changeRobotState(States.PREPARE_CORAL_OUTTAKE_L1));
        NamedCommands.registerCommand("L1", setL(1));
        NamedCommands.registerCommand("L2", setL(2));
        NamedCommands.registerCommand("L3", setL(3));
        NamedCommands.registerCommand("L4", setL(4));
    }

    private Command changeRobotState(States state) {
        return Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(state));
    }

    private Command setL(int L) {
        return Commands.runOnce(() -> RobotState.getInstance().setL(L));
    }

    private void configureBindings() {
        StateMachine stateMachine = StateMachine.getInstance();

        //region Driver buttons
//        driverController.R1().onTrue(Commands.runOnce(() -> Swerve.getInstance().getGyro().resetYaw(Rotation2d.kZero)));
//        driverController.L1().onTrue(Commands.runOnce(() -> Swerve.getInstance().getGyro().resetYaw(RobotState.getInstance().getRobotPose().getRotation())));

        driverController.R2().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.DRIVE_RIGHT_REEF)
        ));

        driverController.L2().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.DRIVE_LEFT_REEF)
        ));

        driverController.triangle().onTrue(Commands.either(
                Commands.runOnce(() -> stateMachine.changeRobotState(States.PREPARE_CORAL_OUTTAKE_L1)),
                Commands.runOnce(() -> stateMachine.changeRobotState(States.PREPARE_CORAL_OUTTAKE)),
                () -> RobotState.getL() == 1
        ));

        driverController.cross().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.INTAKE_CORAL)
        ));

//        driverController.square().onTrue(Commands.either(
//                Commands.runOnce(() ->  stateMachine.changeRobotState(States.PREPARE_ALGAE_OUTTAKE)),
//                Commands.runOnce(() -> stateMachine.changeRobotState(States.INTAKE_ALGAE_HIGH)) ,
//                () -> RobotState.getInstance().getRobotState() == States.ALGAE_IN_OUTTAKE
//        ));

        driverController.circle().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.CLOSE)
        ));
        //endregion

        //region Operator Buttons
        driverController.R1().onTrue(Commands.runOnce(() -> RobotState.setL(RobotState.getL() + 1)));
        driverController.L1().onTrue(Commands.runOnce(() -> RobotState.setL(RobotState.getL() - 1)));

        new Trigger(() -> RobotState.getL() > 1 && RobotState.getInstance().getRobotState() == States.CORAL_IN_INTAKE)
                .onTrue(Commands.runOnce(() -> stateMachine.changeRobotState(States.TRANSFER_CORAL_TO_OUTTAKE)));
        new Trigger(() -> RobotState.getL() == 1 && RobotState.getInstance().getRobotState() == States.CORAL_IN_OUTTAKE)
                .onTrue(Commands.runOnce(() -> stateMachine.changeRobotState(States.TRANSFER_CORAL_TO_INTAKE)));

//        operatorController.square().onTrue(Commands.runOnce(() ->
//                stateMachine.changeRobotState(States.INTAKE_ALGAE_LOW)
//        ));

        driverController.povDown().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.RESET)
        ));

//        operatorController.povUp().onTrue(Commands.either(
//                Commands.runOnce(() -> stateMachine.changeRobotState(States.CLIMB)),
//                Commands.runOnce(() -> stateMachine.changeRobotState(States.PREPARE_CLIMB)),
//                () -> RobotState.getInstance().getRobotState() == States.PREPARE_CLIMB
//        ));
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
////        stdCalculator.update(estimations);
//        for (VisionOutput estimation : estimations)
//            RobotState.getInstance().updateRobotPose(estimation, Constants.getVisionSTD(estimation));
//
//        coralDetection.periodic();
//        if (coralDetection.hasTargets()) {
//            Pose2d robotPose = RobotState.getInstance().getRobotPose();
//            Translation2d dir = coralDetection.getFieldRelativeDir();
//            Logger.recordOutput("Coral Detection Dir", new Pose2d(
//                    robotPose.getX() + dir.getX() / 2,
//                    robotPose.getY() + dir.getY() / 2,
//                    dir.getAngle()
//            ));
//        }

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
