package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.NinjasLib.localization.vision.Vision;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
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
import frc.robot.subsystems.intake_angle.IntakeAngle;
import frc.robot.subsystems.intake_angle.IntakeAngleIO;
import frc.robot.subsystems.intake_angle.IntakeAngleIOController;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private CommandPS5Controller driverController;
    private CommandPS5Controller operatorController;



    private static Elevator elevator;
    private static Arm arm;
    private static Intake intake;
    private static IntakeAngle intake_angle;
    private static Outtake outtake;
    private static Climber climber;
    private static SwerveSubsystem swerveSubsystem;

    private SendableChooser<Command> autoChooser;
    private FOMCalculator fomCalculator;

    public RobotContainer() {
        switch (Constants.kCurrentMode) {
            case REAL, SIM:
                arm = new Arm(false, new ArmIOController());
                elevator = new Elevator(false, new ElevatorIOController());
                intake = new Intake(false, new IntakeIOController());
                intake_angle = new IntakeAngle(false, new IntakeAngleIOController());
                outtake = new Outtake(false, new OuttakeIOController());
                climber = new Climber(false,new ClimberIOController());
                swerveSubsystem = new SwerveSubsystem(true);
                break;

            case REPLAY:
                arm = new Arm(false, new ArmIO() {
                });
                elevator = new Elevator(false, new ElevatorIO() {
                });
                intake = new Intake(false, new IntakeIO() {
                });
                intake_angle = new IntakeAngle(false, new IntakeAngleIO() {
                });
                outtake = new Outtake(false, new OuttakeIO() {
                });
                climber = new Climber(false, new ClimberIO() {
                });
                break;
        }

        RobotStateBase.setInstance(new RobotState(Constants.kSwerveConstants.kinematics, Constants.kInvertGyro, Constants.kPigeonID, Constants.kSwerveConstants.enableOdometryThread));

        Vision.setInstance(new Vision(Constants.kVisionConstants));
        fomCalculator = new FOMCalculator();

//        autoChooser = AutoBuilder.buildAutoChooser();

        driverController = new CommandPS5Controller(Constants.kDriverControllerPort);
        operatorController = new CommandPS5Controller(Constants.kOperatorControllerPort);

        if (Robot.isSimulation()) {
            for (int i = 0; i < 10; i++)
                SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(1.5, 4, Rotation2d.kZero)));
        }

        configureBindings();
    }

    private void configureBindings() {
        StateMachine stateMachine = StateMachine.getInstance();

        //region Driver Buttons
        //region Gyro Reset
        driverController.R1().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
        driverController.L1().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(RobotState.getInstance().getRobotPose().getRotation())));
        //endregion

        //region Auto Drive to Right Reef and score Coral High
        driverController.R2().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.DRIVE_TOWARDS_RIGHT_REEF)
        ));
        //endregion

        //region Auto Drive to Left Reef and score Coral High
        driverController.L2().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.DRIVE_TOWARDS_LEFT_REEF)
        ));
        //endregion

        //region Activating Coral Intake
        driverController.cross().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.INTAKE_CORAL)
        ));
        //endregion

        //region L1
        driverController.povRight().onTrue(Commands.sequence(
                Commands.runOnce(() ->
                        stateMachine.changeRobotState(States.PREPARE_CORAL_OUTTAKE_LOW)
                )
        ));
        //endregion

        //region Intake Algae reef OR Output Algae to barge when ahold of it (depending on state)
        driverController.square().onTrue(Commands.either(
                Commands.runOnce(() ->  stateMachine.changeRobotState(States.PREPARE_ALGAE_OUTTAKE)),
                Commands.runOnce(() -> stateMachine.changeRobotState(States.INTAKE_ALGAE_HIGH)) ,
                () -> RobotState.getInstance().getRobotState() == States.ALGAE_IN_ARM
        ));
        //endregion
        //endregion

        //region Operator Buttons

        RobotState robotState = RobotState.getInstance();
        //region Increment/decrement the desired L level to output coral
        operatorController.R1().onTrue(Commands.runOnce(() -> robotState.setL(robotState.getL() + 1)));

        operatorController.L1().onTrue(Commands.runOnce(() -> robotState.setL(robotState.getL() - 1)));
        //endregion

        //region Intake Algae floor
        operatorController.circle().onTrue(Commands.runOnce(() ->
                stateMachine.changeRobotState(States.INTAKE_ALGAE_LOW)
        ));
        //endregion

        //region Close
        operatorController.circle().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.CLOSE)
        ));
        //endregion

        //region Reset
        operatorController.povDown().onTrue(Commands.runOnce(
                () -> stateMachine.changeRobotState(States.RESET)
        ));
        //endregion

        //region Climb - prepare climbing for first click, and climb for second click.
        operatorController.povUp().onTrue(Commands.either(
                Commands.runOnce(() -> stateMachine.changeRobotState(States.CLIMB)),
                Commands.runOnce(() -> stateMachine.changeRobotState(States.PREPARE_CLIMB)),
                () -> RobotState.getInstance().getRobotState() == States.PREPARE_CLIMB
        ));
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

    public static Intake getIntake() {return intake;}

    public static IntakeAngle getIntakeAngle() {return intake_angle;}

    public static Outtake getOuttake() {return outtake;}

    public static Climber getClimber() {return climber;}

    public static SwerveSubsystem getSwerve() {return swerveSubsystem;}
    //endregion

    public void periodic() {
        swerveSubsystem.swerveDrive(driverController);

        VisionOutput[] estimations = Vision.getInstance().getVisionEstimations();
        fomCalculator.update(estimations);
        for (int i = 0; i < estimations.length; i++)
            RobotState.getInstance().updateRobotPose(estimations[i], fomCalculator.getOdometryFOM(), fomCalculator.getVisionFOM()[i]);

        if (Robot.isSimulation()) {
            Logger.recordOutput("Simulation Field/Corals", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            SimulatedArena.getInstance().simulationPeriodic();
        }
    }

    public Command getAutonomousCommand() {
        return Commands.none();
//        return autoChooser.getSelected();
    }

    public void reset() {
        Swerve.getInstance().resetModulesToAbsolute();
        SwerveController.getInstance().setChannel("Driver");
        SwerveController.getInstance().setControl(new SwerveInput(), "Driver");
    }
}
