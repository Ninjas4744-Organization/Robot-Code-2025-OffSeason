package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakealigner.IntakeAligner;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.outtake.Outtake;

import java.util.Set;

public class StateMachine extends StateMachineBase<States> {
    public static StateMachine getInstance(){
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected boolean canChangeRobotState(States currentState, States wantedState) {
        if (wantedState == States.RESET || wantedState == States.CLOSE)
            return true;

        return switch (currentState) {
            case IDLE -> Set.of(
                    States.INTAKE_CORAL,
                    States.INTAKE_ALGAE_HIGH,
                    States.INTAKE_ALGAE_LOW,
                    States.PREPARE_CLIMB,
                    States.CORAL_IN_OUTTAKE
            ).contains(wantedState);

            case INTAKE_CORAL -> Set.of(
                    States.CORAL_IN_INTAKE,
                    States.TRANSFER_CORAL_TO_OUTTAKE
            ).contains(wantedState);

            case CORAL_IN_INTAKE -> Set.of(
                    States.TRANSFER_CORAL_TO_OUTTAKE,
                    States.PREPARE_CORAL_OUTTAKE_L1,
                    States.PREPARE_CORAL_OUTTAKE,
                    States.DRIVE_LEFT_REEF,
                    States.DRIVE_RIGHT_REEF,
                    States.INTAKE_CORAL
            ).contains(wantedState);

            // LOW CORAL
            case PREPARE_CORAL_OUTTAKE_L1 -> Set.of(
                    States.CORAL_OUTTAKE_L1
            ).contains(wantedState);

            case CORAL_OUTTAKE_L1 -> Set.of(
                    States.CLOSE,
                    States.RESET
            ).contains(wantedState);

            // HIGH CORAL
            case TRANSFER_CORAL_TO_OUTTAKE -> Set.of(
                    States.CORAL_IN_OUTTAKE
            ).contains(wantedState);

            case CORAL_IN_OUTTAKE -> Set.of(
                    States.DRIVE_LEFT_REEF,
                    States.DRIVE_RIGHT_REEF,
                    States.TRANSFER_CORAL_TO_INTAKE,
                    States.PREPARE_CORAL_OUTTAKE,
                    States.INTAKE_CORAL
            ).contains(wantedState);

            case TRANSFER_CORAL_TO_INTAKE -> Set.of(
                    States.CORAL_IN_INTAKE
            ).contains(wantedState);

            case DRIVE_LEFT_REEF, DRIVE_RIGHT_REEF -> Set.of(
                    States.PREPARE_CORAL_OUTTAKE,
                    States.PREPARE_CORAL_OUTTAKE_L1,
                    States.CLOSE
            ).contains(wantedState);

            case PREPARE_CORAL_OUTTAKE -> Set.of(
                    States.CORAL_OUTTAKE
            ).contains(wantedState);

            case CORAL_OUTTAKE -> Set.of(
                    States.INTAKE_CORAL
            ).contains(wantedState);

            // ALGAE
            case INTAKE_ALGAE_LOW,INTAKE_ALGAE_HIGH -> Set.of(
                    States.ALGAE_IN_OUTTAKE
            ).contains(wantedState);
            case ALGAE_IN_OUTTAKE -> Set.of(
                    States.PREPARE_ALGAE_OUTTAKE
            ).contains(wantedState);
            case PREPARE_ALGAE_OUTTAKE -> Set.of(
                    States.ALGAE_OUTTAKE
            ).contains(wantedState);
            case ALGAE_OUTTAKE -> Set.of(
                    States.CLOSE,
                    States.RESET
            ).contains(wantedState);

            case CLOSE, RESET -> Set.of(
                    States.IDLE,
                    States.CORAL_IN_INTAKE,
                    States.CORAL_IN_OUTTAKE,
                    States.ALGAE_IN_OUTTAKE
            ).contains(wantedState);

            case PREPARE_CLIMB -> Set.of(
                    States.CLIMB
            ).contains(wantedState);

            case CLIMB -> Set.of(
                    States.CLIMBED
            ).contains(wantedState);

            case CLIMBED -> false;
        };
    }

//    private boolean intakeAfterClose = false;
    @Override
    protected void setCommandMap() {
        Intake intake = RobotContainer.getIntake();
        Outtake outtake = RobotContainer.getOuttake();
        Arm arm = RobotContainer.getArm();
        IntakeAngle intakeAngle = RobotContainer.getIntakeAngle();
        IntakeAligner intakeAligner = RobotContainer.getIntakeAligner();
        Elevator elevator = RobotContainer.getElevator();
        Climber climber = RobotContainer.getClimber();
        SwerveSubsystem swerve = RobotContainer.getSwerve();

        addCommand(States.IDLE, Commands.runOnce(() -> {
//            if (intakeAfterClose) {
//                intakeAfterClose = false;
//                changeRobotState(States.INTAKE_CORAL);
//            }
        }));

        addCommand(States.INTAKE_CORAL, Commands.sequence(
                Commands.either(
                        Commands.sequence(
//                                elevator.goToSafeHeight(),
                                elevator.setHeight(() -> Constants.Elevator.Positions.Close.get()),
                                Commands.waitUntil(() -> elevator.getHeight() > 1),
                                arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.IntakeCoral.get()))
                        ),
                        Commands.none(),
                        () -> RobotState.getL() > 1
                ),
                intake.intake(),
                intakeAngle.lookDown(),
                intakeAligner.align(),
                Commands.waitUntil(intake::isCoralInside),
//                Commands.waitSeconds(0.2),
                intakeAligner.stop(),
                intake.stop(),
                Commands.either(
                        Commands.runOnce(() -> changeRobotState(States.TRANSFER_CORAL_TO_OUTTAKE)),
                        Commands.runOnce(() -> changeRobotState(States.CLOSE)),
                        () -> RobotState.getL() > 1
                )
        ));

        addCommand(States.CORAL_IN_INTAKE, Commands.none());

        addCommand(States.PREPARE_CORAL_OUTTAKE_L1, Commands.sequence(
                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get())),
                intakeAngle.lookAtL1(),
                intakeAligner.align(),
                intake.intake(),
                Commands.waitUntil(intakeAngle::atGoal),
                intakeAligner.stop(),
                intake.stop(),
                Commands.runOnce(() -> changeRobotState(States.CORAL_OUTTAKE_L1))
        ));

        addCommand(States.CORAL_OUTTAKE_L1, Commands.sequence(
                intake.outtake(),
                Commands.waitSeconds(0.5),
                Commands.runOnce(()-> changeRobotState(States.CLOSE))
        ));

        addCommand(States.TRANSFER_CORAL_TO_OUTTAKE, Commands.sequence(
                intake.intake(),
                intakeAligner.align(),
                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.IntakeCoral.get())),
                Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal()),
                intakeAngle.lookAtArm(),
                Commands.waitUntil(() -> intakeAngle.getAngle().getRadians() > 0.35),
                elevator.setHeight(Constants.Elevator.Positions.Intake::get),
                Commands.waitUntil(elevator::atGoal),
                outtake.intake(),
                intake.outtake(),
                Commands.waitUntil(() -> !intake.isCoralInside()),
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> outtake.forceKnowCoralInside(true)),
                Commands.runOnce(() -> changeRobotState(States.CLOSE))
        ));

        addCommand(States.CORAL_IN_OUTTAKE, Commands.sequence(
                arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.CoralReady.get())),
                Commands.waitUntil(() -> arm.getAngle().getDegrees() < -180),
                elevator.setHeight(Constants.Elevator.Positions.CoralReady::get)
        ));

        addCommand(States.TRANSFER_CORAL_TO_INTAKE, Commands.sequence(
                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.IntakeCoral.get())),
                Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal()),
                intakeAngle.lookAtArm(),
                Commands.waitUntil(() -> intakeAngle.getAngle().getRadians() > 0.35),
                elevator.setHeight(Constants.Elevator.Positions.Intake::get),
                Commands.waitUntil(elevator::atGoal),
                outtake.outtake(),
                intake.intake(),
                intakeAligner.align(),
                Commands.waitUntil(intake::isCoralInside),
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> outtake.forceKnowCoralInside(false)),
                Commands.runOnce(() -> changeRobotState(States.CLOSE))
        ));

        addCommand(States.DRIVE_LEFT_REEF, Commands.sequence(
                Commands.waitUntil(() -> RobotState.getInstance().getDistance(new Pose2d(4, 4, Rotation2d.kZero)) < 3.5),
                new DeferredCommand(() -> new DetachedCommand(swerve.autoDriveToReef(() -> false).get()), Set.of()),
                Commands.waitUntil(() -> swerve.distFromGoal() < 1.5),
                Commands.either(
                        Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.PREPARE_CORAL_OUTTAKE_L1)),
                        Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.PREPARE_CORAL_OUTTAKE)),
                        () -> RobotState.getL() == 1
                )
        ));
        addCommand(States.DRIVE_RIGHT_REEF, Commands.sequence(
                Commands.waitUntil(() -> RobotState.getInstance().getDistance(new Pose2d(4, 4, Rotation2d.kZero)) < 3.5),
                new DeferredCommand(() -> new DetachedCommand(swerve.autoDriveToReef(() -> true).get()), Set.of()),
                Commands.waitUntil(() -> swerve.distFromGoal() < 1.5),
                Commands.either(
                        Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.PREPARE_CORAL_OUTTAKE_L1)),
                        Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.PREPARE_CORAL_OUTTAKE)),
                        () -> RobotState.getL() == 1
                )
        ));

        addCommand(States.PREPARE_CORAL_OUTTAKE, Commands.sequence(
                Commands.either(
                        Commands.sequence(
                            elevator.setHeight(() -> Constants.Elevator.LPositions[RobotState.getL() - 1]),
                            arm.setAngle(() -> Constants.Arm.LPositions[RobotState.getL() - 1])
                        ),
                        Commands.sequence(
                            arm.setAngle(() -> Constants.Arm.LPositions[RobotState.getL() - 1]),
                            Commands.waitUntil(() -> arm.getAngle().getDegrees() < -180),
                            elevator.setHeight(() -> Constants.Elevator.LPositions[RobotState.getL() - 1])
                        ),
                        () -> Constants.Elevator.LPositions[RobotState.getL() - 1] > 6
                ),
                Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal() && swerve.atGoal()),
                Commands.runOnce(() -> changeRobotState(States.CORAL_OUTTAKE))
        ));

        addCommand(States.CORAL_OUTTAKE, Commands.sequence(
                arm.setAngle(() -> Constants.Arm.LPositionsDown[RobotState.getL() - 1]),
                elevator.setHeight(() -> Constants.Elevator.LPositionsDown[RobotState.getL() - 1]),
                Commands.waitUntil(() -> arm.getAngle().getDegrees() < Constants.Arm.LPositions[RobotState.getL() - 1].getDegrees() - 15),
//                Commands.waitUntil(() -> RobotContainer.finishOuttake),
//                Commands.runOnce(() -> RobotContainer.finishOuttake = false),
                outtake.outtake(),
                Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal()),
                swerve.close(),
                Commands.runOnce(() -> {
                    SwerveController.getInstance().setChannel("BackReef");
                    SwerveController.getInstance().setControl(new SwerveInput(-1, 0, 0, false), "BackReef");
                }),
                Commands.either(
                        Commands.waitSeconds(0.1),
                        Commands.none(),
                        () -> RobotState.getL() < 4
                ),
                new DetachedCommand(arm.setAngleSmart(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get()))),
                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                new DetachedCommand(Commands.sequence(
                        Commands.waitSeconds(0.4),
                        swerve.close()
                )),
                Commands.runOnce(()-> changeRobotState(States.INTAKE_CORAL))
//                Commands.runOnce(() -> intakeAfterClose = true),
//                Commands.runOnce(()-> changeRobotState(States.CLOSE))
        ));
        addCommand(States.INTAKE_ALGAE_LOW, Commands.sequence(
                intakeAngle.setAngle(Rotation2d.fromDegrees(Constants.IntakeAngle.Positions.Algae.get())),
                Commands.waitUntil(() -> intakeAngle.getAngle().getRadians() < 0.35),
                arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.IntakeAlgae.get())),
                Commands.waitUntil(arm::atGoal),
                elevator.setHeight(Constants.Elevator.Positions.AlgaeLow::get),
                Commands.waitUntil(elevator::atGoal),
                outtake.intakeAlgae(),
                Commands.waitUntil(outtake::isAlgaeInside),
                Commands.runOnce(()-> changeRobotState(States.ALGAE_IN_OUTTAKE))
        ));

        addCommand(States.INTAKE_ALGAE_HIGH, Commands.sequence(
                //TODO: - There's 2 different heights for Algae in reef, and we need to distinguish the 2 heights. thoughts Eitan?
//                arm.lookAtAlgaeReef(),
//                elevator.goToAlgaeReefHeight(),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal()),
                outtake.intakeAlgae(),
                Commands.waitUntil(outtake::isAlgaeInside),
                Commands.runOnce(()-> changeRobotState(States.ALGAE_IN_OUTTAKE))
        ));

        addCommand(States.ALGAE_IN_OUTTAKE, Commands.sequence(
                outtake.intakeAlgae(),
                arm.setAngle(() -> Rotation2d.fromDegrees(90)),
                elevator.setHeight(Constants.Elevator.Positions.AlgaeLow::get),
                Commands.waitUntil(() -> arm.getAngle().getDegrees() > 75),
                intakeAngle.close()
//                Commands.waitUntil(() -> !outtake.isAlgaeInside()),
//                Commands.runOnce(() -> changeRobotState(States.CLOSE))
        ));

        addCommand(States.PREPARE_ALGAE_OUTTAKE, Commands.sequence(
                outtake.intakeAlgae(),
                elevator.setHeight(Constants.Elevator.Positions.Net::get),
                arm.setAngle(() -> {
                    double robotAngle = RobotState.getInstance().getRobotPose().getRotation().getDegrees();
                    if (Math.abs(0 - robotAngle) < 90)
                        return Rotation2d.fromDegrees(Constants.Arm.Positions.NetInverse.get());
                    return Rotation2d.fromDegrees(Constants.Arm.Positions.Net.get());
                }),
                Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal())
        ));

        addCommand(States.ALGAE_OUTTAKE, Commands.sequence(
                outtake.outtakeAlgae(),
                Commands.waitSeconds(0.2),
                outtake.stop(),
                Commands.runOnce(()-> changeRobotState(States.CLOSE))
        ));
        //endregion

        //region close + reset
        addCommand(States.CLOSE, Commands.sequence(
                Commands.parallel(
                        outtake.stop(),
                        intake.stop(),
                        intakeAligner.stop(),
                        swerve.close(),
                        Commands.sequence(
                                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                                arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get()))
                        )
                ),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal() && outtake.isReset()),
                new DetachedCommand(Commands.sequence(
                    Commands.waitSeconds(0.1),
                    arm.reset()
                )),
//                Commands.either(
//                        Commands.sequence(
//                                intakeAngle.reset(),
//                                intakeAngle.lookDown()
//                        ),
                        Commands.sequence(
                                intakeAngle.close(),
                                Commands.waitUntil(intakeAngle::atGoal),
                                new DetachedCommand(Commands.sequence(
                                        Commands.waitSeconds(0.1),
                                        intakeAngle.reset()
                                ))
                        ),
//                        () -> intakeAfterClose
//                ),
                Commands.waitUntil(intakeAngle::atGoal),
                Commands.runOnce(() -> {
                    if (outtake.isCoralInside())
                        changeRobotState(States.CORAL_IN_OUTTAKE);
                    else if (outtake.isAlgaeInside())
                        changeRobotState(States.ALGAE_IN_OUTTAKE);
                    else if (intake.isCoralInside())
                        changeRobotState(States.CORAL_IN_INTAKE);
                    else
                        changeRobotState(States.IDLE);
                })
        ));

        addCommand(States.RESET, Commands.sequence(
                Commands.parallel(
                    intake.reset(),
                    intakeAligner.stop(),
                    swerve.reset(),
                    outtake.reset(),
                    Commands.either(
                            Commands.sequence(
                                    arm.reset(),
                                    intakeAngle.reset(),
                                    elevator.setHeight(Constants.Elevator.Positions.Close::get),
                                    Commands.waitUntil(elevator::atGoal),
                                    arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get())),
                                    intakeAngle.close()
                            ),
                            Commands.sequence(
                                    arm.reset(),
                                    intakeAngle.reset(),
                                    elevator.specialReset(),
                                    new DetachedCommand(arm.setAngleSmart(() -> {
                                        double rawDelta = 90 - arm.getAngle().getDegrees();
                                        double delta = ((rawDelta % 360.0) + 540.0) % 360.0 - 180.0;
                                        return Rotation2d.fromDegrees(arm.getAngle().getDegrees() + delta);
                                    })),
                                    intakeAngle.lookDown(),
                                    Commands.waitUntil(arm::atGoal),
                                    Commands.parallel(
                                        elevator.reset(),
                                        intakeAngle.close()
                                    ),
                                    Commands.waitUntil(elevator::isReset),
                                    new DetachedCommand(arm.setAngleSmart(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get()))),
                                    elevator.setHeight(Constants.Elevator.Positions.Close::get),
                                    Commands.waitUntil(elevator::atGoal)
//                                    arm.setAngle(() -> Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get()))
                            ),
                            elevator::isReset
                    )
                ),
                Commands.waitUntil(() -> arm.isReset() && intakeAngle.isReset() && outtake.isReset()),
                intakeAngle.reset(),
                Commands.runOnce(() -> {
                    if (outtake.isCoralInside())
                        changeRobotState(States.CORAL_IN_OUTTAKE);
                    else if (outtake.isAlgaeInside())
                        changeRobotState(States.ALGAE_IN_OUTTAKE);
                    else if (intake.isCoralInside())
                        changeRobotState(States.CORAL_IN_INTAKE);
                    else
                        changeRobotState(States.IDLE);
                })
        ));
        //endregion

        //region climb
        //TODO: PREPARE CLIMB COMMAND
        addCommand(States.PREPARE_CLIMB, Commands.none());
        //TODO: CLIMB COMMAND
        addCommand(States.CLIMB, Commands.sequence(
        ));

        addCommand(States.CLIMBED, Commands.none());
        //endregion
    }
}
