package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake_angle.IntakeAngle;
import frc.robot.subsystems.outtake.Outtake;

import java.util.Set;

public class StateMachine extends StateMachineBase<States> {
    public static StateMachine getInstance(){
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected boolean canChangeRobotState(States currentState, States wantedState) {
        return switch (currentState) {
            case IDLE -> Set.of(
                    States.INTAKE_CORAL,
                    States.INTAKE_ALGAE_HIGH,
                    States.INTAKE_ALGAE_LOW,
                    States.PREPARE_CLIMB
            ).contains(wantedState);


            case INTAKE_CORAL -> Set.of(
                    States.CORAL_IN_INTAKE
            ).contains(wantedState);

            case CORAL_IN_INTAKE -> Set.of(
                    States.TRANSFER_CORAL_FROM_INTAKE_TO_ARM,
                    States.PREPARE_CORAL_OUTTAKE_LOW,
                    States.PREPARE_CORAL_OUTTAKE_HIGH,
                    States.DRIVE_TOWARDS_LEFT_REEF,
                    States.DRIVE_TOWARDS_RIGHT_REEF
            ).contains(wantedState);

            // LOW CORAL
            case PREPARE_CORAL_OUTTAKE_LOW -> Set.of(
                    States.CORAL_OUTTAKE_LOW
            ).contains(wantedState);

            case CORAL_OUTTAKE_LOW -> Set.of(
                    States.CLOSE,
                    States.RESET
            ).contains(wantedState);

            // HIGH CORAL
            case TRANSFER_CORAL_FROM_INTAKE_TO_ARM -> Set.of(
                    States.CORAL_IN_ARM
            ).contains(wantedState);

            case CORAL_IN_ARM -> Set.of(
                    States.DRIVE_TOWARDS_LEFT_REEF,
                    States.DRIVE_TOWARDS_RIGHT_REEF,
                    States.TRANSFER_CORAL_FROM_ARM_TO_INTAKE,
                    States.PREPARE_CORAL_OUTTAKE_HIGH
            ).contains(wantedState);

            case TRANSFER_CORAL_FROM_ARM_TO_INTAKE -> Set.of(
                    States.CORAL_IN_INTAKE
            ).contains(wantedState);


            case DRIVE_TOWARDS_LEFT_REEF, DRIVE_TOWARDS_RIGHT_REEF -> Set.of(
                    States.PREPARE_CORAL_OUTTAKE_HIGH,
                    States.PREPARE_CORAL_OUTTAKE_LOW,
                    States.CLOSE
            ).contains(wantedState);

            case PREPARE_CORAL_OUTTAKE_HIGH -> Set.of(
                    States.CORAL_OUTTAKE_HIGH
            ).contains(wantedState);

            case CORAL_OUTTAKE_HIGH -> Set.of(
                    States.CLOSE,
                    States.RESET
            ).contains(wantedState);


            // ALGAE
            case INTAKE_ALGAE_LOW,INTAKE_ALGAE_HIGH -> Set.of(
                    States.ALGAE_IN_ARM
            ).contains(wantedState);
            case ALGAE_IN_ARM -> Set.of(
                    States.PREPARE_ALGAE_OUTTAKE
            ).contains(wantedState);
            case PREPARE_ALGAE_OUTTAKE -> Set.of(
                    States.ALGAE_OUTTAKE
            ).contains(wantedState);
            case ALGAE_OUTTAKE -> Set.of(
                    States.CLOSE,
                    States.RESET
            ).contains(wantedState);


            case CLOSE,RESET -> Set.of(
                    States.IDLE
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

    @Override
    protected void setCommandMap() {
        Intake intake = RobotContainer.getIntake();
        Outtake outtake = RobotContainer.getOuttake();
        Arm arm = RobotContainer.getArm();
        IntakeAngle intakeAngle = RobotContainer.getIntakeAngle();
        Elevator elevator = RobotContainer.getElevator();
        Climber climber = RobotContainer.getClimber();
        SwerveSubsystem swerve = RobotContainer.getSwerve();


        //region idle
        addCommand(States.IDLE, Commands.none());
        //endregion

        //region intake coral

        addCommand(States.INTAKE_CORAL,Commands.sequence(
                intake.intakeCoral(),
                intakeAngle.lookDown(),
                Commands.waitUntil(intake::isCoralInside),
                intake.stop(),
                Commands.runOnce(()-> changeRobotState(States.CORAL_IN_INTAKE))
        ));

        addCommand(States.CORAL_IN_INTAKE, Commands.none());
        //endregion

        //region outtake coral
        addCommand(States.PREPARE_CORAL_OUTTAKE_LOW, Commands.sequence(
                intakeAngle.lookAtL1(),
                Commands.waitUntil(intakeAngle::atGoal),
                Commands.runOnce(()-> changeRobotState(States.CORAL_OUTTAKE_LOW))
        ));
        addCommand(States.CORAL_OUTTAKE_LOW, Commands.sequence(
                intake.outtakeCoral(),
                Commands.waitSeconds(0.2),
                intakeAngle.lookDown(),
                Commands.runOnce(()-> changeRobotState(States.CLOSE))
        ));
        addCommand(States.TRANSFER_CORAL_FROM_INTAKE_TO_ARM, Commands.sequence(
                intakeAngle.lookAtArm(),
                arm.lookDown(),
                Commands.waitUntil(() -> intakeAngle.atGoal() && arm.atGoal()),
                outtake.intakeObject(),
                intake.outtakeCoral(),
                Commands.waitUntil(arm::isObjectInside),
                Commands.runOnce(()-> changeRobotState(States.CORAL_IN_ARM))
        ));
        addCommand(States.CORAL_IN_ARM, Commands.none());

        addCommand(States.TRANSFER_CORAL_FROM_ARM_TO_INTAKE, Commands.sequence(
                intakeAngle.lookAtArm(),
                arm.lookDown(),
                Commands.waitUntil(() -> intakeAngle.atGoal() && arm.atGoal()),
                intake.intakeCoral(),
                outtake.outtakeCoral(),
                Commands.waitUntil(arm::isObjectInside),
                Commands.runOnce(()-> changeRobotState(States.CORAL_IN_INTAKE))
        ));

        addCommand(States.DRIVE_TOWARDS_LEFT_REEF, Commands.sequence(
                swerve.autoDriveToReef(() -> false),
                Commands.either(
                        Commands.runOnce( () -> StateMachine.getInstance().changeRobotState(States.PREPARE_CORAL_OUTTAKE_LOW)),
                        Commands.runOnce( () -> StateMachine.getInstance().changeRobotState(States.PREPARE_CORAL_OUTTAKE_HIGH)),
                        () -> RobotState.getInstance().getL() == 1
                )
        ));
        addCommand(States.DRIVE_TOWARDS_RIGHT_REEF, Commands.sequence(
                swerve.autoDriveToReef(() -> true),
                Commands.either(
                        Commands.runOnce( () -> StateMachine.getInstance().changeRobotState(States.PREPARE_CORAL_OUTTAKE_LOW)),
                        Commands.runOnce( () -> StateMachine.getInstance().changeRobotState(States.PREPARE_CORAL_OUTTAKE_HIGH)),
                        () -> RobotState.getInstance().getL() == 1
                )
        ));

        addCommand(States.PREPARE_CORAL_OUTTAKE_HIGH, Commands.sequence(
                elevator.goToLHeight(RobotState.getInstance().getL()),
                arm.lookAtCoralReef(RobotState.getInstance().getL()),
                Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal()),
                Commands.runOnce(()-> changeRobotState(States.CORAL_OUTTAKE_HIGH))
        ));
        addCommand(States.CORAL_OUTTAKE_HIGH, Commands.sequence(
                outtake.outtakeCoral(),
                Commands.waitSeconds(0.2),
                Commands.runOnce(()-> changeRobotState(States.CLOSE))
        ));
        //endregion

        //region intake algae
        addCommand(States.INTAKE_ALGAE_LOW, Commands.sequence(
                arm.lookAtAlgaeFloor(),
                elevator.goToFloor(),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal()),
                outtake.intakeObject(),
                Commands.waitUntil(arm::isObjectInside),
                Commands.runOnce(()-> changeRobotState(States.ALGAE_IN_ARM))
                // No need to stop the intaking because we want to keep ahold of the algae. we'll stop only when we outtake.
        ));
        addCommand(States.INTAKE_ALGAE_HIGH, Commands.sequence(
                //TODO: - There's 2 different heights for Algae in reef, and we need to distinguish the 2 heights. thoughts Eitan?
                arm.lookAtAlgaeReef(),
                elevator.goToAlgaeReefHeight(),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal()),
                outtake.intakeObject(),
                Commands.waitUntil(arm::isObjectInside),
                Commands.runOnce(()-> changeRobotState(States.ALGAE_IN_ARM))
                // No need to stop the intaking because we want to keep ahold of the algae. we'll stop only when we outtake.
        ));
        addCommand(States.ALGAE_IN_ARM, outtake.intakeObject());
        //endregion

        //region outtake algae
        addCommand(States.PREPARE_ALGAE_OUTTAKE, Commands.sequence(
                elevator.goToNetHeight(),
                arm.lookAtBarge(),
                Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal()),
                Commands.runOnce(()-> changeRobotState(States.ALGAE_OUTTAKE))
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
                        arm.lookDown(),
                        elevator.goToFloor(),
                        intakeAngle.lookDown(),
                        intake.stop(),
                        outtake.stop(),
                        swerve.reset()
                ),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal() && intakeAngle.atGoal()),
                Commands.runOnce(()-> changeRobotState(States.IDLE))
        ));
        addCommand(States.RESET, Commands.sequence(
                Commands.parallel(
                    elevator.reset(),
                    arm.reset()
                ),
                Commands.waitUntil(() -> elevator.isReset() && arm.isReset()),
                Commands.runOnce(()-> changeRobotState(States.IDLE))
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
