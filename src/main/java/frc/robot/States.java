package frc.robot;

public enum States {
    IDLE,

    INTAKE_CORAL,
    CORAL_IN_INTAKE, //the coral is set in the middle of the intake
    PREPARE_CORAL_OUTTAKE_LOW, //For L1
    CORAL_OUTTAKE_LOW,
    ARM_INTAKE, //transfer the coral from the intake from the bottom to the arm
    CORAL_IN_ARM,//the coral is set in the arm
    PREPARE_CORAL_OUTTAKE_HIGH, //For L2-L4
    CORAL_OUTTAKE_HIGH,

    PREPARE_INTAKE_ALGAE_HIGH, //grabbing algae from the reef
    PREPARE_INTAKE_ALGAE_LOW, //grabbing algae from the floor
    INTAKE_ALGAE, //Grabbing algae from the floor or the reef
    PREPARE_ALGAE_OUTTAKE, //towards the barge
    ALGAE_OUTTAKE,

    CLOSE,
    RESET
}
