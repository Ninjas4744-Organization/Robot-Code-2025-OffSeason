package frc.robot;

public enum States {
    IDLE,

    INTAKE_CORAL,
    CORAL_IN_INTAKE, //the coral is set in the middle of the intake


    PREPARE_CORAL_OUTTAKE_LOW, //For L1
    CORAL_OUTTAKE_LOW,


    ARM_INTAKE, //transfer the coral from the intake from the bottom to the arm
    CORAL_IN_ARM,//the coral is set in the arm
    // automatic driving for when we want to score a coral in L2-L4 only!!
    DRIVE_TOWARDS_LEFT_REEF,
    DRIVE_TOWARDS_RIGHT_REEF,
    PREPARE_CORAL_OUTTAKE_HIGH, //For L2-L4
    CORAL_OUTTAKE_HIGH,


    INTAKE_ALGAE_LOW, //moving the arm and grabbing algae from the floor
    INTAKE_ALGAE_HIGH, //moving the arm and grabbing algae from the reef
    ALGAE_IN_ARM, //Finished the initial grab of the algae
    PREPARE_ALGAE_OUTTAKE, //towards the barge
    ALGAE_OUTTAKE,

    CLOSE,
    RESET,

    PREPARE_CLIMB,
    CLIMB,
    CLIMBED
}
