package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.NinjasLib.dataclasses.ControlConstants;
import frc.lib.NinjasLib.dataclasses.ControllerConstants;
import frc.lib.NinjasLib.dataclasses.RealControllerConstants.SimpleControllerConstants;

public class Constants {
    public static enum RobotMode {
        /**
         * Running on a real robot
         */
        REAL,

        /** Running on a simulator */
        SIM,

        /** Replaying from a log file */
        REPLAY
    }

    /* General */
    public static final RobotMode kSimMode = RobotMode.SIM;
    public static final RobotMode kCurrentMode = Robot.isReal() ? RobotMode.REAL : kSimMode;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /* Subsystems */
    public static final ControllerConstants kArmControllerConstants = new ControllerConstants();

    static {
        /* Base */
        kArmControllerConstants.real.main.id = 20;
        kArmControllerConstants.real.main.inverted = false;
        kArmControllerConstants.real.currentLimit = 80;
        kArmControllerConstants.real.isBrakeMode = true;

        /* Followers */
        kArmControllerConstants.real.followers = new SimpleControllerConstants[1];
        kArmControllerConstants.real.followers[0] = new SimpleControllerConstants();
        kArmControllerConstants.real.followers[0].id = 21;
        kArmControllerConstants.real.followers[0].inverted = true;

        /* Control */
        kArmControllerConstants.real.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
        kArmControllerConstants.real.gearRatio = 50;
        kArmControllerConstants.real.conversionFactor = 2 * Math.PI;
        kArmControllerConstants.real.homePosition = Units.degreesToRadians(-60);
        kArmControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(1.5);

        /* Soft Limits */
        kArmControllerConstants.real.minSoftLimit = Units.degreesToRadians(-60);
        kArmControllerConstants.real.maxSoftLimit = Units.degreesToRadians(240);

        /* Hard Limit */
        kArmControllerConstants.real.isLimitSwitch = true;
        kArmControllerConstants.real.limitSwitchID = 2;
        kArmControllerConstants.real.limitSwitchDirection = -1;
        kArmControllerConstants.real.limitSwitchAutoStopReset = true;
        kArmControllerConstants.real.limitSwitchInverted = true;

        /* Simulation */
        kArmControllerConstants.motorType = DCMotor.getKrakenX60(2);
    }

    /* Positions */
    public enum ArmPositions {
        Close(0),
        Open(180),
        Mid(90);

        final double angle;

        ArmPositions(double angle) {
            this.angle = angle;
        }

        public double get() {
            return angle;
        }
    }

    public enum ElevatorPositions {
        Close(0),
        Open(1.5),
        Mid(0.75);

        final double height;

        ElevatorPositions(double height) {
            this.height = height;
        }

        public double get() {
            return height;
        }
    }
}
