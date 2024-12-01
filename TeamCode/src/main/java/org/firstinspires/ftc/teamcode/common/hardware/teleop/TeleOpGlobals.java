package org.firstinspires.ftc.teamcode.common.hardware.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.common.hardware.auto.Globals;

@Config
public class TeleOpGlobals {

    //Slide States:
    public enum ExtendoFailState { // This will be used to track current on extendo motors.
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    TeleOpGlobals.ExtendoFailState extendoFailState = TeleOpGlobals.ExtendoFailState.GOOD;

    public enum ExtendoState {
        EXTENDING,
        RETRACTING,
        REST
    }

    TeleOpGlobals.ExtendoState extendoState = TeleOpGlobals.ExtendoState.REST;

    public enum OuttakeState {
        EXTENDING,
        RETRACTING,
        REST
    }

    TeleOpGlobals.OuttakeState outtakeState = TeleOpGlobals.OuttakeState.REST;

    //Intake States:
    public enum IntakeClawState {
        CUSTOM,
        OPEN,
        CLOSED,
        SPECIMEN,
        OPEN_TRANSFER
    }

    TeleOpGlobals.IntakeClawState intakeClawState = TeleOpGlobals.IntakeClawState.CLOSED;

    public enum IntakeRotationState {
        CUSTOM,
        REST,
        TRANSFER
    }

    TeleOpGlobals.IntakeRotationState intakeRotationState = TeleOpGlobals.IntakeRotationState.TRANSFER;

    public enum IntakeCoaxialState {
        CUSTOM,
        REST,
        TRANSFER,
        SPECIMEN,
        INTAKE
    }

    TeleOpGlobals.IntakeCoaxialState intakeCoaxialState = TeleOpGlobals.IntakeCoaxialState.REST;

    public enum FourBarState {
        CUSTOM,
        INTAKE,
        SCANNING, //optimal height for auto scanning
        LOW,
        TRANSFER,
        RESTING,
        BETWEEN,
        SPECIMEN
    }

    TeleOpGlobals.FourBarState fourBarState = TeleOpGlobals.FourBarState.RESTING;

    //Outtake States:
    public enum OuttakeClawState {
        CUSTOM,
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }

    TeleOpGlobals.OuttakeClawState outtakeClawState = TeleOpGlobals.OuttakeClawState.OPEN;

    public enum OuttakeRotationState {
        CUSTOM,
        EXIT,
        TRANSFER,
        SPECIMEN
    }

    TeleOpGlobals.OuttakeRotationState outtakeRotationState = TeleOpGlobals.OuttakeRotationState.TRANSFER;

    public enum OuttakeArmState {
        CUSTOM,
        TRANSFER,
        RAISING,
        BUCKET,
        DUNK,
        SPECIMEN,
        SPECIMEN_INTAKE,
        SPECIMEN_OUTTAKE
    }

    TeleOpGlobals.OuttakeArmState outtakeArmState = TeleOpGlobals.OuttakeArmState.TRANSFER;

    // Lift Subsystem Constants
    public static int LIFT_PARK_POS = 710;
    public static int LIFT_HIGH_POS = 2600;
    public static int LIFT_MID_POS = 1400;
    public static int LIFT_SPECIMEN_POS = 960;
    public static int LIFT_SPECIMEN_DROP = 160;
    public static int LIFT_RETRACT_POS = -5;

    //Extendo Subsystem Constants
    public static int EXTENDO_MAX_EXTENSION = 1766;
    public static int EXTENDO_MAX_RETRACTION = -5;
    public static double EXTENDO_MAX_TOLERANCE = 5;

    //Outtake Subsystem Constants
    public static double OUTTAKE_CLAW_OPEN = 0.7;
    public static double OUTTAKE_CLAW_TRANSFER = 0.7;
    public static double OUTTAKE_CLAW_CLOSED = 0.48;

    public static double OUTTAKE_ARM_TRANSFER = 0.04;
    public static double OUTTAKE_ARM_RAISING = 0.446;
    public static double OUTTAKE_ARM_BUCKET = 0.63;
    public static double OUTTAKE_ARM_DUNK = 0.62;
    public static double OUTTAKE_ARM_INTAKE = 0.75;

    public static double OUTTAKE_ARM_SPECIMEN = 0.66;
    public static double OUTTAKE_ARM_SPECIMEN_LOWER = 0.76;

    //Intake Subsystem Constants
    public static double INTAKE_CLAW_OPEN = 0.5;
    public static double INTAKE_CLAW_TRANSFER = 0.6;
    public static double INTAKE_CLAW_SPECIMEN = 0.6;
    public static double INTAKE_CLAW_CLOSED = 0;

    public static double INTAKE_ROTATION_REST = 0.39;

    public static double INTAKE_FOURBAR_INTAKE = 0.52;
    public static double INTAKE_FOURBAR_SCANNING = 0.59;
    public static double INTAKE_FOURBAR_LOW = 0.52;
    public static double INTAKE_FOURBAR_TRANSFER = 0.74;
    public static double INTAKE_FOURBAR_BETWEEN = 0.60;
    public static double INTAKE_FOURBAR_SPECIMEN = 0.675;
    public static double INTAKE_FOURBAR_RESTING = 0.675;

    public static double INTAKE_COAXIAL_RESTING = 0.915;
    public static double INTAKE_COAXIAL_INTAKE = 0.15;
    public static double INTAKE_COAXIAL_SPECIMEN = 0.435;
    public static double INTAKE_COAXIAL_TRANSFER = 0.93;

    public static final double[] intakeRotationPositions = { 0.39, 0.535, 0.68, 0.825 };

    //Auto Positions:
    public static Pose2d DEFAULT_START_POSE = new Pose2d(0, 0, Math.toRadians(0));

}
