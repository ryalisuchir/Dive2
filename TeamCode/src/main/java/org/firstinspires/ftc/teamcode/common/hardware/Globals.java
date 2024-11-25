package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Globals {
    public static boolean AUTO = false;
    public static boolean IS_PARKING = false;

    public static double extendoStaticMax = 7; //TODO: tune this by pushing hand against extendo

    //Slide States:
    public enum ExtendoFailState { // This will be used to track current on extendo motors.
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }
    Globals.ExtendoFailState extendoFailState = ExtendoFailState.GOOD;

    public enum ExtendoState {
        EXTENDING,
        RETRACTING,
        REST
    }
    Globals.ExtendoState extendoState = ExtendoState.REST;

    public enum OuttakeState {
        EXTENDING,
        RETRACTING,
        REST
    }
    Globals.OuttakeState outtakeState = OuttakeState.REST;

    //Intake States:
    public enum IntakeClawState {
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }
    Globals.IntakeClawState intakeClawState = IntakeClawState.CLOSED;

    public enum IntakeRotationState {
        REST,
        TRANSFER,
        CUSTOM_AUTO
    }
    Globals.IntakeRotationState intakeRotationState = IntakeRotationState.TRANSFER;

    public enum IntakeCoaxialState {
        REST,
        TRANSFER,
        INTAKE
    }
    Globals.IntakeCoaxialState intakeCoaxialState = IntakeCoaxialState.REST;

    public enum FourBarState {
        INTAKE,
        SCANNING, //optimal height for auto scanning
        LOW,
        TRANSFER,
        RESTING,
        BETWEEN
    }
    Globals.FourBarState fourBarState = FourBarState.RESTING;

    //Outtake States:
    public enum OuttakeClawState {
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }
    Globals.OuttakeClawState outtakeClawState = OuttakeClawState.OPEN;

    public enum OuttakeRotationState {
        EXIT,
        TRANSFER,
        SPECIMEN
    }
    Globals.OuttakeRotationState outtakeRotationState = OuttakeRotationState.TRANSFER;

    public enum OuttakeArmState {
        TRANSFER,
        RAISING,
        BUCKET,
        DUNK,
        SPECIMEN
    }
    Globals.OuttakeArmState outtakeArmState = OuttakeArmState.TRANSFER;



    // Lift Subsystem Constants
    public static int LIFT_HIGH_POS = 2650;
    public static int LIFT_MID_POS = 1400;
    public static int LIFT_SPECIMEN_POS = 1212;
    public static int LIFT_RETRACT_POS = 0;
    public static double LIFT_MAX_TOLERANCE = 5;

    //Extendo Subsystem Constants
    public static int EXTENDO_MAX_EXTENSION = 1766;
    public static int EXTENDO_MAX_RETRACTION = 0;
    public static double EXTENDO_MAX_TOLERANCE = 5;

    //Outtake Subsystem Constants
    public static double OUTTAKE_CLAW_OPEN = 0.25;
    public static double OUTTAKE_CLAW_TRANSFER = 0.4;
    public static double OUTTAKE_CLAW_CLOSED = 0.05;

    public static double OUTTAKE_ROTATION_TRANSFER = 0.60;
    public static double OUTTAKE_ROTATION_SPECIMEN = 0.60;
    public static double OUTTAKE_ROTATION_EXIT = 0.60;

    public static double OUTTAKE_ARM_TRANSFER = 0.04;
    public static double OUTTAKE_ARM_RAISING = 0.425;
    public static double OUTTAKE_ARM_BUCKET = 0.66;
    public static double OUTTAKE_ARM_DUNK = 0.66;
    public static double OUTTAKE_ARM_SPECIMEN = 0.66;

    //Intake Subsystem Constants
    public static double INTAKE_CLAW_OPEN = 0.5;
    public static double INTAKE_CLAW_TRANSFER = 0.6;
    public static double INTAKE_CLAW_CLOSED = 0;

    public static double INTAKE_ROTATION_REST = 0;
    public static double INTAKE_ROTATION_TRANSFER = 0.5;
    public static double INTAKE_ROTATION_AUTO_1 = 0.5;
    public static double INTAKE_ROTATION_AUTO_2 = 0.6;
    public static double INTAKE_ROTATION_AUTO_3 = 0.8;

    public static double INTAKE_FOURBAR_INTAKE = 0.57;
    public static double INTAKE_FOURBAR_SCANNING = 0.6;
    public static double INTAKE_FOURBAR_LOW = 0.555;
    public static double INTAKE_FOURBAR_TRANSFER = 0.675;
    public static double INTAKE_FOURBAR_BETWEEN = 0.63;
    public static double INTAKE_FOURBAR_RESTING = 0.675;

    public static double INTAKE_COAXIAL_RESTING = 0.915;
    public static double INTAKE_COAXIAL_INTAKE = 0.15;
    public static double INTAKE_COAXIAL_TRANSFER = 0.915;

    //Auto Positions:
    public static Pose2d DEFAULT_START_POSE = new Pose2d(0, 0, Math.toRadians(0.00));
    public static Pose2d BLUE_CLOSE_START_POSE = new Pose2d(17.67, 65.73, Math.toRadians(-90.00));

    //Tele-Op Positions:
    public static double INTAKE_ROTATION_ZERO = 0.0;
    public static double INTAKE_ROTATION_MAX = 1.0;
    public static double INTAKE_ROTATION_INCREMENT = 0.25;

}
