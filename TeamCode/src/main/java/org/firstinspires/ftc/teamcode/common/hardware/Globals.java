package org.firstinspires.ftc.teamcode.common.hardware;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoFailState.GOOD;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.FourBarState.RESTING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.IntakeClawState.CLOSED;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeClawState.OPEN;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeRotationState.TRANSFER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Globals {
    public static boolean AUTO = false;
    public static boolean IS_PARKING = false;

    public static double extendoStaticMax = 7; //TODO: tune this by pushing hand against extendo

    public enum ExtendoFailState { // This will be used to track current on extendo motors.
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }
    Globals.ExtendoFailState extendoFailState = GOOD;

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

    public enum IntakeClawState {
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }
    Globals.IntakeClawState intakeClawState = CLOSED;

    public enum IntakeRotationState {
        REST,
        TRANSFER,
        AUTO_1,
        AUTO_2,
        AUTO_3
    }
    Globals.IntakeRotationState intakeRotationState = IntakeRotationState.TRANSFER;

    public enum IntakeCoaxialState {
        REST,
        TRANSFER,
        INTAKE
    }
    Globals.IntakeCoaxialState intakeCoaxialState = IntakeCoaxialState.REST;

    public enum OuttakeClawState {
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }
    Globals.OuttakeClawState outtakeClawState = OPEN;

    public enum OuttakeRotationState {
        EXIT,
        TRANSFER,
        SPECIMEN
    }
    Globals.OuttakeRotationState outtakeRotationState = TRANSFER;

    public enum FourBarState {
        INTAKE,
        SCANNING, //optimal height for auto scanning
        LOW,
        TRANSFER,
        RESTING
    }
    Globals.FourBarState fourBarState = RESTING;

    // Lift Subsystem Constants
    //TODO: Tune these values:
    public static int LIFT_HIGH_POS = 597;
    public static int LIFT_MID_POS = 365;
    public static int LIFT_SPECIMEN_POS = 400;
    public static int LIFT_RETRACT_POS = -5;

    //Extendo Subsystem Constants
    //TODO: Tune these values:
    public static int EXTENDO_MAX_EXTENSION = 700;
    public static int EXTENDO_MAX_RETRACTION = -5;
    public static int EXTENDO_SAMPLE_1 = 250;

    //Outtake Subsystem Constants
    //TODO: Tune these values
    public static double OUTTAKE_CLAW_OPEN = 0;
    public static double OUTTAKE_CLAW_TRANSFER = 0.3;
    public static double OUTTAKE_CLAW_CLOSED = 1;

    public static double OUTTAKE_ROTATION_TRANSFER = 0;
    public static double OUTTAKE_ROTATION_SPECIMEN = 0.5;
    public static double OUTTAKE_ROTATION_EXIT = 1;

    //Intake Subsystem Constants
    //TODO: Tune these values:
    public static double INTAKE_CLAW_OPEN = 0;
    public static double INTAKE_CLAW_TRANSFER = 0.3;
    public static double INTAKE_CLAW_CLOSED = 1;

    public static double INTAKE_ROTATION_REST = 0;
    public static double INTAKE_ROTATION_TRANSFER = 0.5;
    public static double INTAKE_ROTATION_AUTO_1 = 0.5;
    public static double INTAKE_ROTATION_AUTO_2 = 0.6;
    public static double INTAKE_ROTATION_AUTO_3 = 0.8;

    public static double INTAKE_FOURBAR_INTAKE = 0.58;
    public static double INTAKE_FOURBAR_SCANNING = 0.7;
    public static double INTAKE_FOURBAR_LOW = 0.4; //In case we decide to drop intake further to grab
    public static double INTAKE_FOURBAR_TRANSFER = 1;
    public static double INTAKE_FOURBAR_RESTING = 0.72;

    public static double INTAKE_COAXIAL_RESTING = 0;
    public static double INTAKE_COAXIAL_INTAKE = 0.2;
    public static double INTAKE_COAXIAL_TRANSFER = 1;

    public static Pose2d DEFAULT_START_POSE = new Pose2d(0, 0, Math.toRadians(0.00));
    public static Pose2d BLUE_CLOSE_START_POSE = new Pose2d(30, 63, Math.toRadians(270.00));

}
