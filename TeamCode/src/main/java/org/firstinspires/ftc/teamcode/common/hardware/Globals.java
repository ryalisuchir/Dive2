package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.opencv.core.Point;

@Config
public class Globals {
    public static double extendoStaticMax = 15;

    public static double visionOffset = -0.8;
    public static Point cameraCenter = new Point(320, 180);

    public static double EXTENDO_MAX_EXTENSION_TICKS_IN_INCHES = 472 / 21.26;

    public static Globals.ExtendoFailState extendoFailState = ExtendoFailState.GOOD;
    public static Globals.ExtendoState extendoState = ExtendoState.REST;
    public static Globals.OuttakeState outtakeState = OuttakeState.REST;
    public static Globals.IntakeClawState intakeClawState = IntakeClawState.CLOSED;
    public static Globals.IntakeRotationState intakeRotationState = IntakeRotationState.TRANSFER;
    public static Globals.IntakeCoaxialState intakeCoaxialState = IntakeCoaxialState.REST;
    public static Globals.FourBarState fourBarState = FourBarState.RESTING;
    public static Globals.OuttakeClawState outtakeClawState = OuttakeClawState.OPEN;
    public static Globals.OuttakeArmState outtakeArmState = OuttakeArmState.TRANSFER;

    // Lift Subsystem Constants
    public static int LIFT_HIGH_POS = 1850;
    public static int LIFT_MID_POS = 980;
    public static int LIFT_PARK_POS = 502;
    public static int LIFT_SPECIMEN_POS = 742;
    public static int LIFT_SPECIMEN_DROP = 220;
    public static int LIFT_RETRACT_POS = 0;
    public static double LIFT_MAX_TOLERANCE = 25;

    //Extendo Subsystem Constants
    public static int EXTENDO_MAX_EXTENSION = 472;
    public static int EXTENDO_MAX_RETRACTION = -5;
    public static double EXTENDO_MAX_TOLERANCE = 20;
    //Outtake Subsystem Constants
    public static double OUTTAKE_CLAW_OPEN = 0.7;
    public static double OUTTAKE_CLAW_TRANSFER = 0.7;
    public static double OUTTAKE_CLAW_CLOSED = 0.48;

    public static double OUTTAKE_ARM_TRANSFER = 0.21; //0.08, new was 0.19
    public static double OUTTAKE_ARM_RAISING = 0.46;
    public static double OUTTAKE_ARM_BUCKET = 0.66;
    public static double OUTTAKE_ARM_DUNK = 0.76;
    public static double OUTTAKE_ARM_INTAKE = 0.8;

    public static double OUTTAKE_ARM_SPECIMEN = 0.7;
    public static double OUTTAKE_ARM_SPECIMEN_LOWER = 0.85;
    //Intake Subsystem Constants
    public static double INTAKE_CLAW_OPEN = 0.5;
    public static double INTAKE_CLAW_TRANSFER = 0.18;
    public static double INTAKE_CLAW_SPECIMEN = 0.6;
    public static double INTAKE_CLAW_CLOSED = 0.13;

    public static double INTAKE_ROTATION_REST = 0.55;
    public static double INTAKE_ROTATION_TRANSFER = 0.55;

    public static double INTAKE_FOURBAR_INTAKE = 0.58;
    public static double INTAKE_FOURBAR_SCANNING = 0.675;
    public static double INTAKE_FOURBAR_LOW = 0.55;
    public static double INTAKE_FOURBAR_TRANSFER = 0.82; //0.8 //new was 0.89
    public static double INTAKE_FOURBAR_BETWEEN = 0.82; //0.63 //new was .89
    public static double INTAKE_FOURBAR_RESTING = 0.83;
    public static double INTAKE_FOURBAR_CAMERA_READING = 0.83;

    public static double INTAKE_COAXIAL_RESTING = 0.5;
    public static double INTAKE_COAXIAL_INTAKE = 0; //0.01
    public static double INTAKE_COAXIAL_TRANSFER = 0.66; //0.8 change to 0.66
    public static double INTAKE_COAXIAL_BETWEEN = 0.66; //0.75 new was 0.66
    public static double INTAKE_COAXIAL_CAMERA_READING = 0.29; //0.29

    //Auto Positions:
    public static Pose2d DEFAULT_START_POSE = new Pose2d(0, 0, Math.toRadians(0));
    public static Pose2d BLUE_CLOSE_START_POSE = new Pose2d(17, 64, Math.toRadians(270));
    public static Pose2d BLUE_SIDEWAYS_START_POSE = new Pose2d(41, 64, Math.toRadians(0));
    public static Pose2d BLUE_FAR_START_POSE = new Pose2d(-18, 66.43, Math.toRadians(-90));
    public static Pose2d BLUE_FAR_START_POSE_REVERSED = new Pose2d(-18, 66.43, Math.toRadians(90));


    //Slide States:
    public enum ExtendoFailState { // This will be used to track current on extendo motors.
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    public enum ExtendoState {
        EXTENDING,
        RETRACTING,
        REST
    }
    public enum OuttakeState {
        EXTENDING,
        RETRACTING,
        REST
    }

    //Intake States:
    public enum IntakeClawState {
        CUSTOM,
        OPEN,
        CLOSED,
        SPECIMEN,
        OPEN_TRANSFER
    }

    public enum IntakeRotationState {
        CUSTOM,
        REST,
        TRANSFER
    }

    public enum IntakeCoaxialState {
        CUSTOM,
        REST,
        TRANSFER,
        SPECIMEN,
        INTAKE,
        BETWEEN,
        CAMERA_READING
    }

    public enum FourBarState {
        CUSTOM,
        INTAKE,
        SCANNING, //optimal height for auto scanning
        LOW,
        TRANSFER,
        RESTING,
        BETWEEN,
        CAMERA_READING
    }

    //Outtake States:
    public enum OuttakeClawState {
        CUSTOM,
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }

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

}
