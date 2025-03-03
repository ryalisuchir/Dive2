package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;


import org.opencv.core.Point;

@Config
public class Globals {
    public static double extendoStaticMax = 15;

    public static double visionOffset = -1.0;
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
    public static int LIFT_HIGH_POS = 1120;
    public static int LIFT_AUTO_HIGH_POS = 1080;
    public static int LIFT_MID_POS = 550;
    public static int LIFT_PARK_POS = 200;
    public static int LIFT_SPECIMEN_POS = 330; //510
    public static int LIFT_RETRACT_POS = 0;
    public static double LIFT_MAX_TOLERANCE = 25;

    //Extendo Subsystem Constants
    public static int EXTENDO_MAX_EXTENSION = 472;
    public static int EXTENDO_MAX_RETRACTION = -10;
    public static double EXTENDO_MAX_TOLERANCE = 25;
    //Outtake Subsystem Constants
    public static double OUTTAKE_CLAW_OPEN = 0.66;
    public static double OUTTAKE_CLAW_CLOSED = 0.52;

    public static double OUTTAKE_ARM_TRANSFER = 0.21; //0.08, new was 0.19
    public static double OUTTAKE_ARM_RAISING = 0.46;
    public static double OUTTAKE_ARM_BUCKET = 0.66;
    public static double OUTTAKE_ARM_DUNK = 0.76;

    public static double OUTTAKE_ARM_SPECIMEN_INTAKE = 0.8;
    public static double OUTTAKE_ARM_SPECIMEN_DROPOFF = 0.57;
    public static double OUTTAKE_ARM_SPECIMEN_SMASH = 0.8;

    //Intake Subsystem Constants
    public static double INTAKE_CLAW_OPEN = 0.60;
    public static double INTAKE_CLAW_TRANSFER = 0.3;
    public static double INTAKE_CLAW_CLOSED = 0.17;

    public static double INTAKE_ROTATION_REST = 0.56;
    public static double INTAKE_ROTATION_TRANSFER = 0.56;

    public static double INTAKE_FOURBAR_INTAKE = 0.58;
    public static double INTAKE_FOURBAR_SCANNING = 0.675;
    public static double INTAKE_FOURBAR_TRANSFER = 0.83;
    public static double INTAKE_FOURBAR_RESTING = 0.83;
    public static double INTAKE_FOURBAR_CAMERA_READING = 0.83;

    public static double INTAKE_COAXIAL_RESTING = 0.5;
    public static double INTAKE_COAXIAL_INTAKE = 0; //0.01
    public static double INTAKE_COAXIAL_TRANSFER = 0.66; //0.8 change to 0.66
    public static double INTAKE_COAXIAL_CAMERA_READING = 0.29; //0.29

    //Auto Positions:
    public static Pose2d DEFAULT_START_POSE = new Pose2d(0, 0, Math.toRadians(0));
    public static Pose2d BLUE_SIDEWAYS_START_POSE = new Pose2d(41, 64, Math.toRadians(0));
    public static Pose2d BLUE_FAR_START_POSE = new Pose2d(-7, 66.43, Math.toRadians(-90));


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
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }

    public enum IntakeRotationState {
        CUSTOM,
        REST,
        TRANSFER
    }

    public enum IntakeCoaxialState {
        REST,
        TRANSFER,
        INTAKE,
        CAMERA_READING
    }

    public enum FourBarState {
        CUSTOM,
        INTAKE,
        SCANNING, //optimal height for auto scanning
        TRANSFER,
        RESTING,
        CAMERA_READING
    }

    //Outtake States:
    public enum OuttakeClawState {
        OPEN,
        CLOSED
    }

    public enum OuttakeArmState {
        CUSTOM,
        TRANSFER,
        RAISING,
        BUCKET,
        DUNK,
        SPECIMEN_INTAKE,
        SPECIMEN_OUTTAKE,
        SPECIMEN_SMASH
    }

}
