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
        RESTING
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
    public static int LIFT_HIGH_POS = 4180;
    public static int LIFT_MID_POS = 2284;
    public static int LIFT_SPECIMEN_POS = 1038;
    public static int LIFT_RETRACT_POS = -5;

    //Extendo Subsystem Constants
    public static int EXTENDO_MAX_EXTENSION = 1266;
    public static int EXTENDO_MAX_RETRACTION = 0;
    public static int EXTENDO_SAMPLE_1 = 250;
    public static int EXTENDO_SAMPLE_2 = 250;
    public static int EXTENDO_SAMPLE_3 = 250;

    //Outtake Subsystem Constants
    public static double OUTTAKE_CLAW_OPEN = 0.2; //done
    public static double OUTTAKE_CLAW_TRANSFER = 0.25; //done
    public static double OUTTAKE_CLAW_CLOSED = 0.04; //done

    public static double OUTTAKE_ROTATION_TRANSFER = 0.88;
    public static double OUTTAKE_ROTATION_SPECIMEN = 0.5;
    public static double OUTTAKE_ROTATION_EXIT = 0.88; //done

    public static double OUTTAKE_ARM_TRANSFER = 0.15; //done
    public static double OUTTAKE_ARM_RAISING = 0.53; //done
    public static double OUTTAKE_ARM_BUCKET = 0.77; //done
    public static double OUTTAKE_ARM_DUNK = 0.77; //done
    public static double OUTTAKE_ARM_SPECIMEN = 0.53; //done

    //Intake Subsystem Constants
    public static double INTAKE_CLAW_OPEN = 0.2;
    public static double INTAKE_CLAW_TRANSFER = 0.3;
    public static double INTAKE_CLAW_CLOSED = 1;

    public static double INTAKE_ROTATION_REST = 0;
    public static double INTAKE_ROTATION_TRANSFER = 0.5;
    public static double INTAKE_ROTATION_AUTO_1 = 0.5;
    public static double INTAKE_ROTATION_AUTO_2 = 0.6;
    public static double INTAKE_ROTATION_AUTO_3 = 0.8;

    public static double INTAKE_FOURBAR_INTAKE = 0.57;
    public static double INTAKE_FOURBAR_SCANNING = 0.65;
    public static double INTAKE_FOURBAR_LOW = 0.54; //In case we decide to drop intake further to grab
    public static double INTAKE_FOURBAR_TRANSFER = 0.72;
    public static double INTAKE_FOURBAR_RESTING = 0.83;

    public static double INTAKE_COAXIAL_RESTING = 0.5;
    public static double INTAKE_COAXIAL_INTAKE = 0.2;
    public static double INTAKE_COAXIAL_TRANSFER = 1;

    public static Pose2d DEFAULT_START_POSE = new Pose2d(0, 0, Math.toRadians(0.00));
    public static Pose2d BLUE_CLOSE_START_POSE = new Pose2d(30, 63, Math.toRadians(270.00));

}
