package org.firstinspires.ftc.teamcode.common.hardware;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.CoaxialState.REST;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ExtendoFailState.GOOD;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.FourBarState.RESTING;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.IntakeClawState.CLOSED;
import static org.firstinspires.ftc.teamcode.common.hardware.Globals.OuttakeClawState.OPEN;

import com.acmerobotics.dashboard.config.Config;

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

    public enum OuttakeClawState {
        OPEN,
        CLOSED,
        OPEN_TRANSFER
    }
    Globals.OuttakeClawState outtakeClawState = OPEN;

    public enum FourBarState {
        INTAKE,
        LOW,
        TRANSFER,
        RESTING
    }
    Globals.FourBarState fourBarState = RESTING;

    public enum CoaxialState {
        REST
    }
    Globals.CoaxialState coaxialState = REST;

    // Lift Subsystem Constants
    //TODO: Tune these values:
    public static int LIFT_HIGH_POS = 597;
    public static int LIFT_MID_POS = 365;
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

    //Intake Subsystem Constants
    //TODO: Tune these values:
    public static double INTAKE_CLAW_OPEN = 0;
    public static double INTAKE_CLAW_TRANSFER = 0.3;
    public static double INTAKE_CLAW_CLOSED = 1;

    public static double INTAKE_FOURBAR_INTAKE = 0.036;
    public static double INTAKE_FOURBAR_LOW = 0.359; //In case we decide to drop intake further to grab
    public static double INTAKE_FOURBAR_TRANSFER = 0.472;
    public static double INTAKE_FOURBAR_RESTING = 0.33;

}
