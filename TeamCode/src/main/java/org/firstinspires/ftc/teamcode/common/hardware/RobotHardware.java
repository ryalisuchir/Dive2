package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.Intake4BarSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeCoaxialSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeRotationSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.roadrunner.PinpointDrive;

import java.util.List;

public class RobotHardware {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear; //Drivetrain motors
    public DcMotorEx leftLift, rightLift; //Outtake lift motors
    public DcMotorEx extendoMotor; //Intake extension motor
    //1 is left
    //2 is right
    public ServoImplEx intakeRotation, intakeClaw, intakeCoaxialLeft, intakeCoaxialRight, intake4BarLeft, intake4BarRight; //Intake servos
    public ServoImplEx outtakeArmLeft, outtakeArmRight, outtakeClaw; //Outtake servos
    public CRServo leftHang, rightHang;
    public Intake4BarSubsystem intake4BarSubsystem;
    public IntakeClawSubsystem intakeClawSubsystem;
    public IntakeCoaxialSubsystem intakeCoaxialSubsystem;
    public IntakeRotationSubsystem intakeRotationSubsystem;
    public OuttakeArmSubsystem outtakeArmSubsystem;
    public OuttakeClawSubsystem outtakeClawSubsystem;
    public DepositSubsystem depositSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public DriveSubsystem driveSubsystem;
    public PinpointDrive pinpointDrive;
    public HangSubsystem hangSubsystem;
    List<LynxModule> allHubs;


    public RobotHardware(HardwareMap hardwareMap, Pose2d initialPose, boolean autoBoolean) {
        //Optimizing Loop Times:
        allHubs = hardwareMap.getAll(LynxModule.class);

        //Configuration of all motors:
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");
        //Reversing motors:
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);

        rightLift.setDirection(DcMotorEx.Direction.REVERSE);
        extendoMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //Setting all motors to stop:
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Resetting encoders (RR will take care of drivetrain motors):
        if (autoBoolean) {
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        //Creating a P Controller requires these motors to be run without an encoder:
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Setting all servos:
        intakeRotation = hardwareMap.get(ServoImplEx.class, "intakeRotation");
        intakeClaw = hardwareMap.get(ServoImplEx.class, "intakeClaw");
        intakeCoaxialLeft = hardwareMap.get(ServoImplEx.class, "intakeCoaxial1");
        intakeCoaxialRight = hardwareMap.get(ServoImplEx.class, "intakeCoaxial2");
        intake4BarLeft = hardwareMap.get(ServoImplEx.class, "intake4Bar1");
        intake4BarRight = hardwareMap.get(ServoImplEx.class, "intake4Bar2");
        leftHang = hardwareMap.get(CRServo.class, "leftHang");
        rightHang = hardwareMap.get(CRServo.class, "rightHang");
        outtakeArmLeft = hardwareMap.get(ServoImplEx.class, "leftOuttakeArm");
        outtakeArmRight = hardwareMap.get(ServoImplEx.class, "rightOuttakeArm");
        outtakeClaw = hardwareMap.get(ServoImplEx.class, "outtakeClaw");

        //Setting all the doubled-up right-side servos to be reversed to prevent gear slippage:
        outtakeArmRight.setDirection(ServoImplEx.Direction.REVERSE);
        intake4BarRight.setDirection(ServoImplEx.Direction.REVERSE);
        intakeCoaxialRight.setDirection(ServoImplEx.Direction.REVERSE);
        rightHang.setDirection(DcMotorSimple.Direction.REVERSE);


        //Initializing all subsystems:
        intake4BarSubsystem = new Intake4BarSubsystem(intake4BarLeft, intake4BarRight);
        intakeClawSubsystem = new IntakeClawSubsystem(intakeClaw);
        intakeCoaxialSubsystem = new IntakeCoaxialSubsystem(intakeCoaxialLeft, intakeCoaxialRight);
        intakeRotationSubsystem = new IntakeRotationSubsystem(intakeRotation);
        outtakeArmSubsystem = new OuttakeArmSubsystem(outtakeArmLeft, outtakeArmRight);
        outtakeClawSubsystem = new OuttakeClawSubsystem(outtakeClaw);
        depositSubsystem = new DepositSubsystem(leftLift, rightLift);
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        hangSubsystem = new HangSubsystem(leftHang, rightHang);
        pinpointDrive = new PinpointDrive(hardwareMap, initialPose);
        driveSubsystem = new DriveSubsystem(pinpointDrive, false);

        //Registering all subsystems: //test to make sure push went through
        CommandScheduler.getInstance().registerSubsystem(
                //Intakes:
                intake4BarSubsystem,
                intakeClawSubsystem,
                intakeCoaxialSubsystem,
                intakeRotationSubsystem,
                //Outtakes:
                outtakeArmSubsystem,
                outtakeClawSubsystem,
                //Slides:
                depositSubsystem,
                extendoSubsystem,
                //Drivetrain:
                driveSubsystem,
                //Hang:
                hangSubsystem
        );
    }

    public void clearCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void systemLoop(Telemetry telemetryInput) {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        CommandScheduler.getInstance().run();
        extendoSubsystem.extendoSlidesLoop();
        depositSubsystem.outtakeSlidesLoop();
        extendoSubsystem.currentLoop();
        driveSubsystem.updatePoseEstimate();
        telemetryInput.addData("Robot Position: ", pinpointDrive.pose.position);
        telemetryInput.addData("Extendo State: ", Globals.extendoState);
        telemetryInput.addData("Outtake State: ", Globals.outtakeState);
        telemetryInput.addData("Intake Rotation State: ", Globals.intakeRotationState);
        telemetryInput.addData("Intake Coaxial State: ", Globals.intakeCoaxialState);
        telemetryInput.addData("Intake Claw State: ", Globals.intakeClawState);
        telemetryInput.addData("FourBar State: ", Globals.fourBarState);
        telemetryInput.addData("Outtake Arm State: ", Globals.outtakeArmState);
        telemetryInput.addData("Outtake Claw State: ", Globals.outtakeClawState);

        if (Globals.extendoFailState == Globals.ExtendoFailState.FAILED_EXTEND) {
            Log.i("Extendo Failed:", "FAILED_EXTENSION");
        }

        if (Globals.extendoFailState == Globals.ExtendoFailState.FAILED_RETRACT) {
            Log.i("Extendo Failed:", "FAILED_RETRACTION");
        }

        telemetryInput.update();
    }

}