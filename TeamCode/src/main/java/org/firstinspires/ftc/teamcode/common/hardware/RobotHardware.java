package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

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
import org.firstinspires.ftc.teamcode.lib.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.lib.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.lib.roadrunner.MecanumDrive;

import java.util.List;

public class RobotHardware {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear; //Drivetrain motors
    public DcMotorEx leftLift, rightLift; //Outtake lift motors
    public DcMotorEx extendoMotor; //Intake extension motor
    //1 is left
    //2 is right
    public ServoImplEx intakeRotation, intakeClaw, intakeCoaxialLeft, intakeCoaxialRight, intake4BarLeft, intake4BarRight; //Intake servos
    public ServoImplEx outtakeArmLeft, outtakeArmRight, outtakeClaw; //Outtake servos
    public double voltage;
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
    public MecanumDrive mecanumDrive;
    public Follower follower;
    public HangSubsystem hangSubsystem;
    List<LynxModule> allHubs;
    LynxModule CONTROL_HUB, EXPANSION_HUB;


    public RobotHardware(HardwareMap hardwareMap, Pose2d initialPose, boolean autoBoolean) {
        //Optimizing Loop Times:
        allHubs = hardwareMap.getAll(LynxModule.class);

        if (allHubs.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(allHubs.get(0).getSerialNumber())) {
            CONTROL_HUB = allHubs.get(0);
            EXPANSION_HUB = allHubs.get(1);
        } else {
            CONTROL_HUB = allHubs.get(1);
            EXPANSION_HUB = allHubs.get(0);
        }

        //Configuration of all motors:
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

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

        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        driveSubsystem = new DriveSubsystem(mecanumDrive, false);

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
                //Drive:
                driveSubsystem,
                //Hang:
                hangSubsystem
        );
    }

    public RobotHardware(HardwareMap hardwareMap, Pose initialPose, boolean autoBoolean) {
        //Optimizing Loop Times:
        allHubs = hardwareMap.getAll(LynxModule.class);

        if (allHubs.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(allHubs.get(0).getSerialNumber())) {
            CONTROL_HUB = allHubs.get(0);
            EXPANSION_HUB = allHubs.get(1);
        } else {
            CONTROL_HUB = allHubs.get(1);
            EXPANSION_HUB = allHubs.get(0);
        }

        //Configuration of all motors:
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(initialPose);

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
                //Hang:
                hangSubsystem
        );
    }


    public void clearCache() {
        for (LynxModule hub : allHubs) {
            if (
                    hub.getDeviceName().equals("Servo Hub") ||
                            hub.getDeviceName().equals("Servo Hub 1")
            ) return;
            CONTROL_HUB.clearBulkCache();
            EXPANSION_HUB.clearBulkCache();
        }
    }

}