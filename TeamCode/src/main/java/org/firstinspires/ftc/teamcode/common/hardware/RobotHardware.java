package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.camera.AngleDetection;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.Intake4BarSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeCoaxialSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.intake.IntakeRotationSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.outtake.OuttakeRotationSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.DepositSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.slides.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.common.roadrunner.PinpointDrive;
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotHardware {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear; //Drivetrain motors
    public DcMotorEx leftLift, rightLift; //Outtake lift motors
    public DcMotorEx extendoMotor; //Intake extension motor
    //1 is left
    //2 is right
    public Servo intakeRotation, intakeClaw, intakeCoaxialLeft, intakeCoaxialRight, intake4BarLeft, intake4BarRight; //Intake servos
    public Servo outtakeRotation, outtakeArmLeft, outtakeArmRight, outtakeClaw; //Outtake servos
    public PhotonLynxVoltageSensor batteryVoltageSensor;

    public Intake4BarSubsystem intake4BarSubsystem;
    public IntakeClawSubsystem intakeClawSubsystem;
    public IntakeCoaxialSubsystem intakeCoaxialSubsystem;
    public IntakeRotationSubsystem intakeRotationSubsystem;
    public OuttakeArmSubsystem outtakeArmSubsystem;
    public OuttakeClawSubsystem outtakeClawSubsystem;
    public OuttakeRotationSubsystem outtakeRotationSubsystem;
    public DepositSubsystem depositSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public DriveSubsystem driveSubsystem;

    private double voltage = 0.0;
    public OpenCvWebcam sampleCamera;
    AngleDetection sampleDetection;

    public RobotHardware(HardwareMap hardwareMap, Pose2d initialPose) {
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
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Setting all servos:
        intakeRotation = hardwareMap.get(Servo.class, "intakeRotation");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeCoaxialLeft = hardwareMap.get(Servo.class, "intakeCoaxial1");
        intakeCoaxialRight = hardwareMap.get(Servo.class, "intakeCoaxial2");
        intake4BarLeft = hardwareMap.get(Servo.class, "intake4Bar1");
        intake4BarRight = hardwareMap.get(Servo.class, "intake4Bar2");
        outtakeRotation = hardwareMap.get(Servo.class, "outtakeRotation");
        outtakeArmLeft = hardwareMap.get(Servo.class, "leftOuttakeArm");
        outtakeArmRight = hardwareMap.get(Servo.class, "rightOuttakeArm");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        intake4BarRight.setDirection(Servo.Direction.REVERSE);
        intakeCoaxialRight.setDirection(Servo.Direction.REVERSE);

        intake4BarSubsystem = new Intake4BarSubsystem(intake4BarLeft, intake4BarRight);
        intakeClawSubsystem = new IntakeClawSubsystem(intakeClaw);
        intakeCoaxialSubsystem = new IntakeCoaxialSubsystem(intakeCoaxialLeft, intakeCoaxialRight);
        intakeRotationSubsystem = new IntakeRotationSubsystem(intakeRotation);
        outtakeArmSubsystem = new OuttakeArmSubsystem(outtakeArmLeft, outtakeArmRight);
        outtakeClawSubsystem = new OuttakeClawSubsystem(outtakeClaw);
        outtakeRotationSubsystem = new OuttakeRotationSubsystem(outtakeRotation);
        depositSubsystem = new DepositSubsystem(leftLift, rightLift);
        extendoSubsystem = new ExtendoSubsystem(extendoMotor);
        driveSubsystem = new DriveSubsystem(new PinpointDrive(hardwareMap, initialPose), false);

//        CommandScheduler.getInstance().registerSubsystem(intake4BarSubsystem, intakeClawSubsystem, intakeCoaxialSubsystem, intakeRotationSubsystem, outtakeArmSubsystem, outtakeClawSubsystem, outtakeRotationSubsystem, depositSubsystem, extendoSubsystem, driveSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intake4BarSubsystem, intakeClawSubsystem, intakeCoaxialSubsystem, intakeRotationSubsystem,driveSubsystem, extendoSubsystem);
        //        if (Globals.AUTO) {
//            sampleDetection = new AngleDetection();
//            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            sampleCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//            sampleCamera.setPipeline(sampleDetection = new AngleDetection());
//            sampleCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    sampleCamera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//                }
//
//                @Override
//                public void onError(int errorCode) {
//                }
//            });
//
//            FtcDashboard.getInstance().startCameraStream(sampleCamera, 60);
//        }

    }

//    public double getSampleAngle() {
//        if (!Double.isNaN(sampleDetection.getAngleOfGreenSample())) {
//            return sampleDetection.getAngleOfGreenSample();
//        } else {
//            return -1;
//        }
//    }
//
//    public Point getSamplePosition() {
//        if (sampleDetection.getGreenSampleCoordinates() != null) {
//            return sampleDetection.getGreenSampleCoordinates();
//        } else {
//            return new Point(0,0);
//        }
//    }
//
//    public void stopCameraStream() {
//        sampleCamera.closeCameraDeviceAsync(() -> System.out.println("Stopped camera."));
//    }

    public double getVoltage() {
        return voltage;
    }

}