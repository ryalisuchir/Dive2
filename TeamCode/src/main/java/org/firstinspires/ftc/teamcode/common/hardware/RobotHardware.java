package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.firstinspires.ftc.teamcode.common.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.roadrunner.PinpointDrive;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class RobotHardware {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear; //Drivetrain motors
    public DcMotorEx leftLift, rightLift; //Outtake lift motors
    public DcMotorEx extendoMotor; //Intake extension motor
    public Servo intakeRotation, intakeClaw, intakeCoaxial, intake4Bar; //Intake servos
    public Servo outtakeRotation, leftOuttakeArm, rightOuttakeArm, outtakeClaw; //Outtake servos
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
    private ElapsedTime voltageTimer;
    public OpenCvWebcam sampleCamera;
    AngleDetection sampleDetection;

    private static RobotHardware instance = null;
    public boolean enabled;
    private HardwareMap hardwareMap;
    GoBildaPinpointDriverRR pinPointDriver;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        voltageTimer = new ElapsedTime();
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

        //Setting all servos:
        Servo intakeRotation = hardwareMap.get(Servo.class, "intakeRotation");
        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        Servo intakeCoaxial = hardwareMap.get(Servo.class, "intakeCoaxial");
        Servo intake4Bar = hardwareMap.get(Servo.class, "intake4Bar");
        Servo outtakeRotation = hardwareMap.get(Servo.class, "outtakeRotation");
        Servo leftOuttakeArm = hardwareMap.get(Servo.class, "leftOuttakeArm");
        Servo rightOuttakeArm = hardwareMap.get(Servo.class, "rightOuttakeArm");
        Servo outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        intake4BarSubsystem = new Intake4BarSubsystem(getInstance());
        intakeClawSubsystem = new IntakeClawSubsystem(getInstance());
        intakeCoaxialSubsystem = new IntakeCoaxialSubsystem(getInstance());
        intakeRotationSubsystem = new IntakeRotationSubsystem(getInstance());
        outtakeArmSubsystem = new OuttakeArmSubsystem(getInstance());
        outtakeClawSubsystem = new OuttakeClawSubsystem(getInstance());
        outtakeRotationSubsystem = new OuttakeRotationSubsystem(getInstance());
        depositSubsystem = new DepositSubsystem(getInstance());
        extendoSubsystem = new ExtendoSubsystem(getInstance());
        driveSubsystem = new DriveSubsystem(new PinpointDrive(hardwareMap, new Pose2d(0,0,0)), false);

        if (Globals.AUTO) {
            sampleDetection = new AngleDetection();
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            sampleCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
            sampleCamera.setPipeline(sampleDetection = new AngleDetection());
            sampleCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    sampleCamera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });

            FtcDashboard.getInstance().startCameraStream(sampleCamera, 60);
        }
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        batteryVoltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();

    }

    public double getSampleAngle() {
        if (!Double.isNaN(sampleDetection.getAngleOfGreenSample())) {
            return sampleDetection.getAngleOfGreenSample();
        } else {
            return -1;
        }
    }

    public Point getSamplePosition() {
        if (sampleDetection.getGreenSampleCoordinates() != null) {
            return sampleDetection.getGreenSampleCoordinates();
        } else {
            return new Point(0,0);
        }
    }

    public void stopCameraStream() {
        sampleCamera.closeCameraDeviceAsync(() -> System.out.println("Stopped camera."));
    }

    public double getVoltage() {
        return voltage;
    }
}