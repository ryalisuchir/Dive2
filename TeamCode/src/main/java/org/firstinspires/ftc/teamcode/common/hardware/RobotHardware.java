package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.camera.AngleDetection;
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
    public VoltageSensor batteryVoltageSensor;

    private double voltage = 0.0;
    private ElapsedTime voltageTimer;
//    public OpenCvWebcam sampleCamera;
//    AngleDetection sampleDetection;

    private static RobotHardware instance = null;
    public boolean enabled;
    private HardwareMap hardwareMap;

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
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

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
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    }
//    public void stopCameraStream() {
//        sampleCamera.closeCameraDeviceAsync(() -> System.out.println("Stopped camera."));
//    }

    public double getVoltage() {
        return voltage;
    }
}