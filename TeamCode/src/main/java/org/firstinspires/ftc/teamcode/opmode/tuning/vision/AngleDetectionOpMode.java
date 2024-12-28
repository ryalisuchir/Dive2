package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.AllSystemInitializeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.utility.KalmanFilter;
import org.firstinspires.ftc.teamcode.common.vision.YellowAngleDetection;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AngleDetectionOpMode extends OpMode {
    RobotHardware robot;
    OpenCvWebcam webcam;

    Servo claw;

    YellowAngleDetection sampleDetection;

    // State variables
    boolean isScanning = false;
    boolean isButtonPressed = false;

    double xTravel = 0;
    double yTravel = 0;
    double angle = 0;

    KalmanFilter kalmanFilter;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);
        CommandScheduler.getInstance().schedule(new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION));

        sampleDetection = new YellowAngleDetection();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setPipeline(sampleDetection);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error: ", errorCode);
            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 60);

        // Initialize Kalman filter with sample parameters
        double Q = 10; // Process noise covariance
        double R = 2;  // Measurement noise covariance
        int N = 3;     // Number of historical estimates
        kalmanFilter = new KalmanFilter(Q, R, N);
    }

    @Override
    public void loop() {
        // Check for button press to toggle scanning
        if (gamepad1.x && !isButtonPressed) {
            isScanning = !isScanning; // Toggle scanning state
            isButtonPressed = true;  // Prevent multiple toggles from a single press
        } else if (!gamepad1.x) {
            isButtonPressed = false; // Reset button press state when released
        }

        if (isScanning) {
            double greenAngle = sampleDetection.getAngleOfGreenSample();
            Point greenPoint = sampleDetection.getGreenSampleCoordinates();

            if (!Double.isNaN(greenAngle)) {
                // Smooth the detected angle using Kalman filter
                angle = kalmanFilter.estimate(greenAngle % 180);

                // Calculate travel
                xTravel = (greenPoint.x);
                yTravel = -(greenPoint.y);

                // Stop scanning
                isScanning = false;
            }
        }

        // Display results only after scanning stops
        if (!isScanning) {
            telemetry.addData("X Travel", xTravel);
            telemetry.addData("Y Travel", yTravel);
            telemetry.addData("Angle", angle);
        }

        telemetry.update();
    }
}
