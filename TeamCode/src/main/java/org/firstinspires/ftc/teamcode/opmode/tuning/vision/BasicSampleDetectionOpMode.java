package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.utility.KalmanFilter;
import org.firstinspires.ftc.teamcode.common.vision.YellowAngleDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class BasicSampleDetectionOpMode extends OpMode {
    RobotHardware robot;
    OpenCvWebcam webcam;
    double lastDetectedBlue = 0;
    Point lastDetectedPoint = new Point(0,0);
    Servo claw;

    YellowAngleDetection sampleDetection;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);
        CommandScheduler.getInstance().schedule(new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_RETRACTION));

        sampleDetection = new YellowAngleDetection();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setPipeline(sampleDetection);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
    }

    public void init_loop() {
        double greenAngle = sampleDetection.getAngleOfGreenSample();
        double clawAngle;
        Point greenPoint = sampleDetection.getGreenSampleCoordinates();

        if (Double.isNaN(greenAngle)) {
            telemetry.addData("Last Detected At: ", lastDetectedBlue);
            telemetry.addData("Point Detected At: ", lastDetectedPoint);
        } else {
            // Update lastDetectedBlue with modulo 180
            lastDetectedBlue = greenAngle % 170;
            lastDetectedPoint = greenPoint;
            telemetry.addData("Last Detected At: ", greenAngle);
            telemetry.addData("Point Detected At: ", greenPoint);
        }

        clawAngle = lastDetectedBlue / 180; // Claw angle between 0 and 1
        double estimate;
        double Q = 10; // High values put more emphasis on the sensor.
        double R = 2; // High Values put more emphasis on regression.
        int N = 3; // The number of estimates in the past we perform regression on.
        KalmanFilter filter = new KalmanFilter(Q,R,N);
        estimate = filter.estimate(clawAngle); // smoothed sensor

        telemetry.addData("Claw Angle: ", estimate);
        robot.intakeRotation.setPosition(clawAngle);


        telemetry.update();
    }


    @Override
    public void loop() {
        //Do nothing
    }
}
