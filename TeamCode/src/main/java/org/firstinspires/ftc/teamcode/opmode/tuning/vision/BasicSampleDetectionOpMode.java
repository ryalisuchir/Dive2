package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.utility.KalmanFilter;
import org.firstinspires.ftc.teamcode.common.vision.SigmaAngleDetection;
import org.firstinspires.ftc.teamcode.common.vision.YellowAngleDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
public class BasicSampleDetectionOpMode extends OpMode {
    RobotHardware robot;
    OpenCvWebcam webcam;
    double lastDetectedBlue = 0;
    Point lastDetectedPoint = new Point(0,0);
    public static boolean scanning = false;
    public static double offset = 0;
    public static boolean toScan = true;

    SigmaAngleDetection sampleDetection;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);
        CommandScheduler.getInstance().schedule(new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_RETRACTION));

        sampleDetection = new SigmaAngleDetection();
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
            KalmanFilter filter = new KalmanFilter(Q, R, N);
            estimate = filter.estimate(clawAngle); // smoothed sensor

            telemetry.addData("Claw Angle: ", estimate);


        if (scanning) {
            CommandScheduler.getInstance().schedule(new ScanningCommand(robot,  mapSampleToServo(estimate), Globals.EXTENDO_MAX_RETRACTION));
        } else {
            CommandScheduler.getInstance().schedule(new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_RETRACTION));
        }

        CommandScheduler.getInstance().run();

        telemetry.update();
    }
    private static final double[] SAMPLE_POSITIONS = {0.1834, 0.04, 0.205, 0.4271, 0.5531, 0.6958, 0.72, 0.8037};
    private static final double[] SERVO_POSITIONS = {0.65, 0.53, 0.65, 0.83, 0.9, 1.0, 0.45, 0.52};

    public static double mapSampleToServo(double samplePosition) {
        // Check bounds
        if (samplePosition <= SAMPLE_POSITIONS[0]) {
            return SERVO_POSITIONS[0];
        }
        if (samplePosition >= SAMPLE_POSITIONS[SAMPLE_POSITIONS.length - 1]) {
            return SERVO_POSITIONS[SERVO_POSITIONS.length - 1];
        }

        for (int i = 0; i < SAMPLE_POSITIONS.length - 1; i++) {
            if (samplePosition >= SAMPLE_POSITIONS[i] && samplePosition <= SAMPLE_POSITIONS[i + 1]) {
                double t = (samplePosition - SAMPLE_POSITIONS[i]) / (SAMPLE_POSITIONS[i + 1] - SAMPLE_POSITIONS[i]);
                return SERVO_POSITIONS[i] + t * (SERVO_POSITIONS[i + 1] - SERVO_POSITIONS[i]);
            }
        }

        return -1;
    }


    @Override
    public void loop() {
        //Do nothing
    }
}
