package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.intake.ScanningCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.vision.YellowRedDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
@Disabled
public class RedYellowAngleDetection extends OpMode {
    public static boolean scanning = false;
    public static double offset = 0;

    RobotHardware robot;
    OpenCvWebcam webcam;
    YellowRedDetection sampleDetection;
    double x = 0;
    double y = 0;
    double lastX = 0;
    double lastY = 0;
    private double estimate = 0; // Current estimate
    private double lastEstimate = 0; // Preserved last valid estimate

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);
        CommandScheduler.getInstance().schedule(new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION / 2));

        sampleDetection = new YellowRedDetection();
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
                // Handle camera error
            }
        });
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
    }

    @Override
    public void init_loop() {
        if (!scanning) {
            double greenAngle = sampleDetection.getAngleOfGreenSample();

            if (!Double.isNaN(greenAngle)) {
                estimate = (greenAngle % 180) / 180;
                lastEstimate = estimate;
                try {
                    x = sampleDetection.getGreenSampleCoordinates().x;
                    y = sampleDetection.getGreenSampleCoordinates().y;
                } catch (RuntimeException e) {
                    telemetry.addLine("Error");
                }

                lastX = x;
                lastY = y;
            } else {
                estimate = lastEstimate;
                x = lastX;
                y = lastY;
            }
        }

        telemetry.addData("Claw Angle: ", lastEstimate);
        telemetry.addData("Rotation Position: ", lastEstimate);

        telemetry.addData("X: ", lastX);
        telemetry.addData("Y: ", lastY);

        if (scanning) {
            CommandScheduler.getInstance().schedule(new ScanningCommand(robot, lastEstimate + offset, Globals.EXTENDO_MAX_RETRACTION));
        } else {
            CommandScheduler.getInstance().schedule(new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_RETRACTION));
        }

        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    @Override
    public void loop() {
        // Do nothing
    }
}
