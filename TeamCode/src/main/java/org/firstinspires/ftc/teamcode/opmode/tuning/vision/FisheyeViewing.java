package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.CameraScanningPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.commands.recipes.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.vision.FishEyeYellowBlueDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
public class FisheyeViewing extends OpMode {
    public static boolean scanning = false;
    public static double offset = 0;
    private final double area = 0;
    private final double lastArea = 0;
    RobotHardware robot;
    OpenCvWebcam webcam;
    FishEyeYellowBlueDetection sampleDetection;
    private double estimate = 0; // Current estimate
    private double lastEstimate = 0; // Preserved last valid estimate

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap, Globals.DEFAULT_START_POSE, true);
        CommandScheduler.getInstance().schedule(new CameraScanningPositionCommand(robot, Globals.INTAKE_ROTATION_REST, Globals.EXTENDO_MAX_EXTENSION / 2));

        sampleDetection = new FishEyeYellowBlueDetection();
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
            } else {
                estimate = lastEstimate;
            }
        }

        telemetry.addData("Claw Angle: ", lastEstimate);
        telemetry.addData("Rotation Position: ", lastEstimate);

        if (scanning) {
            CommandScheduler.getInstance().schedule(new IntakeCommand(robot, lastEstimate + offset, Globals.EXTENDO_MAX_RETRACTION));
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
