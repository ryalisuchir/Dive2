package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AngleDetection extends OpMode {
    private VisionPortal visionPortal;
    OpenCvWebcam webcam;
    double lastDetectedBlue = 0;
    double lastDetectedArea = 0;
    Point lastDetectedPoint = new Point(0,0);
    Servo claw;

    org.firstinspires.ftc.teamcode.common.camera.AngleDetection sampleDetection;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setPipeline(sampleDetection = new org.firstinspires.ftc.teamcode.common.camera.AngleDetection());
        claw = hardwareMap.get(Servo.class, "servo");
        claw.setPosition(0);

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
            telemetry.addLine("No Blue Sample Detected");
        } else {
            // Update lastDetectedBlue with modulo 180
            lastDetectedBlue = greenAngle % 170;
            lastDetectedPoint = greenPoint;
            lastDetectedArea = sampleDetection.getGreenSampleArea();
            telemetry.addData("Last Detected At: ", greenAngle);
            telemetry.addData("Point Detected At: ", greenPoint);
        }

        clawAngle = lastDetectedBlue / 180; // Claw angle between 0 and 1
        telemetry.addData("Claw Angle: ", clawAngle);
        claw.setPosition(clawAngle);
        telemetry.addData("Area: ", lastDetectedArea);
        telemetry.update();
    }


    @Override
    public void loop() {
        //Do nothing
    }
}
