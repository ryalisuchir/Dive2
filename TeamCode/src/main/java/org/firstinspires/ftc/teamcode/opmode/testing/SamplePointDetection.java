package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class SamplePointDetection extends OpMode {
    private VisionPortal visionPortal;
    OpenCvWebcam webcam;
    double lastDetectedBlue = 0;
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
        Point greenPoint = sampleDetection.getGreenSampleCoordinates();

        if (Double.isNaN(greenAngle)) {
            telemetry.addData("Point Detected At: ", lastDetectedPoint);
            telemetry.addLine("No Blue Sample Detected");
        } else {
            lastDetectedPoint = greenPoint;
            telemetry.addData("Point Detected At: ", greenPoint);
        }

        telemetry.update();
    }


    @Override
    public void loop() {
        //Do nothing
    }
}
