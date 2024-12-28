package org.firstinspires.ftc.teamcode.opmode.tuning.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.utility.KalmanFilter;
import org.firstinspires.ftc.teamcode.common.vision.SigmaAngleDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class SecondaryAngleDetectionOpMode extends OpMode {
    private VisionPortal visionPortal;
    OpenCvWebcam webcam;
    double lastDetectedBlue = 0;
    double lastDetectedArea = 0;
    Point lastDetectedPoint = new Point(0,0);

    SigmaAngleDetection sampleDetection;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.setPipeline(sampleDetection = new org.firstinspires.ftc.teamcode.common.vision.SigmaAngleDetection());

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
//        double greenSampleArea = sampleDetection.getGreenSampleArea();
        Point greenPoint = sampleDetection.getGreenSampleCoordinates();

        if (Double.isNaN(greenAngle)) {
            telemetry.addLine("1");
            telemetry.addData("Last Detected At: ", lastDetectedBlue);
            telemetry.addData("Point Detected At: ", lastDetectedPoint);
        } else {
            // Update lastDetectedBlue with modulo 180
            lastDetectedBlue = greenAngle % 170;
            lastDetectedPoint = greenPoint;
            telemetry.addLine("2");
            telemetry.addData("Last Detected At: ", greenAngle);
            telemetry.addData("Point Detected At: ", greenPoint);
        }

//        if (Double.isNaN(greenSampleArea)) {
//            telemetry.addData("Last Detected Area: ", lastDetectedArea);
//            telemetry.addLine("No Blue Sample Detected");
//        } else {
//            lastDetectedArea = greenSampleArea;
//            telemetry.addData("Last Deteced Area: ", greenSampleArea);
//        }

        clawAngle = lastDetectedBlue / 180; // Claw angle between 0 and 1
        double estimate;
        double Q = 10; // High values put more emphasis on the sensor.
        double R = 2; // High Values put more emphasis on regression.
        int N = 3; // The number of estimates in the past we perform regression on.
        KalmanFilter filter = new KalmanFilter(Q,R,N);
        double currentValue = clawAngle;  // imaginary, noisy sensor
        estimate = filter.estimate(currentValue); // smoothed sensor

        telemetry.addData("Claw Angle: ", estimate);

        telemetry.update();
    }


    @Override
    public void loop() {
        //Do nothing
    }
}
