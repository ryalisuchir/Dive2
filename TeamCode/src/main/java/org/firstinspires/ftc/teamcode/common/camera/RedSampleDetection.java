package org.firstinspires.ftc.teamcode.common.camera;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class RedSampleDetection extends OpenCvPipeline
{
    /*
     * Our working image buffers
     */
    Mat ycrcbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();

    Mat redThresholdMat = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    static final int RED_MASK_THRESHOLD = 150;

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    /*
     * Colors
     */
    static final Scalar RED = new Scalar(0, 0, 255);

    static final int CONTOUR_LINE_THICKNESS = 2;

    static class AnalyzedStone
    {
        double angle;
        String color;
        Mat rvec;
        Mat tvec;
    }

    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    /*
     * Camera Calibration Parameters
     */
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();

    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage
    {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS;
    }

    Stage[] stages = Stage.values();
    int stageNum = 0;

    public RedSampleDetection()
    {
        // Initialize camera parameters
        double fx = 800; // Replace with your camera's focal length in pixels
        double fy = 800;
        double cx = 320; // Replace with your camera's principal point x-coordinate
        double cy = 240; // Replace with your camera's principal point y-coordinate

        cameraMatrix.put(0, 0, fx, 0, cx, 0, fy, cy, 0, 0, 1);
        distCoeffs = new MatOfDouble(0, 0, 0, 0, 0);
    }

    @Override
    public void onViewportTapped()
    {
        int nextStageNum = stageNum + 1;
        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }
        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        internalStoneList.clear();
        findContours(input);
        clientStoneList = new ArrayList<>(internalStoneList);

        switch (stages[stageNum])
        {
            case YCrCb:
                return ycrcbMat;

            case FINAL:
                return input;

            case MASKS:
                Mat masks = new Mat();
                Core.addWeighted(masks, 1.0, redThresholdMat, 1.0, 0.0, masks);
                return masks;

            case MASKS_NR:
                Mat masksNR = new Mat();
                Core.addWeighted(masksNR, 1.0, morphedRedThreshold, 1.0, 0.0, masksNR);
                return masksNR;

            case CONTOURS:
                return contoursOnPlainImageMat;
        }

        return input;
    }

    public ArrayList<AnalyzedStone> getDetectedStones()
    {
        return clientStoneList;
    }

    void findContours(Mat input)
    {
        // Convert the input image to YCrCb color space
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cr channel for red detection
        Core.extractChannel(ycrcbMat, crMat, 1); // Cr channel index is 1

        // Threshold the Cr channel to form a mask for red
        Imgproc.threshold(crMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);

        // Apply morphology to the red mask
        morphMask(redThresholdMat, morphedRedThreshold);

        // Find contours in the red mask
        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        if (!redContoursList.isEmpty()) {
            double largestArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : redContoursList) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                analyzeContour(largestContour, input, "Red");
            }
        }
    }

    void morphMask(Mat input, Mat output)
    {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color)
    {
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        drawRotatedRect(rotatedRectFitToContour, input, color);

        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
            rotRectAngle += 90;
        }

        double angle = -(rotRectAngle - 180);
        drawTagText(rotatedRectFitToContour, Integer.toString((int) Math.round(angle)) + " deg", input, color);

        double objectWidth = 10.0;  // Replace with your object's width in real-world units
        double objectHeight = 5.0;  // Replace with your object's height in real-world units

        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, -objectHeight / 2, 0),
                new Point3(objectWidth / 2, objectHeight / 2, 0),
                new Point3(-objectWidth / 2, objectHeight / 2, 0)
        );

        Point[] rectPoints = new Point[4];
        rotatedRectFitToContour.points(rectPoints);
        Point[] orderedRectPoints = orderPoints(rectPoints);
        MatOfPoint2f imagePoints = new MatOfPoint2f(orderedRectPoints);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        boolean success = Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

        if (success) {
            drawLargestAreaAxis(input, rvec, tvec, cameraMatrix, distCoeffs);

            AnalyzedStone analyzedStone = new AnalyzedStone();
            analyzedStone.angle = rotRectAngle;
            analyzedStone.color = color;
            analyzedStone.rvec = rvec;
            analyzedStone.tvec = tvec;
            internalStoneList.add(analyzedStone);
        }
    }

    void drawLargestAreaAxis(Mat img, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs)
    {
        double axisLength = 5.0;
        MatOfPoint3f axisPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),
                new Point3(axisLength, 0, 0), // X axis
                new Point3(0, axisLength, 0), // Y axis
                new Point3(0, 0, -axisLength) // Z axis
        );

        MatOfPoint2f imagePoints = new MatOfPoint2f();
        Calib3d.projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

        Point[] imgPts = imagePoints.toArray();
        int largestAreaIndex = getLargestArea(imgPts);

        if (largestAreaIndex == 0) {
            Imgproc.line(img, imgPts[0], imgPts[1], new Scalar(0, 0, 255), 2); // X axis in red
        } else if (largestAreaIndex == 1) {
            Imgproc.line(img, imgPts[0], imgPts[3], new Scalar(255, 0, 0), 2); // Z axis in blue
        } else if (largestAreaIndex == 2) {
            Imgproc.line(img, imgPts[0], imgPts[2], new Scalar(0, 255, 0), 2); // Y axis in green
        }
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color)
    {
        Scalar colorScalar = getColorScalar(color);
        Imgproc.putText(mat, text, new Point(rect.center.x - 50, rect.center.y + 25), Imgproc.FONT_HERSHEY_PLAIN, 1, colorScalar, 1);
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color)
    {
        Point[] points = new Point[4];
        rect.points(points);
        Scalar colorScalar = getColorScalar(color);
        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    static Scalar getColorScalar(String color)
    {
        return RED;
    }

    static int getLargestArea(Point[] points) {
        // Example function to determine the index of the largest area
        return 0; // This is a placeholder, replace with actual logic
    }

    static Point[] orderPoints(Point[] points) {
        // Order the points in clockwise order (example logic)
        return points; // This is a placeholder, replace with actual logic
    }
}
