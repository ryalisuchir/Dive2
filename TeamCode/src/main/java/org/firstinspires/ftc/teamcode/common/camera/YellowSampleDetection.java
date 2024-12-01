package org.firstinspires.ftc.teamcode.common.camera;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class YellowSampleDetection extends OpenCvPipeline {
    /*
     * Our working image buffers
     */
    Mat hsvMat = new Mat();
    Mat yellowThresholdMat = new Mat();
    Mat morphedYellowThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values for yellow detection in HSV space
     */
    static final Scalar YELLOW_LOWER_BOUND = new Scalar(20, 100, 100); // Adjust the lower bound for yellow
    static final Scalar YELLOW_UPPER_BOUND = new Scalar(30, 255, 255); // Adjust the upper bound for yellow

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    /*
     * Colors
     */
    static final Scalar YELLOW = new Scalar(0, 255, 255);
    static final int CONTOUR_LINE_THICKNESS = 2;

    static class AnalyzedStone {
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
    enum Stage {
        FINAL,
        HSV,
        MASKS,
        MASKS_NR,
        CONTOURS;
    }

    Stage[] stages = Stage.values();
    int stageNum = 0;

    public YellowSampleDetection() {
        // Initialize camera parameters with placeholders
        double fx = 800; // Replace with your camera's focal length in pixels
        double fy = 800;
        double cx = 320; // Replace with your camera's principal point x-coordinate
        double cy = 240; // Replace with your camera's principal point y-coordinate

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        // Distortion coefficients
        distCoeffs = new MatOfDouble(0, 0, 0, 0, 0);
    }

    @Override
    public void onViewportTapped() {
        int nextStageNum = stageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input) {
        internalStoneList.clear();
        findContours(input);
        clientStoneList = new ArrayList<>(internalStoneList);

        switch (stages[stageNum]) {
            case HSV:
                return hsvMat;
            case FINAL:
                return input;
            case MASKS:
                return yellowThresholdMat;
            case MASKS_NR:
                return morphedYellowThreshold;
            case CONTOURS:
                return contoursOnPlainImageMat;
        }

        return input;
    }

    public ArrayList<AnalyzedStone> getDetectedStones() {
        return clientStoneList;
    }

    void findContours(Mat input) {
        // Convert the input image to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Apply threshold to detect yellow
        Core.inRange(hsvMat, YELLOW_LOWER_BOUND, YELLOW_UPPER_BOUND, yellowThresholdMat);

        // Apply morphology to the yellow mask
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours in the yellow mask
        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Draw contours on plain image mat
        input.copyTo(contoursOnPlainImageMat);
        Imgproc.drawContours(contoursOnPlainImageMat, yellowContoursList, -1, YELLOW, CONTOUR_LINE_THICKNESS);

        // Analyze the largest yellow contour if found
        if (!yellowContoursList.isEmpty()) {
            double largestArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : yellowContoursList) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                analyzeContour(largestContour, input, "Yellow");
            }
        }
    }

    void morphMask(Mat input, Mat output) {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color) {
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

        Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

        AnalyzedStone analyzedStone = new AnalyzedStone();
        analyzedStone.angle = angle;
        analyzedStone.color = color;
        analyzedStone.rvec = rvec;
        analyzedStone.tvec = tvec;
        internalStoneList.add(analyzedStone);
    }

    void drawRotatedRect(RotatedRect rect, Mat drawOn, String color) {
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; i++) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], getColorScalar(color), CONTOUR_LINE_THICKNESS);
        }
    }

    void drawTagText(RotatedRect rect, String text, Mat drawOn, String color) {
        Imgproc.putText(drawOn, text, rect.center, Imgproc.FONT_HERSHEY_PLAIN, 1.5, getColorScalar(color), 2);
    }

    Point[] orderPoints(Point[] points) {
        Point[] ordered = new Point[4];

        double sumMax = Double.NEGATIVE_INFINITY;
        double sumMin = Double.POSITIVE_INFINITY;
        for (Point p : points) {
            double sum = p.x + p.y;
            if (sum < sumMin) {
                sumMin = sum;
                ordered[0] = p;
            }
            if (sum > sumMax) {
                sumMax = sum;
                ordered[2] = p;
            }
        }

        double diffMax = Double.NEGATIVE_INFINITY;
        double diffMin = Double.POSITIVE_INFINITY;
        for (Point p : points) {
            double diff = p.x - p.y;
            if (diff < diffMin) {
                diffMin = diff;
                ordered[3] = p;
            }
            if (diff > diffMax) {
                diffMax = diff;
                ordered[1] = p;
            }
        }

        return ordered;
    }

    static Scalar getColorScalar(String color) {
        return YELLOW;
    }
}
