package org.firstinspires.ftc.teamcode.common.vision;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
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
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class YellowRedDetection extends OpenCvPipeline {

    static final Scalar YELLOW_LOWER_BOUND = new Scalar(20, 100, 100); // Adjust the lower bound for yellow
    static final Scalar YELLOW_UPPER_BOUND = new Scalar(30, 255, 255); // Adjust the upper bound for yellow

    // Updated Red color bounds
    static final Scalar RED_LOWER_BOUND = new Scalar(0, 100, 100); // Adjust the lower bound for red
    static final Scalar RED_UPPER_BOUND = new Scalar(10, 255, 255); // Adjust the upper bound for red

    static final Scalar RED = new Scalar(0, 0, 255); // Use red for visualization
    private static final String defaultSavePath = "/sdcard/EasyOpenCV";
    public double AREA_THRESHOLD = 4000;
    public MatOfPoint3f axisPoints;
    Mat ycrcbMat = new Mat();
    Mat hsvMat = new Mat();
    Mat yellowThresholdMat = new Mat();
    Mat morphedYellowThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();
    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
    MatOfDouble distCoeffs = new MatOfDouble();
    Stage[] stages = Stage.values();
    int stageNum = 0;

    public YellowRedDetection() {
        double fx = 800;
        double fy = 800;
        double cx = 320;
        double cy = 240;

        cameraMatrix.put(0, 0,
                fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        distCoeffs = new MatOfDouble(0, 0, 0, 0, 0);
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color) {
        Point[] points = new Point[4];
        rect.points(points);

        Scalar colorScalar = getColorScalar(color);

        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    static Scalar getColorScalar(String color) {
        if (color.equals("Green")) {
            return new Scalar(0, 255, 0); // Green color
        }
        return RED;
    }

    static Point[] orderPoints(Point[] pts) {
        Point[] orderedPts = new Point[4];

        double[] sum = new double[4];
        double[] diff = new double[4];

        for (int i = 0; i < 4; i++) {
            sum[i] = pts[i].x + pts[i].y;
            diff[i] = pts[i].y - pts[i].x;
        }

        int tlIndex = indexOfMin(sum);
        orderedPts[0] = pts[tlIndex];

        int brIndex = indexOfMax(sum);
        orderedPts[2] = pts[brIndex];

        int trIndex = indexOfMin(diff);
        orderedPts[1] = pts[trIndex];

        int blIndex = indexOfMax(diff);
        orderedPts[3] = pts[blIndex];

        return orderedPts;
    }

    static int indexOfMin(double[] array) {
        int index = 0;
        double min = array[0];

        for (int i = 1; i < array.length; i++) {
            if (array[i] < min) {
                min = array[i];
                index = i;
            }
        }
        return index;
    }

    static int indexOfMax(double[] array) {
        int index = 0;
        double max = array[0];

        for (int i = 1; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                index = i;
            }
        }
        return index;
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat,
                text,
                new Point(
                        rect.center.x - 50,
                        rect.center.y + 25),
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                colorScalar,
                1);
    }

    @Override
    public Mat processFrame(Mat input) {
        internalStoneList.clear();

        // Ensure input is not null and valid before processing
        if (input.empty()) {
            return input;
        }

        // Reuse existing Mats to avoid memory leaks
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Process yellow mask
        Core.inRange(hsvMat, YELLOW_LOWER_BOUND, YELLOW_UPPER_BOUND, yellowThresholdMat);
        morphMask(yellowThresholdMat, morphedYellowThreshold); // Reusing `morphedYellowThreshold` for yellow mask

        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, contoursOnPlainImageMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        processContours(yellowContoursList, input, "Yellow");

        // Process red mask (replacing blue mask with red)
        Core.inRange(hsvMat, RED_LOWER_BOUND, RED_UPPER_BOUND, yellowThresholdMat); // Reusing the same Mat
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, redContoursList, contoursOnPlainImageMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        processContours(redContoursList, input, "Red");

        clientStoneList = new ArrayList<>(internalStoneList);

        saveMatToDisk(hsvMat, "processed_frame.jpg");

        switch (stages[stageNum]) {
            case YCrCb:
                return ycrcbMat;
            case MASKS:
                return yellowThresholdMat; // Reusing the Mat object
            case MASKS_NR:
                return morphedYellowThreshold;
            case CONTOURS:
                return contoursOnPlainImageMat;
            default:
                return input;
        }
    }

    void morphMask(Mat input, Mat output) {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void processContours(ArrayList<MatOfPoint> contoursList, Mat input, String color) {
        if (!contoursList.isEmpty()) {
            double smallestDistance = Double.MAX_VALUE;
            MatOfPoint closestContour = null;

            Point fovCenter = Globals.cameraCenter;

            for (MatOfPoint contour : contoursList) {
                double contourArea = Imgproc.contourArea(contour);
                if (contourArea > AREA_THRESHOLD) {
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    Point contourCenter = new Point(
                            boundingRect.x + boundingRect.width / 2.0,
                            boundingRect.y + boundingRect.height / 2.0
                    );

                    // Calculate the distance to the center of the FOV
                    double distance = Math.sqrt(
                            Math.pow(contourCenter.x - fovCenter.x, 2) +
                                    Math.pow(contourCenter.y - fovCenter.y, 2)
                    );

                    if (distance < smallestDistance) {
                        smallestDistance = distance;
                        closestContour = contour;
                    }
                }
            }

            for (MatOfPoint contour : contoursList) {
                if (contour.equals(closestContour)) {
                    analyzeContour(contour, input, "Green"); // Mark the closest as green
                } else {
                    analyzeContour(contour, input, color);
                }
            }
        }
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
        drawTagText(rotatedRectFitToContour, (int) Math.round(angle) + " deg", input, color);

        double objectWidth = 10.0;
        double objectHeight = 5.0;

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

        boolean success = Calib3d.solvePnP(
                objectPoints,
                imagePoints,
                cameraMatrix,
                distCoeffs,
                rvec,
                tvec
        );

        if (success) {
            drawLargestAreaAxis(input, rvec, tvec, cameraMatrix, distCoeffs);

            AnalyzedStone analyzedStone = new AnalyzedStone();
            analyzedStone.angle = rotRectAngle;
            analyzedStone.color = color;
            analyzedStone.rvec = rvec;
            analyzedStone.tvec = tvec;
            analyzedStone.x = -1; // Initializing x and y to -1 or any default value
            analyzedStone.y = -1;
            internalStoneList.add(analyzedStone);
        }
    }

    void drawLargestAreaAxis(Mat img, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs) {
        double axisLength = 5.0;

        axisPoints = new MatOfPoint3f(
                new Point3(0, 0, 0),                   // Origin
                new Point3(axisLength, 0, 0),           // X axis
                new Point3(0, axisLength, 0),           // Y axis
                new Point3(0, 0, -axisLength)           // Z axis
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

    double calculateArea(Point p0, Point p1, Point p2) {
        return Math.abs((p0.x * (p1.y - p2.y) +
                p1.x * (p2.y - p0.y) +
                p2.x * (p0.y - p1.y)) / 2.0);
    }

    int getLargestArea(Point[] imgPts) {
        double areaXY = calculateArea(imgPts[0], imgPts[1], imgPts[2]); // Triangle formed by Origin, X, Y
        double areaXZ = calculateArea(imgPts[0], imgPts[1], imgPts[3]); // Triangle formed by Origin, X, Z
        double areaYZ = calculateArea(imgPts[0], imgPts[2], imgPts[3]); // Triangle formed by Origin, Y, Z

        if (areaXY >= areaXZ && areaXY >= areaYZ) {
            return 0; // XY axis has the largest area
        } else if (areaXZ >= areaXY && areaXZ >= areaYZ) {
            return 1; // XZ axis has the largest area
        } else {
            return 2; // YZ axis has the largest area
        }
    }

    public double getAngleOfGreenSample() {
        AnalyzedStone greenSample = null;
        double highestYCoordinate = Double.MAX_VALUE;

        List<AnalyzedStone> stonesCopy = new ArrayList<>(internalStoneList);
        for (AnalyzedStone stone : stonesCopy) {
            if (stone != null && "Green".equals(stone.color) && stone.tvec != null) {
                double[] yValueArray = stone.tvec.get(1, 0);
                if (yValueArray != null && yValueArray.length > 0) {
                    double yValue = yValueArray[0]; // Extracting the y-coordinate of translation vector
                    if (yValue < highestYCoordinate) {
                        highestYCoordinate = yValue;
                        greenSample = stone;
                    }
                }
            }
        }

        if (greenSample != null) {
            return greenSample.angle;
        } else {
            return Double.NaN;  // Indicate that no "green" sample was found
        }
    }

    public Point getGreenSampleCoordinates() {
        AnalyzedStone greenSample = null;

        List<AnalyzedStone> stonesCopy = new ArrayList<>(internalStoneList);
        // Loop through the internal stone list to find the green sample
        for (AnalyzedStone stone : stonesCopy) {
            if (stone != null && "Green".equals(stone.color) && stone.tvec != null) {
                greenSample = stone;
                break;
            }
        }

        // If we found a green sample, calculate its coordinates
        if (greenSample != null) {
            // Extract tvec values which represent the translation vector
            double[] tvecValues = new double[3];
            greenSample.tvec.get(0, 0, tvecValues);

            // Extract x and y from tvec directly
            double x = tvecValues[0]; // Original unit (cm)
            double y = tvecValues[1]; // Original unit (cm)

            // Convert to inches (assuming the original unit is centimeters)

            x = -(Math.round((x / 2.54) * 10.0) / 10.0) + 1;
            y = (Math.round((y / 2.54) * 10.0) / 10.0);

            // Store these coordinates in the sample and return as a point
            greenSample.x = x;
            greenSample.y = y;

            // Calculate center point if needed (currently just returning the translated point)
            return new Point(greenSample.x + Globals.visionOffset, -greenSample.y + Globals.visionOffset);
        } else {
            return null; // No green sample found
        }
    }

    public void saveMatToDisk(Mat mat, final String filename) {
        saveMatToDiskFullPath(mat, String.format("%s/%s.png", defaultSavePath, filename));
    }


    enum Stage {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS
    }

    static class AnalyzedStone {
        double angle;
        String color;
        Mat rvec;
        Mat tvec;
        double x;
        double y;
        MatOfPoint contour; // Add a field to store the contour of the stone
    }
}