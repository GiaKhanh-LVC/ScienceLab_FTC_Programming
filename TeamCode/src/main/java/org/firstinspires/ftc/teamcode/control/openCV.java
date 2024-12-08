package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "OpenCV - Real Time Processing", group = "Experimental")
public class openCV extends LinearOpMode {

    // Camera configuration
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    // Object detection constants
    private static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 89 * 0.03937; // Convert mm to inches
    private static final double FOCAL_LENGTH = 700; // Camera focal length (arbitrary example)

    private OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        // Initialize camera
        initializeOpenCv();

        // Setup telemetry and dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(webcam, 30); // Stream video to FTC Dashboard

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            telemetry.update();
        }

        // Cleanup resources
        webcam.stopStreaming();
    }

    /**
     * Initialize OpenCV and setup the camera.
     */
    private void initializeOpenCv() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new MultiColorBlobDetectionPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    /**
     * Pipeline for real-time multi-color blob detection and distance estimation.
     */
    static class MultiColorBlobDetectionPipeline extends OpenCvPipeline {

        private Point yellowCenter = null;
        private double yellowDistance = 0.0;

        public Point getYellowCenter() {
            return yellowCenter;
        }

        public double getYellowDistance() {
            return yellowDistance;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Preprocess and detect yellow blobs
            Mat yellowMask = preprocessFrame(input, new Scalar(20, 100, 100), new Scalar(30, 255, 255));
            processColor(input, yellowMask, new Scalar(0, 255, 255), "Yellow");

            return input;
        }

        /**
         * Preprocess the input frame for a specific color range.
         */
        private Mat preprocessFrame(Mat frame, Scalar lowerBound, Scalar upperBound) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Mat mask = new Mat();
            Core.inRange(hsvFrame, lowerBound, upperBound, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel); // Noise reduction
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel); // Smoothing

            return mask;
        }

        /**
         * Process detected color mask and annotate the frame with results.
         */
        private void processColor(Mat input, Mat mask, Scalar color, String label) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint largestContour = findLargestContour(contours);
            if (largestContour != null) {
                Moments moments = Imgproc.moments(largestContour);
                yellowCenter = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());
                yellowDistance = calculateDistance(Imgproc.boundingRect(largestContour).width);

                // Annotate the frame
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), color, 2);
                Imgproc.circle(input, yellowCenter, 5, color, -1);

                String distanceLabel = label + " Distance: " +
                        String.format(Locale.US, "%.2f", yellowDistance) + " in";
                Imgproc.putText(input, distanceLabel, new Point(yellowCenter.x + 10, yellowCenter.y + 50),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }
        }

        /**
         * Find the largest contour from the given list of contours.
         */
        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        /**
         * Calculate the distance to an object based on its width in pixels.
         */
        private double calculateDistance(double widthInPixels) {
            if (widthInPixels > 0) {
                return (OBJECT_WIDTH_IN_REAL_WORLD_UNITS * FOCAL_LENGTH) / widthInPixels;
            }
            return 0;
        }
    }
}
