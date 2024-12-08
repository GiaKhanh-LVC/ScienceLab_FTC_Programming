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

@TeleOp(name = "OpenCV - Multi-Color Detection", group = "Experimental")
public class openCV extends LinearOpMode {

    // config camera
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    private OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        // init camera
        initializeOpenCv();

        // Setup telemetry and dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(webcam, 30); // Stream video toi FTC Dashboard

        waitForStart();

        // vong lap chinh
        while (opModeIsActive()) {
            telemetry.update();
        }

        // clean du lieu khi dung
        webcam.stopStreaming();
    }
    // init camera
    private void initializeOpenCv() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new MultiColorDetectionPipeline());
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

   // su dung pipline de xu li mau va tinh toan toa do, huong cua vat the
    static class MultiColorDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // xu li mau vang
            Mat yellowMask = preprocessFrame(input, new Scalar(20, 100, 100), new Scalar(30, 255, 255));
            processColorAndOrientation(input, yellowMask, new Scalar(0, 255, 255), "Yellow");

            // xu li mau do (mau do co hai dai mau HSV)
            Mat redMask1 = preprocessFrame(input, new Scalar(0, 120, 100), new Scalar(10, 255, 255));
            Mat redMask2 = preprocessFrame(input, new Scalar(170, 120, 100), new Scalar(180, 255, 255));
            Mat redMask = new Mat();
            Core.addWeighted(redMask1, 1.0, redMask2, 1.0, 0.0, redMask);
            processColorAndOrientation(input, redMask, new Scalar(0, 0, 255), "Red");

            // xu li mau xanh nuoc bien
            Mat blueMask = preprocessFrame(input, new Scalar(100, 150, 100), new Scalar(130, 255, 255));
            processColorAndOrientation(input, blueMask, new Scalar(255, 0, 0), "Blue");

            return input;
        }

        // tien xu li dau vao va cho mot dai mau cu the
        private Mat preprocessFrame(Mat frame, Scalar lowerBound, Scalar upperBound) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Mat mask = new Mat();
            Core.inRange(hsvFrame, lowerBound, upperBound, mask);

            // xu li va lam sach mask
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel); // khu nhieu
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel); // lam muot

            return mask;
        }

        /**
         * Process detected color mask and compute orientation.
         */
        private void processColorAndOrientation(Mat input, Mat mask, Scalar color, String label) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint largestContour = findLargestContour(contours);
            if (largestContour != null) {
                // Draw contours
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), color, 2);

                // Compute orientation
                getOrientation(largestContour, input);

                // Compute center and annotate distance
                Moments moments = Imgproc.moments(largestContour);
                Point center = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());
                Imgproc.circle(input, center, 5, color, -1);

                String centerLabel = label + " Center: (" +
                        String.format(Locale.US, "%.1f, %.1f", center.x, center.y) + ")";
                Imgproc.putText(input, centerLabel, new Point(center.x + 10, center.y + 50),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }
        }

       // tim vien vat the lon nhat va ve duong vien
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

       // tinh toan huong cua vat the su dung PCA và chu thich len anh
        private void getOrientation(MatOfPoint contour, Mat input) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Perform PCA
            Mat mean = new Mat();
            Mat eigenvectors = new Mat();
            Mat eigenvalues = new Mat();
            Core.PCACompute2(new MatOfPoint2f(contour2f), mean, eigenvectors, eigenvalues);

            Point center = new Point(mean.get(0, 0)[0], mean.get(0, 0)[1]);

            // Draw the center point
            Imgproc.circle(input, center, 3, new Scalar(255, 0, 255), 2);

            // Principal axes
            double scale = 0.02;
            Point p1 = new Point(
                    center.x + scale * eigenvectors.get(0, 0)[0] * eigenvalues.get(0, 0)[0],
                    center.y + scale * eigenvectors.get(0, 1)[0] * eigenvalues.get(0, 0)[0]);
            Point p2 = new Point(
                    center.x - scale * eigenvectors.get(1, 0)[0] * eigenvalues.get(1, 0)[0],
                    center.y - scale * eigenvectors.get(1, 1)[0] * eigenvalues.get(1, 0)[0]);

            drawAxis(input, center, p1, new Scalar(255, 255, 0), 1);
            drawAxis(input, center, p2, new Scalar(0, 0, 255), 5);

            double angle = Math.atan2(eigenvectors.get(0, 1)[0], eigenvectors.get(0, 0)[0]);

            // Add label with rotation angle
            String label = "Rotation Angle: " + String.format("%.2f", Math.toDegrees(angle)) + " degrees";
            Imgproc.putText(input, label, new Point(center.x + 10, center.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1, Imgproc.LINE_AA);
        }

        private void drawAxis(Mat img, Point p, Point q, Scalar color, int scale) {
            Imgproc.line(img, p, q, color, 2);

            double angle = Math.atan2(p.y - q.y, p.x - q.x);
            Point hook1 = new Point(q.x + scale * 10 * Math.cos(angle + Math.PI / 4),
                    q.y + scale * 10 * Math.sin(angle + Math.PI / 4));
            Point hook2 = new Point(q.x + scale * 10 * Math.cos(angle - Math.PI / 4),
                    q.y + scale * 10 * Math.sin(angle - Math.PI / 4));

            Imgproc.line(img, q, hook1, color, 2);
            Imgproc.line(img, q, hook2, color, 2);
        }
    }
}
