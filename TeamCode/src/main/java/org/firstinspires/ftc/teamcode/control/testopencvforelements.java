package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "OpenCV Integrated", group = "OpenCV")
public class testopencvforelements  extends LinearOpMode {

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;
    private OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        // khoi tao webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // gan pipline xu li khung hinh
        webcam.setPipeline(new IntegratedPipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }

        // dung camera khi ket thuc
        webcam.stopStreaming();
    }

    static class IntegratedPipeline extends OpenCvPipeline {

        private Mat hsvFrame = new Mat();
        private Mat gray = new Mat();
        private Mat mask = new Mat();
        private Mat binary = new Mat();
        private Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

        @Override
        public Mat processFrame(Mat input) {
            // chuyen doi sang HSV de xu li mau
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);
            processColor(input, hsvFrame, new Scalar(20, 100, 100), new Scalar(30, 255, 255), new Scalar(0, 255, 255), "Yellow");
            processColor(input, hsvFrame, new Scalar(100, 150, 100), new Scalar(130, 255, 255), new Scalar(255, 0, 0), "Blue");
            processRedColor(input, hsvFrame);

            // phat hien hinh chu nhat xoay
            detectRotatedRectangles(input);

            return input;
        }

        private void processColor(Mat input, Mat hsvFrame, Scalar lowerBound, Scalar upperBound, Scalar color, String label) {
            Core.inRange(hsvFrame, lowerBound, upperBound, mask);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
            handleContours(input, mask, color, label);
        }

        private void processRedColor(Mat input, Mat hsvFrame) {
            Mat lowerRedMask = new Mat();
            Mat upperRedMask = new Mat();
            Core.inRange(hsvFrame, new Scalar(0, 120, 100), new Scalar(10, 255, 255), lowerRedMask);
            Core.inRange(hsvFrame, new Scalar(170, 120, 100), new Scalar(180, 255, 255), upperRedMask);
            Core.addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, mask);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
            handleContours(input, mask, new Scalar(0, 0, 255), "Red");
        }

        private void handleContours(Mat input, Mat mask, Scalar color, String label) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < 1000) continue;

                Rect boundingBox = Imgproc.boundingRect(contour);
                double cX = boundingBox.x + boundingBox.width / 2.0;
                double cY = boundingBox.y + boundingBox.height / 2.0;

                Imgproc.rectangle(input, boundingBox, color, 2);
                Imgproc.putText(input, label, new Point(cX - 20, cY - 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }
        }

        private void detectRotatedRectangles(Mat input) {
            // chuyen doi anh sang muc xam nhi phan
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.threshold(gray, binary, 50, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

            // tim duong vien
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < 3700 || area > 100000) continue;

                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);

                Point center = rect.center;
                double width = rect.size.width;
                double height = rect.size.height;
                double angle = rect.angle;

                if (width < height) {
                    angle = 90 - angle;
                } else {
                    angle = -angle;
                }

                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 0, 255), 2);
                }
                Imgproc.putText(input, "Angle: " + (int) angle + "°", new Point(center.x - 50, center.y),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0, 255, 0), 2);
            }
        }
    }
}
