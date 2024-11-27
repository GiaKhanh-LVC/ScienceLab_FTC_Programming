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

@TeleOp(name = "OpenCV")
public class opencvtest extends LinearOpMode {

    // kich thuoc camera
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    // thong so vat ly va camera
    private static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 89 * 0.03937; // kich thuoc vat the
    private static final double FOCAL_LENGTH = 700; // tieu cu

    private OpenCvCamera controlHubCam;

    @Override
    public void runOpMode() {
        initOpenCV();

        // cai dat telemetry va dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(controlHubCam, 30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }

        // giai phong tai nguyen
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new MultiColorBlobDetectionPipeline());
        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    // pipline xu li khung hinh va nhan dien mau
    static class MultiColorBlobDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // tien xu li mat na cho cac mau
            Mat yellowMask = preprocessFrame(input, new Scalar(20, 100, 100), new Scalar(30, 255, 255)); // vang
            Mat redMask = preprocessRedFrame(input); // do
            Mat blueMask = preprocessFrame(input, new Scalar(100, 150, 100), new Scalar(130, 255, 255)); // xanh nuoc bien

            // xu li tung mau va chu thich mau
            processColor(input, yellowMask, new Scalar(0, 255, 255), "Yellow");
            processColor(input, redMask, new Scalar(0, 0, 255), "Red");
            processColor(input, blueMask, new Scalar(255, 0, 0), "Blue");

            return input;
        }

        private Mat preprocessFrame(Mat frame, Scalar lowerBound, Scalar upperBound) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Mat mask = new Mat();
            Core.inRange(hsvFrame, lowerBound, upperBound, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel); // khu nhieu
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel); // lam muot vung mask

            return mask;
        }

        private Mat preprocessRedFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // mau do co the nam trong hai khoang khong gian HSV
            Mat lowerRedMask = new Mat();
            Mat upperRedMask = new Mat();
            Core.inRange(hsvFrame, new Scalar(0, 120, 100), new Scalar(10, 255, 255), lowerRedMask);
            Core.inRange(hsvFrame, new Scalar(170, 120, 100), new Scalar(180, 255, 255), upperRedMask);

            Mat redMask = new Mat();
            Core.addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, redMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel); // khu nhieu
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel); // lam muot vung mask

            return redMask;
        }

        private void processColor(Mat input, Mat mask, Scalar color, String label) {
            // tim duong vien
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                Moments moments = Imgproc.moments(largestContour);
                double cX = moments.get_m10() / moments.get_m00(); // toa do tam x
                double cY = moments.get_m01() / moments.get_m00(); // toa do tam y
                double detectedWidth = calculateWidth(largestContour); // chieu rong phan tu

                // chu thich
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), color, 2);
                Imgproc.circle(input, new Point(cX, cY), 5, color, -1);

                String widthLabel = label + " Width: " + (int) detectedWidth + " px";
                String distanceLabel = label + " Distance: " +
                        String.format(Locale.US, "%.2f", calculateDistance(detectedWidth)) + " in";

                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour); // dien tich duong vien
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width; // tra ve chieu rong hcn bao quanh
        }

        private double calculateDistance(double widthInPixels) {
            if (widthInPixels > 0) {
                return (OBJECT_WIDTH_IN_REAL_WORLD_UNITS * FOCAL_LENGTH) / widthInPixels; //  tinh khoang cach
            }
            return 0;
        }
    }
}
