package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.OpMode.AprilTagDetectionPipeline;
import java.util.List;

@Autonomous
public class OpenCVAprilTagNav extends OpMode {

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagPipeline;

    // Hằng số camera và cấu hình April Tag
    double fx = 578.272; // Focal length x
    double fy = 578.272; // Focal length y
    double cx = 402.145; // Principal point x
    double cy = 221.506; // Principal point y
    double tagsize = 0.166; // Kích thước thực tế của April Tag (m)

    // Biến lưu trữ April Tag mục tiêu
    int targetTagId = 1; // ID tag bạn muốn tìm
    AprilTagDetection targetTag = null;

    @Override
    public void init() {
        // Lấy ID của camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Tạo pipeline để xử lý April Tag
        aprilTagPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        // Kết nối camera với pipeline
        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Error opening camera");
            }
        });
    }

    @Override
    public void loop() {
        // Lấy danh sách các tag được phát hiện
        List<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();

        // Nếu không tìm thấy tag nào
        if (currentDetections.isEmpty()) {
            telemetry.addData("Status", "No April Tags detected");
            return;
        }

        // Tìm tag mục tiêu
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetTagId) {
                targetTag = detection;
                break;
            }
        }

        // Nếu không tìm thấy tag mục tiêu
        if (targetTag == null) {
            telemetry.addData("Status", "Target Tag not found");
            return;
        }

        // Hiển thị thông tin vị trí của tag
        double x = targetTag.pose.x;
        double y = targetTag.pose.y;
        double z = targetTag.pose.z;
        telemetry.addData("Target Tag ID", targetTagId);
        telemetry.addData("Position (x, y, z)", "%.2f, %.2f, %.2f", x, y, z);

        // **Điều hướng robot**
        if (z > 0.5) { // Nếu còn xa tag, tiến tới
            moveForward(0.5);
            telemetry.addData("Action", "Moving Forward");
        } else if (x > 0.1) { // Nếu tag lệch phải, xoay robot
            turnRight(0.3);
            telemetry.addData("Action", "Turning Right");
        } else if (x < -0.1) { // Nếu tag lệch trái, xoay robot
            turnLeft(0.3);
            telemetry.addData("Action", "Turning Left");
        } else {
            stopRobot(); // Đã đến tag, dừng robot
            telemetry.addData("Action", "Arrived at Target");
        }
    }

    private void moveForward(double power) {
        // Hàm điều khiển robot tiến tới
    }

    private void turnRight(double power) {
        // Hàm điều khiển robot xoay phải
    }

    private void turnLeft(double power) {
        // Hàm điều khiển robot xoay trái
    }

    private void stopRobot() {
        // Hàm dừng robot
    }
}
