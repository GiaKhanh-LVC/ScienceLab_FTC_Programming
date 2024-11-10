package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoControlModule extends LinearOpMode {
    private Servo servo1 ;

    @Override
    public void runOpMode() {
        // Khởi tạo servo
        servo1 = hardwareMap.get(Servo.class, "servo_name");

        // Đợi lệnh bắt đầu từ trạm điều khiển
        waitForStart();

        // Vòng lặp chính
        while (opModeIsActive()) {
            // Đặt vị trí servo (0.0 đến 1.0 cho các servo tương thích FTC)
            servo1.setPosition(0.5); // Đặt vị trí giữa

            telemetry.addData("Servo Position", servo1.getPosition());
            telemetry.update();

            // Chờ
            sleep(500);
        }
    }
}