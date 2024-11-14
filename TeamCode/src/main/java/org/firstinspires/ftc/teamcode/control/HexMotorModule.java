package org.firstinspires.ftc.teamcode.control;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
public class HexMotorModule {


    public class MotorControlModule extends LinearOpMode {
        private DcMotor motor;

        @Override
        public void runOpMode() {

            motor = hardwareMap.get(DcMotor.class, "motorHex");

            motor.setDirection(DcMotor.Direction.FORWARD);

            waitForStart();

            while (opModeIsActive()) {
                // Điều khiển động cơ chạy với công suất 50%
                motor.setPower(0.5);

                // Thêm logic hoặc điều kiện dừng nếu cần thiết
                sleep(1000); // Chạy trong 1 giây

                // Dừng motor
                motor.setPower(0);
                sleep(1000); // Nghỉ trong 1 giây
            }
        }
    }

}
