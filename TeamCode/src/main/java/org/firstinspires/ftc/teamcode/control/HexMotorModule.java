package org.firstinspires.ftc.teamcode.control;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
public class HexMotorModule {
        public DcMotor motor1, motor2, motor3;

        public HexMotorModule() {

            motor1 = hardwareMap.get(DcMotor.class, "LinearSlide 1");
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor2 = hardwareMap.get(DcMotor.class, "LinearSlide 2");
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setDirection(DcMotor.Direction.FORWARD);

        }
        public void SlideUp(DcMotor motor){
            motor.setPower(1);
        }
        public void SlideDown(DcMotor motor){
            motor.setPower(-1);
        }
    }

