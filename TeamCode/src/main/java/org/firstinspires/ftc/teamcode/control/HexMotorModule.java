package org.firstinspires.ftc.teamcode.control;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
public class HexMotorModule {


    public class MotorControlModule{
        private DcMotor motor;

        public void runOpMode() {

            motor = hardwareMap.get(DcMotor.class, "motorHex");
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
        public void SlideUp(){
            motor.setPower(1);
        }
        public void SlideDown(){
            motor.setPower(-1);
        }
    }

}