package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.MotorControlModule;
@TeleOp(name="Sample_TeleOp", group="TeleOp")
public class TeleOpMode extends OpMode {
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;

    private double theta;

    @Override
    public void init() {
//        leftFront = new MotorControlModule();
//        rightFront= new MotorControlModule();
//        leftRear= new MotorControlModule();
//        rightRear = new MotorControlModule();
//        ///////////////////////
//        leftFront.LeftFront();
//        rightFront.RightFront();
//        leftRear.LeftRear();
//        rightRear.RightRear();
        MotorControlModule mcm = new MotorControlModule();
        leftFront = mcm.leftFront;
        rightFront = mcm.rightFront;
        leftRear=mcm.leftRear;
        rightRear=mcm.rightRear;

    }

    @Override
    public void loop() {
        double driveX = gamepad1.left_stick_x;  // Chuyển động theo trục X
        double driveY = -gamepad1.left_stick_y; // Chuyển động theo trục Y (âm vì Y đảo ngược)
        double turn = gamepad1.right_stick_x;   // Thành phần quay


        double magnitude = Math.hypot(driveX, driveY);
        theta = Math.atan2(driveY, driveX);


        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));


        double leftFrontPower = (magnitude * cos / max) + turn;
        double rightFrontPower = (magnitude * sin / max) - turn;
        double leftRearPower = (magnitude * sin / max) + turn;
        double rightRearPower = (magnitude * cos / max) - turn;


        leftFront.setPower(Range.clip(leftFrontPower, -1.0, 1.0));
        rightFront.setPower(Range.clip(rightFrontPower, -1.0, 1.0));
        leftRear.setPower(Range.clip(leftRearPower, -1.0, 1.0));
        rightRear.setPower(Range.clip(rightRearPower, -1.0, 1.0));
    }
}
