package org.firstinspires.ftc.teamcode.control;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorControlModule {
    public DcMotorEx leftFront,rightFront,leftRear,rightRear;
    public MotorControlModule() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
//    // Method to configure the left front motor
//    public void LeftFront() {
//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftFront.setDirection(DcMotor.Direction.FORWARD);
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    // Method to configure the right front motor
//    public void RightFront() {
//        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    // Method to configure the left rear motor
//    public void LeftRear() {
//        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
//        leftRear.setDirection(DcMotor.Direction.FORWARD);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    // Method to configure the right rear motor
//    public void RightRear() {
//        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
//        rightRear.setDirection(DcMotor.Direction.REVERSE);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

}
