package org.firstinspires.ftc.teamcode.control;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.control.ServoControlModule;
import org.firstinspires.ftc.teamcode.control.MotorControlModule;
import org.firstinspires.ftc.teamcode.control.HexMotorModule;
public class Activity {
    ServoControlModule scm = new ServoControlModule();
    MotorControlModule motors = new MotorControlModule();
    HexMotorModule hex = new HexMotorModule();
    Servo claw1 = scm.servo1;
    Servo box = scm.servo2;
    Servo fold3 = scm.servo3;
    Servo fold4 = scm.servo4;
    DcMotor linear_1 = hex.motor1;
    DcMotor linear_2 = hex.motor2;
    public void claw_catch(){
        hex.SlideUp(linear_1);
        scm.openClaw(claw1);
        scm.down(fold3,fold4);
        scm.closeClaw(claw1);
        hex.SlideDown(linear_1);
        scm.up(fold3,fold4);
        scm.openClaw(claw1);
        // cần them nhan dien tu camera
    }
    public void score_1(){
        hex.SlideUp(linear_2);
        box.setPosition(0.0);
        box.setPosition(1.0);
        hex.SlideDown(linear_2);
    }
}
