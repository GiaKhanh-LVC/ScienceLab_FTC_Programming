package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
public class ServoControlModule {
    public Servo servo1;
    public Servo servo2;
    public Servo servo3,servo4;

    // Constructor that takes HardwareMap and servo name to initialize the servo
    public ServoControlModule() {
        servo1 = hardwareMap.get(Servo.class, "claw");
        servo1.setPosition(0.5); // Set initial position to center (optional)
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2 = hardwareMap.get(Servo.class, "box");
        servo2.setPosition(1.0); // Set initial position to center (optional)
        servo2.setDirection(Servo.Direction.FORWARD);
        servo3 = hardwareMap.get(Servo.class, "foldeing 1");
        servo3.setPosition(0.5); // Set initial position to center (optional)
        servo3.setDirection(Servo.Direction.FORWARD);
        servo4 = hardwareMap.get(Servo.class, "foldeing 2");
        servo4.setPosition(0.5); // Set initial position to center (optional)
        servo4.setDirection(Servo.Direction.FORWARD);
    }

    // Method to close the claw
    public void closeClaw( Servo servo) {
        if (servo != null) {
            servo.setPosition(1.0); // Position for closed claw
        }
    }

    // Method to open the claw
    public void openClaw(Servo servo) {
        if (servo != null) {
            servo.setPosition(0.0); // Position for open claw
        }
    }
    public void down(Servo servo1,Servo servo2){
        servo1.setPosition(0.7);
        servo2.setPosition(0.7);
    }
    public void up(Servo servo1,Servo servo2){
        servo1.setPosition(0.7);
        servo2.setPosition(0.7);
    }

}
