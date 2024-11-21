package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
public class ServoControlModule {
    private Servo servo1;
    private Servo servo2;
    // Constructor that takes HardwareMap and servo name to initialize the servo
    public ServoControlModule() {
        servo1 = hardwareMap.get(Servo.class, "servo claw 1");
        servo1.setPosition(0.5); // Set initial position to center (optional)
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2 = hardwareMap.get(Servo.class, "servo claw 2");
        servo2.setPosition(0.5); // Set initial position to center (optional)
        servo2.setDirection(Servo.Direction.FORWARD);
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

    // Optional: Method to set servo to an initial position (center position)
    public void servoSetup(Servo servo) {
        if (servo != null) {
            servo.setPosition(0.5); // Center position
        }
    }
}
