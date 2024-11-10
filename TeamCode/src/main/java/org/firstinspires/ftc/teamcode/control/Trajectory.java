package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Trajectory {
    public class MyOpmode extends LinearOpMode {
        @Override
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(10)
                    .forward(5)
                    .build();

            waitForStart();

            if(isStopRequested()) return;

            drive.followTrajectory(myTrajectory);
        }
    }
}
