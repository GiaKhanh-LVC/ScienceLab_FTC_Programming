package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Trajectory {
    public class MyOpmode extends LinearOpMode {
        @Override
        public void runOpMode() {
            MecanumDrive drive = new MecanumDrive(hardwareMap);

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
