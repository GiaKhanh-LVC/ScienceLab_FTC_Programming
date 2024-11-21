package org.firstinspires.ftc.teamcode.control;


// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localizer;
@Autonomous
public class TrajectoryModule extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Khởi tạo vị trí bắt đầu
        Pose2d startPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        // Khởi tạo targetPose
        Pose2d targetPose = new Pose2d(50, 50, Math.toRadians(90));

        // Lấy giá trị positionX và positionY
        double positionX = targetPose.position.x;
        double positionY = targetPose.position.y;


        // Build the trajectory
        TrajectoryActionBuilder myTrajectory =  drive.actionBuilder(startPose)
                .splineTo(new Vector2d(positionX, positionY), targetPose.heading)
                .waitSeconds(3);


        waitForStart();
        if (isStopRequested()) return;

        // Follow the built trajectory
        Action trajectoryActionChosen = myTrajectory.build();
        Actions.runBlocking(trajectoryActionChosen);
    }
}
