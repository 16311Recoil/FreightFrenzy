package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Trajectories extends LinearOpMode {
    @Override
    public void runOpMode() {
        DrivetrainRR drive = new DrivetrainRR(hardwareMap, true);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(0, 0, 0)) //TODO figure out points
                .strafeRight(10)
                .forward(5)
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }
}