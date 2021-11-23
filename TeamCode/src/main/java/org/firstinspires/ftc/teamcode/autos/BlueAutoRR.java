package org.firstinspires.ftc.teamcode.autos;

import android.graphics.Point;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.DrivetrainRR;
import org.firstinspires.ftc.teamcode.Manipulator;
import org.firstinspires.ftc.teamcode.VisionTest;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="BlueAutoRR", group="Auto")
public class BlueAutoRR extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-39.25, -65.75, Math.toRadians(180));


        Drivetrain duckBoi = new Drivetrain(this); // Used ONLY for getting the ducks. THANKS ADITYA
        Manipulator manipulator = new Manipulator(this); // TODO: Test manipulator class
        DrivetrainRR drive = new DrivetrainRR(hardwareMap, true);
        // FIXME: Test manipulator class
        // FIXME: Test manipulator class
        // FIXME: Test manipulator class
        // FIXME: Test manipulator class


        drive.setPoseEstimate(startPose);
        /*Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-59.75, -65.50), Math.toRadians(225))
                .build();*/

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-39.25, -65.75, Math.toRadians(90)))
                .splineTo(new Vector2d(-59.75, -65.50), Math.toRadians(225))
                .back(40)
                .forward(2)
                .splineTo(new Vector2d(-60, -36), Math.toRadians(180))

                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(traj);
        //drive.followTrajectory(traj1);

//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (timer.seconds() < 1.5){
//            duckBoi.spinDuck(0.7, 0.1, Math.PI * 1.75, 0, false);
//        }
//        drive.followTrajectory(deliveryTrajectory);

        // TODO: Add vision stuff
        //if ()
        //manipulator.placePresetLevel();


        //drive.followTrajectory(deliveryTrajectory);



    }
}

/*
* ListOfVectors To go by
* a list of trajectories
* for each vector in teh lsit of vectors
*   add a trajectory
*   build the trajectory based on the vector
*
*
* */
