package org.firstinspires.ftc.teamcode.autos;

import android.graphics.Point;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.DrivetrainRR;
import org.firstinspires.ftc.teamcode.Manipulator;
import org.firstinspires.ftc.teamcode.VisionTest;

@Disabled
@Autonomous(name="BlueAutoRR", group="Auto")
public class BlueAutoRR extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-65.75, -39.25, 90);

        DrivetrainRR drive = new DrivetrainRR(hardwareMap, true);
        Drivetrain duckBoi = new Drivetrain(this); // Used ONLY for getting the ducks. THANKS ADITYA
        Manipulator manipulator = new Manipulator(this); // TODO: Test manipulator class
        // FIXME: Test manipulator class
        // FIXME: Test manipulator class
        // FIXME: Test manipulator class
        // FIXME: Test manipulator class


        Trajectory duckTrajectory = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-59.75, -65.50), 90)
                .build();

        //TODO: get endangle
        Trajectory deliveryTrajectory = drive.trajectoryBuilder(duckTrajectory.end())
                .splineTo(new Vector2d(-36, -36), 90)
                .build();

        Trajectory parkingTrajectory = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-36, -60), 90)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(duckTrajectory);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < 1.5){
           // duckBoi.spinDuck(0.7, 0.1, Math.PI * 1.75, 0, false);
        }
        drive.followTrajectory(deliveryTrajectory);

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
