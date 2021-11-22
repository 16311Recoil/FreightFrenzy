package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // TODO: If you experience poor performance, enable this flag
        // System.setProperty("sun.java2d.opengl", "true");

        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setDriveTrainType(DriveTrainType.TANK)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.25, -65.75, Math.toRadians(90)))
                                .splineTo(new Vector2d(-59.75, -65.50), Math.toRadians(225))
                                .back(40)
                                .forward(2)
                                .splineTo(new Vector2d(-60, -36), Math.toRadians(180))

                                .build()
                )
                .start();

        MeepMeep mm2 = new MeepMeep(800)
                // Set field image
                .setDriveTrainType(DriveTrainType.TANK)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        /*drive.trajectorySequenceBuilder(new Pose2d(-39.25, -65.75, Math.toRadians(90)))
                                .splineTo(new Vector2d(-59.75, -65.50), Math.toRadians(225))
                                .back(40)
                                .forward(2)
                                .splineTo(new Vector2d(-60, -36), Math.toRadians(180))

                                .build()*/
                        drive.trajectorySequenceBuilder(new Pose2d(-32, -36, Math.toRadians(-45)))
                                .splineTo(new Vector2d(-9, -48), Math.toRadians(0))
                                .splineTo(new Vector2d(32, -64), Math.toRadians(0))
                                .forward(10)
                                .back(20)
                                .splineTo(new Vector2d(0, -38), Math.toRadians(90))

                                .build()
                )
                .start();

        MeepMeep mm3 = new MeepMeep(800)
                // Set field image
                .setDriveTrainType(DriveTrainType.TANK)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.25, 65.75, Math.toRadians(270)))
                                .splineTo(new Vector2d(-59.75, 65.50), Math.toRadians(135))
                                .back(40)
                                .forward(2)
                                .splineTo(new Vector2d(-60, 36), Math.toRadians(180))
                                .build()
                )
                .start();

        MeepMeep mm4 = new MeepMeep(800)
                // Set field image
                .setDriveTrainType(DriveTrainType.TANK)
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(new Pose2d(-32, 36, Math.toRadians(45)))
                                .splineTo(new Vector2d(-9, 48), Math.toRadians(0))
                                .splineTo(new Vector2d(32, 64), Math.toRadians(0))
                                .forward(10)
                                .back(20)
                                .splineTo(new Vector2d(0, 38), Math.toRadians(270))

                                .build()
                )
                .start();
    }

}