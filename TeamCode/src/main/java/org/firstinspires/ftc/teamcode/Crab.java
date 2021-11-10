package org.firstinspires.ftc.teamcode;

// robot class

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Drivetrain;

public class Crab {

    Drivetrain drivetrain;
    Manipulator manip;
    Turret turret;
    Sensors sensors;

    private OpMode teleOp;
    private LinearOpMode auto;
    private double relativeHeading;
    private boolean triggerPressRight;

    /*
      --------------------
      |                  |
      | BLEG |SSH | RLEG |
      |    > |    | <    |
      |   ===========    |
      |        ^         |
      |       Hat        |
    | |🦀              🦀|
    | |   BSH      RSH   |
    | |🦀              🦀|
    | O------------------O
    (0, 0)-----
     */

    final double HAT_X_BLUE = 13.7, HAT_X_RED = 0;
    final double HAT_Y = 0;
    final double BLUE_LEG_X = 0, RIGHT_LEG_X = 0;
    final double LEG_Y = 0;

    static class Hub{
        double x;
        double y;
        Hub(double x, double y){
            this.x = x;
            this.y = y;
        }
    }
    final Hub
            BLUE_HUB = new Hub(0, 0),
            RED_HUB = new Hub(0, 0),
            SHARED_HUB = new Hub(0, 0);

    Hub team_hub;
    double x = 0, y = 0;

    boolean alliance; // true = blue

    public Crab(LinearOpMode opMode){

        auto = opMode;

        drivetrain = new Drivetrain(opMode);
        manip = new Manipulator(opMode);
        turret = new Turret(opMode);
        sensors = new Sensors(opMode);

        opMode.telemetry.addLine("Crabtrain Init Completed - Linear");
        opMode.telemetry.update();
    }

    public Crab(OpMode opMode) {
        teleOp = opMode;

        drivetrain = new Drivetrain(opMode);
        manip = new Manipulator(opMode);
        turret = new Turret(opMode);
        sensors = new Sensors(opMode);

        opMode.telemetry.addLine("Crabtrain Init Completed - Iterative");
        opMode.telemetry.update();
    }

    public void setAlliance(boolean blueSide){
        if (blueSide)
            team_hub = BLUE_HUB;
        else
            team_hub = RED_HUB;
    }

    // --- Auto Functions --- //

    /**
     * Send off robot to reach a target state
     * @param targetX Resulting x position of intake
     * @param targetY Resulting y position of intake
     * @param targetOrientation Resulting robot orientation in degrees
     * @param grab Grab or release object?
     * @param grabLevel Level to release at; 0: Floor; 1-3: Hub levels
     */
    public void executeAction(double targetX, double targetY, double targetOrientation, boolean grab, int grabLevel){
        driveTo(targetX, targetY, targetOrientation);
        if (grab){
            manip.grabFloor();
        }
        else {
            manip.placePresetLevel(grabLevel);
        }
    }

    public void driveTo(double x, double y, double orientation){

    }

    public void grabTeamFreight(){

    }

    // --- Utility Functions --- //
    public double angleTowards(double x, double y){
        return Math.atan2(y - this.y, x - this.x);
    }

    public void teleOpControlsBrenden(double init_Heading){
        double currentAngle = sensors.getFirstAngle();

        if (teleOp.gamepad1.right_trigger > 0){
            drivetrain.moveGyroTeleOp_Plus(teleOp.gamepad1.right_stick_x, teleOp.gamepad1.right_stick_y, drivetrain.lockHeadingAngle(drivetrain.lockNearestX(currentAngle - init_Heading), currentAngle), 1, 1, currentAngle); //lock x-mode
        }
        else if (teleOp.gamepad1.left_trigger > 0){
            if (!triggerPressRight){
                relativeHeading = currentAngle;
            }
            triggerPressRight = true;
            drivetrain.moveTeleOp_Plus(teleOp.gamepad1.right_stick_x, teleOp.gamepad1.right_stick_y, drivetrain.lockHeadingAngle(relativeHeading - init_Heading, currentAngle), 1, 1); //lock angle

        }
        else { //regular driving
            triggerPressRight = false;
            drivetrain.moveGyroTeleOp_Plus(teleOp.gamepad1.right_stick_x, teleOp.gamepad1.right_stick_y, teleOp.gamepad1.left_stick_x, 1, 1, currentAngle);
        }

        manip.teleOpControls(-teleOp.gamepad2.left_stick_y, teleOp.gamepad2.a, teleOp.gamepad2.b);
        turret.teleOpControls(-teleOp.gamepad2.right_stick_x);
    }

    public void teleOpControlsAditya(double init_Heading){
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Manipulator getManip() {
        return manip;
    }

    public Turret getTurret() {
        return turret;
    }

    public Sensors getSensors() {
        return sensors;
    }


}
