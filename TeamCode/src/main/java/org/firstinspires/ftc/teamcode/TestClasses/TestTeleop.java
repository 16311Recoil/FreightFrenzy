package org.firstinspires.ftc.teamcode.TestClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Manipulator;
import org.firstinspires.ftc.teamcode.Sensors;
import org.firstinspires.ftc.teamcode.Turret;

@TeleOp(name = "testTeleop", group = "teleop")
public class TestTeleop extends OpMode{

    Drivetrain drivetrain;
    Manipulator arm;
    Turret turret;
    Sensors sensor;
    FtcDashboard dash;
    private boolean regular = false;
    private double init_Heading = 0;
    private boolean triggerPress = false;
    private double relativeHeading = 0;
    @Override
    public void init() {
        drivetrain = new Drivetrain(this);
        turret = new Turret(this);
        turret.setTurretMode(true);
        sensor = new Sensors(this);
        arm = new Manipulator(this);
        init_Heading = sensor.getFirstAngle();
        dash = FtcDashboard.getInstance();

        double armHeight = 0;

    }

    //drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(relativeHeading - init_Heading, sensor.getFirstAngle()), 1, 1); //lock angle
    //drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(drivetrain.lockNearestX(sensor.getFirstAngle() - init_Heading), sensor.getFirstAngle()), 1, 1); //lock x-mode

    @Override
    public void loop() {

        turret.teleOpControls(gamepad2.right_stick_x, gamepad2.x, gamepad2.a, gamepad2.dpad_right,  gamepad2.dpad_down,  gamepad2.dpad_left,  gamepad2.dpad_up);
        drivetrain.moveGyroTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, 1, sensor.getFirstAngle() - init_Heading);


        if (gamepad2.a){
            arm.magGrab();
        }

        if (gamepad2.b){
            arm.magRelease();
        }



        /*if (gamepad1.a){
            regular = false;
        }
        if (gamepad1.b){
            regular = true;
        }


        if (regular){
            double currentAngle = sensor.getFirstAngle();
            if (gamepad1.left_trigger > 0){ //whenever button pressed do locked angle driving
                if (!triggerPress){                              //get angle whenever button is first pressed
                    relativeHeading = sensor.getFirstAngle();
                }
                triggerPress = true;
                //drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(relativeHeading - init_Heading, sensor.getFirstAngle()), 1, 1); //lock angle
                drivetrain.moveGyroTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(drivetrain.lockNearestX(currentAngle - init_Heading), currentAngle), 1, 1, currentAngle); //lock x-mode
            }
            else { //regular driving
                triggerPress = false;
                drivetrain.moveGyroTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, 1, currentAngle);
            }

        }
        else{ //field centric driving
            drivetrain.moveGyroTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, 1, sensor.getFirstAngle() - init_Heading);
            turret.setTurretPower(gamepad2.right_stick_x);


        }*/

        drivetrain.setDashboard(dash);

        /*
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", gamepad1.left_stick_x);
        packet.put("y", gamepad1.left_stick_y);
        packet.put("right x", gamepad1.right_stick_x);
        packet.put("right y", gamepad1.right_stick_y);
        packet.put("getAngle()", sensor.getFirstAngle());
        packet.put("currentAngle", sensor.getFirstAngle() - init_Heading);
        packet.put("relativeHeading", relativeHeading);
        packet.put("angle Math", drivetrain.lockHeadingAngle(relativeHeading, sensor.getFirstAngle()));
        packet.put("angle Math 2", drivetrain.lockHeadingAngle(init_Heading, drivetrain.lockNearestX(sensor.getFirstAngle() - init_Heading)));
        packet.put("gamepad trigger", gamepad1.left_trigger);
        packet.put("mode", regular);
        dash.sendTelemetryPacket(packet);
         */

        telemetry.update();
    }



}
