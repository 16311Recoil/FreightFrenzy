package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "teleOp", group = "teleop")
public class teleOp extends OpMode{


    Crab robot;
    FtcDashboard dash;
    private boolean regular = false;
    private double init_Heading = 0;
    private boolean triggerPress = false;
    private double relativeHeading = 0;
    @Override
    public void init() {
        robot = new Crab(this);


        init_Heading = robot.getSensors().getFirstAngle();
        dash = FtcDashboard.getInstance();
        robot.getTurret().setTurretPower(0);
        robot.getTurret().setTurretMode(true);
    }

    //drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(relativeHeading - init_Heading, sensor.getFirstAngle()), 1, 1); //lock angle
    //drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(drivetrain.lockNearestX(sensor.getFirstAngle() - init_Heading), sensor.getFirstAngle()), 1, 1); //lock x-mode

    @Override
    public void loop() {

        if (gamepad1.x){
            init_Heading = robot.getSensors().getFirstAngle();
        }

        robot.teleOpControlsBrenden(init_Heading);

        telemetry.addData("angle", robot.getSensors().getFirstAngle());
        telemetry.addData("init_heading", init_Heading);




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

        //drivetrain.setDashboard(dash);

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
