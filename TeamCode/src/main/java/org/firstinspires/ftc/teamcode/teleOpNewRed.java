package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "teleOpNew", group = "teleop")
public class teleOpNewRed extends OpMode{


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

        robot.teleOpNewControls(init_Heading);


        telemetry.addData("angle", robot.getSensors().getFirstAngle());
        telemetry.addData("init_heading", init_Heading);


        telemetry.update();
    }



}
