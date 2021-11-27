package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.Drivetrain;

import java.util.Timer;

@Config
@Autonomous(name="Duck", group="Auto")
public class duckfuck extends LinearOpMode {
    Crab robot;
    public static double turnPower = 0.3, movePower = 0.1, turnTimeMs = 3000, moveAngle = 5/4;

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        // dt = new Drivetrain(this);
        robot = new Crab(this);
        double initHeading = robot.getSensors().getFirstAngle();


        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < turnTimeMs){
            robot.getDrivetrain().spinDuck(turnPower, movePower, moveAngle * Math.PI, robot.getSensors().getFirstAngle() - initHeading, false);
        }

    }


}
