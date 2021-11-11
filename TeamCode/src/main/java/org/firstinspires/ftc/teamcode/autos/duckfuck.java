package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.Drivetrain;

import java.util.Timer;

@Autonomous(name="Duck", group="Auto")
public class duckfuck extends LinearOpMode {
    Drivetrain dt;

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        dt = new Drivetrain(this);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 3000){
            dt.spinDuck(0.5, 0.1, (1.75) * Math.PI, 0, true);
        }

    }


}
