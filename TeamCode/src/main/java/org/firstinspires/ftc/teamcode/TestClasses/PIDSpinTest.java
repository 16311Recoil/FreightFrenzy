package org.firstinspires.ftc.teamcode.TestClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;

@Config
@Autonomous(name="PIDSpinTest", group="Auto")
public class PIDSpinTest extends LinearOpMode {
    Crab robot;
    public static double power = 0.3;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Crab(this);
        waitForStart();

        telemetry.addData("Starting at angle (in deg)", robot.getSensors().getFirstAngle() * 180 / Math.PI);
        //robot.getDrivetrain().turnToPID(0, robot.getSensors(), 4);

        // robot.getDrivetrain().setAllMotors(0.2);
    }
}
