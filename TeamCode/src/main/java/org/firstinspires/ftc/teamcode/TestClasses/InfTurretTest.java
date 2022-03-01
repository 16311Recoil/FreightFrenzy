package org.firstinspires.ftc.teamcode.TestClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;

@Config
@Autonomous(name="360 Turret Test")
public class InfTurretTest extends LinearOpMode {
    public static int manipPos = 50, turretPos = 50, waitTime = 3000;
    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();

        Crab robot = new Crab(this);

        robot.getManip().setArmRotatorPower(0.3);
        robot.getManip().goToPosition(manipPos);
        robot.getTurret().setTurretPower(0.3);
        robot.getTurret().setPosition(turretPos);
        Thread.sleep(waitTime);
    }

    // turret clockwise movement moves arm down and counterclockwise moves arm up
    // ticks <0 is arm upwards or turret counterclockwise and ticks >0 is arm downwards or turret
    // clockwise
    // for a target angle A turret position is A * 360 / 135
    // for every -300 ticks of manip motor use 360 ticks of turret motor to compensate
    // i hate you so damn much aditya
}
