package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.VisionTestRedWarehouse;

@Config
@Autonomous(name="RedShippingUnitPark", group="Auto")
public class RedUnitPark extends LinearOpMode {
    Crab robot;
    VisionTestRedWarehouse.DeterminationPipeline pipeline;
    FtcDashboard dashboard;
    VisionTestRedWarehouse.DeterminationPipeline.MarkerPosition pos;
    public static int extra = -4;
    public static double power = 0.3;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Crab(this);

        robot.getManip().setArmRotatorPower(0.3);
        robot.getDrivetrain().lowerOdom();
        robot.getManip().rotateClawUp();
        robot.getManip().mechGrab();
        dashboard = FtcDashboard.getInstance();






        waitForStart();

        robot.getDrivetrain().moveInches(24, power, false, 3);
        sleep(1000);
        robot.getDrivetrain().moveInches(-24, power, true, 3);
    }
}
