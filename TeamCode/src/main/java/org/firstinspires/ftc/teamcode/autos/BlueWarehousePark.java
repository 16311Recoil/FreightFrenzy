package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.VisionTest;


@Autonomous(name="BlueWarehousePark", group="Auto")
public class BlueWarehousePark extends LinearOpMode {

    Crab robot;
    FtcDashboard dashboard;
    VisionTest.DeterminationPipeline pipeline;
    VisionTest.DeterminationPipeline.MarkerPosition pos;

    public void runOpMode() throws InterruptedException{

        dashboard = FtcDashboard.getInstance();
        robot = new Crab(this);
        robot.getDrivetrain().lowerOdom();
        robot.getManip().magGrab();









        waitForStart();

        robot.getManip().goToPosition(47);

        Thread.sleep(2000);

        robot.getDrivetrain().moveInches(-26, 0.25, true, 3.5);
       




    }


}
