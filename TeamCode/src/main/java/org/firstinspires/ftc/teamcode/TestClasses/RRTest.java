package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DrivetrainRR;

@Autonomous(name = "RRTest", group = "Controlled")
public class RRTest extends LinearOpMode {

    private DrivetrainRR dr;

    @Override
    public void runOpMode() throws InterruptedException {

        dr = new DrivetrainRR(hardwareMap, true);

        //dr.followTrajectory();
    }
}
