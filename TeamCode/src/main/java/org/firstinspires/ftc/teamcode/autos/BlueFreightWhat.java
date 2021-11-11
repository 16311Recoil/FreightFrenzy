package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;

@Autonomous(name="BlueFreightWhat", group="Auto")
public class BlueFreightWhat extends LinearOpMode {
    Crab crab;

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();

        Thread.sleep(15000);
        crab = new Crab(this);
        crab.setAlliance(true);
        crab.violentlyRamWall(0.5, 1000);
        crab.violentlyRamWall(0.25, 2500);
    }
}
