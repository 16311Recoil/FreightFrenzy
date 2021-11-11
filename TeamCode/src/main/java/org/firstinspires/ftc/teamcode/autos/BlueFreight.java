package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;

@Autonomous(name="BlueFreight", group="Auto")
public class BlueFreight extends LinearOpMode {
    Crab crab;

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        crab = new Crab(this);
        crab.setAlliance(true);
        crab.violentlyRamWall(0.5, 1000);
        crab.violentlyRamWall(0.25, 2500);
    }


}
