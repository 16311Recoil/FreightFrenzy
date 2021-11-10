package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;

@Autonomous(name="BlueFreight", group="Auto")
public class BlueFreightWhat extends LinearOpMode {
    Crab crab;

    @Override
    public void runOpMode() throws InterruptedException{
        Thread.sleep(15000);
        crab = new Crab(this);
        crab.violentlyRamWall(1, 5000);
    }


}
