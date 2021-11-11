package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;

@Autonomous(name="RedDuck", group="Auto")
public class RedDuck extends LinearOpMode {
    Crab crab;

    @Override
    public void runOpMode() throws InterruptedException{
        crab = new Crab(this);
        crab.setAlliance(false);
        crab.violentlyRamDucks();
    }
}
