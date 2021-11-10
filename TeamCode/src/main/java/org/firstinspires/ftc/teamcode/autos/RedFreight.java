package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Crab;

@Autonomous(name="RedFreight", group="Auto")
public class RedFreight extends LinearOpMode {
    Crab crab;

    @Override
    public void runOpMode() throws InterruptedException{
        crab = new Crab(this);
        crab.violentlyRamWall(-1, 5000);
    }
}
