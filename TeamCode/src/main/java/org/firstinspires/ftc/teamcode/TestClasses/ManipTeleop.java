package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Manipulator;
import org.firstinspires.ftc.teamcode.Turret;

@TeleOp(name="ManipTeleop", group="teleop")
public class ManipTeleop extends OpMode {
    Manipulator manip;
    double manipHeight = 6;
    int count = 0;
    boolean changeDpadUp, changeDpadDown, changeX;
    Turret turret;


    @Override
    public void init(){
        manip = new Manipulator(this);
        turret = new Turret(this);
        manip.setArmRotatorPower(0.5);
        turret.setTurretMode(true);

    }


    @Override
    public void loop(){



        turret.teleOpControls(gamepad1.left_stick_x);


        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("encoder", manip.getEncoder());
        telemetry.update();
    }

}
