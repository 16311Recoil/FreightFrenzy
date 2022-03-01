package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.net.HttpURLConnection;

//@TeleOp(name = "turretCalibration", group = "teleop")
public class TurretCalibration extends OpMode {

    Turret turret;

    public void init(){
        turret = new Turret(this);
        turret.setTurretMode(true);
    }

    public void loop(){
        turret.setTurretPower(gamepad1.right_stick_x * 0.5);

        if (gamepad1.y){
            turret.resetEncoder();
        }

        if (gamepad1.b){
            turret.setTurretMode(true);
        }

        if (gamepad1.x){
            turret.setTurretMode(false);
            //turret.setAngle(90);
        }

    }


}
