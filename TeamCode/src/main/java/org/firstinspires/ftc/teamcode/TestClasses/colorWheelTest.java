package org.firstinspires.ftc.teamcode.TestClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "color Sensors Test", group = "Test")
public class colorWheelTest extends LinearOpMode {
    ColorSensor colorSensor;

    public void runOpMode(){
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Hue", getHue(colorSensor));
            telemetry.update();
        }
    }

    public float getHue(ColorSensor test){
        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV(test.red() * 8, test.green() * 8, test.blue() * 8, hsvValues);
        return hsvValues[0];
    }

}
