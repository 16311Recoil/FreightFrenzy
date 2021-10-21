package org.firstinspires.ftc.teamcode.TestClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Sensors;

@TeleOp(name = "testTeleop", group = "teleop")
public class TestTeleop extends OpMode{

    Drivetrain drivetrain;
    Sensors sensor;
    FtcDashboard dash = FtcDashboard.getInstance();
    private boolean regular = false;
    private double init_Heading = 0;
    private boolean triggerPress = false;
    private double relativeHeading = 0;
    @Override
    public void init() {
        drivetrain = new Drivetrain(this);
        sensor = new Sensors(this);
        init_Heading = sensor.getFirstAngle();


    }

    @Override
    public void loop() {

        if (gamepad1.a){
            regular = false;
        }
        if (gamepad1.b){
            regular = true;
        }

       // drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, 1);
        //drivetrain.moveGyroTeleOp_Plus();

        if (regular){
            if (gamepad1.left_trigger > 0){
                if (!triggerPress){
                    relativeHeading = sensor.getFirstAngle();
                }
                triggerPress = true;
                drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(relativeHeading - init_Heading, sensor.getFirstAngle()), 1, 1);
            }
            else {
                triggerPress = false;
                drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, 1);
            }

        }
        else{
            drivetrain.moveGyroTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, 1, sensor.getFirstAngle() - init_Heading);
        }

        drivetrain.setDashboard(dash);

        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", gamepad1.left_stick_x);
        packet.put("y", gamepad1.left_stick_y);
        packet.put("right x", gamepad1.right_stick_x);
        packet.put("right y", gamepad1.right_stick_y);
        packet.put("angle", sensor.getFirstAngle());
        packet.put("relativeHeading", relativeHeading);
        packet.put("angle Math", drivetrain.lockHeadingAngle(relativeHeading, sensor.getFirstAngle()));
        packet.put("gamepad trigger", gamepad1.left_trigger);
        packet.put("mode", regular);
        dash.sendTelemetryPacket(packet);
        telemetry.update();
    }



}
