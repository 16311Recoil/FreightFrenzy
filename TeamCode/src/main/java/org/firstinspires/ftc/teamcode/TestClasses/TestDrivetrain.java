package org.firstinspires.ftc.teamcode.TestClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Sensors;

@Disabled
@TeleOp(name = "testDrivetrain", group = "teleop")
public class TestDrivetrain extends OpMode{

    Drivetrain drivetrain;
    Sensors sensor;
    FtcDashboard dash = FtcDashboard.getInstance();

    private double init_Heading = 0;

    @Override
    public void init() {
        drivetrain = new Drivetrain(this);
        sensor = new Sensors(this);
        init_Heading = sensor.getFirstAngle();


    }

    @Override
    public void loop() {

       drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, 1);
       drivetrain.getEncodersAll();
        drivetrain.setDashboard(dash);

        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);
        TelemetryPacket packet = new TelemetryPacket();
        dash.sendTelemetryPacket(packet);
        telemetry.update();
    }



}
