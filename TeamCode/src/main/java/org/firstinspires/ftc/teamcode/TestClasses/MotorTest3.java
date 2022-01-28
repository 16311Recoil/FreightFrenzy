package org.firstinspires.ftc.teamcode.TestClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorTest3", group = "Controlled")
public class MotorTest3 extends OpMode {
    private int pos1 = 0, pos2 = 20;
    private boolean changeDpadUp, changeDpadDown = false;
    private DcMotorEx f, r, l, b;

    private FtcDashboard dashboard;

    public void init(){



        dashboard = FtcDashboard.getInstance();
    }

    public void loop(){

        TelemetryPacket packet = new TelemetryPacket();
        //packet.put("current pos", //.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData(" F current position", f.getCurrentPosition());
        telemetry.addData(" R current position", r.getCurrentPosition());
        telemetry.addData(" L current position", l.getCurrentPosition());
        telemetry.addData(" B current position", b.getCurrentPosition());

        telemetry.update();
    }


}