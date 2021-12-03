package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Manipulator;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.VisionTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="BlueAutoPathed", group="Auto")
public class BlueAutoPathed extends LinearOpMode {
    Crab robot;
    VisionTest.DeterminationPipeline pipeline;
    FtcDashboard dashboard;
    VisionTest.DeterminationPipeline.MarkerPosition pos;

    @Override
    public void runOpMode(){
        robot = new Crab(this);

        robot.getDrivetrain().lowerOdom();
        dashboard = FtcDashboard.getInstance();

        pipeline = new VisionTest.DeterminationPipeline();
        robot.getSensors().getWebcam().setPipeline(pipeline);
        robot.getSensors().getWebcam().openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.getSensors().getWebcam().startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted()) {
            TelemetryPacket p = new TelemetryPacket();
            dashboard.startCameraStream(robot.getSensors().getWebcam(), 30);

            telemetry.addData("pos", pipeline.getAnalysis());
            p.put("pos", pipeline.getAnalysis());
            dashboard.sendTelemetryPacket(p);

            pos = pipeline.getAnalysis();
        }


        waitForStart();

        robot.getTurret().setPosition(-90);


        // robot.getManip().mechGrab();

        int hub_pos;
        if (pos == VisionTest.DeterminationPipeline.MarkerPosition.LEFT)
            hub_pos = 1;
        else if (pos == VisionTest.DeterminationPipeline.MarkerPosition.CENTER)
            hub_pos = 2;
        else
            hub_pos = 3;

        // TODO: Uncomment manip code after adjusting values in manip class (check manip TODOs)

        // raise arm BEFORE we move forward
        robot.getManip().placePresetLevel(hub_pos);
        // manipulator.rotateClawUp();

        // move towards the hub
        robot.getDrivetrain().moveInches(18, 0.3, false, true);

        // drop block
        // manipulator.mechRelease();

        // go back
        robot.getDrivetrain().moveInches(-18, 0.3, false, true);

        // TODO: Go to duck
        // TODO: Get duck

        // park in freight area
        robot.getDrivetrain().moveInches(-50, 0.5, true, true);
    }
}
