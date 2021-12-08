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
import org.firstinspires.ftc.teamcode.VisionTestRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="BlueAutoPathed", group="Auto")
public class BlueAutoPathed extends LinearOpMode {
    Crab robot;
    VisionTest.DeterminationPipeline pipeline;
    FtcDashboard dashboard;
    VisionTest.DeterminationPipeline.MarkerPosition pos;
    int extra = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Crab(this);

        robot.getDrivetrain().lowerOdom();
        robot.getManip().rotateClawUp();
        robot.getManip().mechGrab();
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

        robot.getTurret().setPosition(-83);


        // robot.getManip().mechGrab();

        int hub_pos;

        if (pos == VisionTest.DeterminationPipeline.MarkerPosition.LEFT)
            hub_pos = 50;
        else if (pos == VisionTest.DeterminationPipeline.MarkerPosition.CENTER)
            hub_pos = 105;
        else{
            hub_pos = 220;
            extra++;
        }



        // TODO: Uncomment manip code after adjusting values in manip class (check manip TODOs)

        // raise arm BEFORE we move forward
        //robot.getManip().placePresetLevel(hub_pos);
        robot.getManip().goToPosition(hub_pos);
        // manipulator.rotateClawUp();

        // move towards the hub
        robot.getDrivetrain().moveInches(13.5 + extra, 0.3, false, 4);
        Thread.sleep(2000);

        // drop block
        robot.getManip().mechRelease();
        Thread.sleep(2000);
        // drop block
        // manipulator.mechRelease();

        // go back

        robot.getDrivetrain().moveInches(-13.5 - extra, 0.3, false, 4);

        Thread.sleep(2000);
        robot.getManip().goToPosition(80);
        // TODO: Go to duck
        // TODO: Get duck

        // park in freight area
        robot.getDrivetrain().moveInches(-46, 0.4, true, 5);
        Thread.sleep(2000);

        robot.getManip().rotateClawDown();
        robot.getManip().mechRelease();
        Thread.sleep(2000);
    }
}
