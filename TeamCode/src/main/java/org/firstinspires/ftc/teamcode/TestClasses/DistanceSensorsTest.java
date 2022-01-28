package org.firstinspires.ftc.teamcode.TestClasses;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
        (name = "Distance Sensors Test", group = "Test")
public class DistanceSensorsTest extends LinearOpMode {
    /*
    private DistanceSensor sensorRange;
    private DistanceSensor sensorRange2;

     */
    NormalizedColorSensor colorSensor;
    DcMotor motor;
    View relativeLayout;
    final float[] hsvValues = new float[3];
    float gain = 2;
    private boolean x1 = false;
    private boolean x1new = false;
    private FtcDashboard dashboard;
    private Telemetry dashboardTelem;



    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        //sensorRange = hardwareMap.get(DistanceSensor.class, "transitionDS");
        //sensorRange2 = hardwareMap.get(DistanceSensor.class, "shooterDS");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        dashboard = FtcDashboard.getInstance();
        dashboardTelem = dashboard.getTelemetry();

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(false);
            }

            // actually execute the sample
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }

       /* motor = hardwareMap.dcMotor.get("screwMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();*/


        waitForStart();

        while(opModeIsActive()) {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(false);
            }
            TelemetryPacket packet = new TelemetryPacket();
            // generic DistanceSensor methods.
            /*
            telemetry.addData("transitionDS-Checkpoint", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("shooterDS-Checkpoint", String.format("%.01f in", sensorRange2.getDistance(DistanceUnit.MM)));
            telemetry.addData("loop speed loops/ms", (loopCount * 1.0/timer.milliseconds()));

             */
            telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
            telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

            // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }



            // Show the gain value via telemetry
            telemetry.addData("Gain", gain);

            // Tell the sensor our desired gain value (normally you would do this during initialization,
            // not during the loop)
            colorSensor.setGain(gain);

            // Check the status of the X button on the gamepad
            x1 = gamepad1.x;

            // If the button state is different than what it was, then act
            if (x1 != x1new) {
                // If the button is (now) down, then toggle the light
                if (x1) {
                    if (colorSensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight)colorSensor;
                        //light.enableLight(!light.isLightOn());

                    }
                }
            }
            x1new = x1;

            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            packet.put("Red", colors.red);
            packet.put("Blue", colors.blue);
            packet.put("Green", colors.green);
            packet.put("Hue", hsvValues[0]);
            packet.put("Saturation", hsvValues[1]);
            packet.put("Value", hsvValues[2]);
            packet.put("Alpha", colors.alpha);

            dashboard.sendTelemetryPacket(packet);

            /*if (colors.blue > colors.red){
                motor.setPower(0);
            }
            else{
                motor.setPower(0.4);
            }*/

            /* If this color sensor also has a distance sensor, display the measured distance.
             * Note that the reported distance is only useful at very close range, and is impacted by
             * ambient light and surface reflectivity. */
            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            telemetry.update();

            // Change the Robot Controller's background color to match the color detected by the color sensor.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });

            telemetry.update();
        }
    }

}
