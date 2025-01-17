package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This sample demonstrates a basic  method of
 * detecting the team marker when lined up with
 * the sample regions over the first 3 stones.
 */
@TeleOp(name = "visionTestRedDuck", group = "test")
public class VisionTestRedDuck extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    DeterminationPipeline pipeline;

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
    //  The GAIN constants set the relationship between the measured position error,
    //  and how much power is applied to the drive motors.  Drive = Error * Gain
    //  Make these values smaller for smoother control.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

    private static final String VUFORIA_KEY =
            "AZSiMOH/////AAABmfP3UCfG4Un7sktEdqGC6b+IVXn+DiesrPGg6m3/fLyrjUX2QbKSdkc9yF2VsOrnhnd0twYsjqzw7g0Pugx75h3Jb8AF51d/90Y/byTitZyMIkTcxZyYtwZHogR7POp0c8lzep26+fKuQLMYK+fGUGduWvO/191isCSBh4zuH6zaKnzPXdMWc0r0q8vH403mREftEG2Zl/rpFX/mkqe3p98GIEVApXuc5kVSRO1Weer5mCr8kuDg68bLuPOa/3gBXfAQwFe3mIngZdHmscqQiWgOe80sjCzy1Pe7cVLEmiGnadzvZn8ONTwCzSMLNOT8i208CYCQHhy7USrmx4/ZyJ+ap5OOzDulonMGJO6rjxfH";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";


    @Override
    public void runOpMode()
    {
        waitForStart();

        boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
        double  drive           = 0;        // Desired forward power (-1 to +1)
        double  turn            = 0;        // Desired turning power (-1 to +1)
        Point center = new Point();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new DeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive())
        {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("avg1", pipeline.avg1);
            telemetry.addData("avg2", pipeline.avg2);
            telemetry.addData("avg3", pipeline.avg3);
            telemetry.addData("diff1", pipeline.diffOne);
            telemetry.addData("diff2", pipeline.diffTwo);


            telemetry.addData("Range",  "%5.1f inches", pipeline.targetDistance(pipeline.getAnalysis()));
            //telemetry.addData("Bearing","%3.0f degrees", pipeline.targetBearing(pipeline.getAnalysis()));
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

    }

    public static class DeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the positions
         */
        public enum MarkerPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final int TOLERANCE = 10;

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(35  ,90);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(175,90);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(105,160);
        static final int REGION_WIDTH =  65;
        static final int REGION_HEIGHT = 65;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3, diffOne, diffTwo;
        boolean side;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile MarkerPosition position = MarkerPosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the marker to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe contains the marker
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            diffOne = Math.abs(avg1 - avg3);// may need to change to min depending on color
            diffTwo = Math.abs(avg2 - avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(diffOne > TOLERANCE) // Was it from region 1?
            {
                if (side){
                    position = MarkerPosition.LEFT;
                }
                else{
                    position = MarkerPosition.CENTER;
                }

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(diffTwo > TOLERANCE) // Was it from region 2?
            {
                if (side){
                    position = MarkerPosition.CENTER;
                }
                else {
                    position = MarkerPosition.RIGHT;
                }

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(diffOne < TOLERANCE && diffTwo < TOLERANCE) // Was it from region 3?
            {
                if (side){
                    position = MarkerPosition.RIGHT;
                }
                else{
                    position = MarkerPosition.LEFT;
                }

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;

        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public MarkerPosition getAnalysis()
        {
            return position;
        }

        public void setSide(boolean warehouse){
            side = warehouse;
        }

        public int getAvg1() {
            return avg1;
        }

        public int getAvg2() {
            return avg2;
        }

        public int getAvg3() {
            return avg3;
        }

        public int getDiffOne() {
            return diffOne;
        }

        public int getDiffTwo() {
            return diffTwo;
        }

        public double targetDistance(Enum direction){

            final double MM_PER_INCH = 25.40 ;
            double  targetRange     = 0;        // Distance from camera to target in Inches
            double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
            Point center = new Point();

            if(direction == MarkerPosition.LEFT){
                center.x = Math.abs((region1_pointA.x - region1_pointB.x) / 2);
                center.y = Math.abs((region1_pointA.y - region1_pointB.y) / 2);
            }
            else if(direction == MarkerPosition.CENTER){
                center.x = Math.abs((region2_pointA.x - region2_pointB.x) / 2);
                center.y = Math.abs((region2_pointA.y - region2_pointB.y) / 2);
            }
            else{//RIGHT
                center.x = Math.abs((region3_pointA.x - region3_pointB.x) / 2);
                center.y = Math.abs((region3_pointA.y - region3_pointB.y) / 2);
            }

            double targetX = center.x / MM_PER_INCH; // Image X axis
            double targetY = center.y / MM_PER_INCH; // Image Z axis

            targetRange = Math.hypot(targetX, targetY);

            // target bearing is based on angle formed between the X axis and the target range line
            //targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));
            return targetRange;
        }
        public double targetBearing(String direction){
            final double MM_PER_INCH = 25.40 ;
            double  targetRange     = 0;        // Distance from camera to target in Inches
            double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
            Point center = new Point();

            if(direction.equals("LEFT")){
                center.x = Math.abs((region1_pointA.x - region1_pointB.x) / 2);
                center.y = Math.abs((region1_pointA.y - region1_pointB.y) / 2);
            }
            else if(direction.equals("CENTER")){
                center.x = Math.abs((region2_pointA.x - region2_pointB.x) / 2);
                center.y = Math.abs((region2_pointA.y - region2_pointB.y) / 2);
            }
            else{//RIGHT
                center.x = Math.abs((region3_pointA.x - region3_pointB.x) / 2);
                center.y = Math.abs((region3_pointA.y - region3_pointB.y) / 2);
            }

            double targetX = center.x / MM_PER_INCH; // Image X axis
            double targetY = center.y / MM_PER_INCH; // Image Z axis

            targetRange = Math.hypot(targetX, targetY);
            targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

            // target bearing is based on angle formed between the X axis and the target range line
            //targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));
            return targetBearing;
        }



    }
}