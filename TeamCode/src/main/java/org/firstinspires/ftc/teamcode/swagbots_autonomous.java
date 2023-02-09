package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

@Autonomous(name = "Swagbots Autonomous", group = "swagbots")
public class swagbots_autonomous extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

//    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    //hardware
    private DcMotor arm;
    private Servo hand;
//    private ColorSensor Coloursensor;
    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;
    private DcMotor BottomLeft;

    //vars
    private ElapsedTime runtime = new ElapsedTime();

    private int encoder;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        //april tag stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        arm = hardwareMap.get(DcMotor.class, "Arm");
        hand = hardwareMap.get(Servo.class, "hand");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");
        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        hand.setPosition(0); /** check later if this start pos is correct **/

        // Put initialization blocks here.
//        waitForStart();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null){
            //default trajectory
            RunSequence(0);
        } else if(tagOfInterest.id == LEFT){
            //end left
            RunSequence(1);
        } else if(tagOfInterest.id == MIDDLE){
            //end middle
            RunSequence(2);
        } else {
            //end right
            RunSequence(3);
        }

        telemetry.update();
    }

    /**
     * the sequence of movements to move to a medium/small pole and place cone
     *
     * trajectory arguments meaning
     * 0 - default trajectory
     * 1 - end left
     * 2 - end middle
     * 3 - end right
     */
    private void RunSequence(int trajectory){
        HandControl();
        ArmControlPreset(1);
        while(opModeIsActive() && (runtime.seconds() < 1)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        moveXY(-.2, 0, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 2.8)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        ArmControlPreset(0);
        while(opModeIsActive() && (runtime.seconds() < 1)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        HandControl();

        moveXY(0, -.2, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 1.4)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        //arm up
        ArmControlPreset(3);
        while(opModeIsActive() && (runtime.seconds() < 5)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        HandControl();

        //arm down
        ArmControlPreset(0);
        while(opModeIsActive() && (runtime.seconds() < 5)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        TerminateMovement();

        moveXY(0, .2, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 1.4)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        moveXY(.2, 0, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < ((trajectory == 0) ? 2.8 : 1.4))){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        switch(trajectory){
            case 1:
                moveXY(0, .2, 0);
                runtime.reset();
                while(opModeIsActive() && (runtime.seconds() < 1.4)){
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                TerminateMovement();
                break;
            case 3:
                moveXY(0, -.2, 0);
                runtime.reset();
                while(opModeIsActive() && (runtime.seconds() < 1.4)){
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                TerminateMovement();
                break;
            default:
                break;
        }

        HandControl();
    }

    /**
     * terminates movement
     */
    private void TerminateMovement(){
        moveXY(0, 0, 0);
        sleep(100);
    }

    /**
     * arm control specific
     */
    private void ArmControl(int target) {
        arm.setPower(1);
        if (target < 0){
            target = 0;
        }
        else if(target > 11416){
            target = 11416;
        }
        arm.setTargetPosition(target);
    }

    /**
     *
     * @param position
     * position = 0: min 0
     * = 1: grab cone height 884
     * = 2: low pole 6736
     * = 3: medium pole 10473
     */
    private void ArmControlPreset(int position){
        arm.setPower(1);
        switch(position){
            case 0:
                arm.setTargetPosition(0);
            case 1:
                arm.setTargetPosition(884);
            case 2:
                arm.setTargetPosition(6736);
            case 3:
                arm.setTargetPosition(10473);
            default:
                return;
        }

    }

    private void HandControl(){
        hand.setPosition((hand.getPosition()) == 0 ? 1 : 0);
    }

    /**
     * Describe this function...
     */
    private void moveXY(double xVal, double yVal, double rVal) {
        omnidirectional(xVal, yVal, rVal);
    }

    /**
     * Describe this function...
     */
    private void omnidirectional(double x, double y, double pivot) {
        TopRight.setPower(1.2 * (-pivot - y + x));
        BottomRight.setPower(1.1 * (pivot - y - x));
        TopLeft.setPower((-pivot - y - x));
        BottomLeft.setPower((pivot - y + x));
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}