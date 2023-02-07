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



@Autonomous(name = "Swagbots Autonomous", group = "swagbots")
public class swagbots_autonomous extends LinearOpMode {

    private DcMotor arm;
    private Servo hand;
//    private ColorSensor Coloursensor;
    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;
    private DcMotor BottomLeft;

//    private OpenCvCamera camera;
//    private AprilTagDetection aprilTagDetection;
//    private DcMotor LiftMotor;

    private ElapsedTime runtime = new ElapsedTime();

    private int encoder;

    /**
     * Describe this function...
     */
    private int CheckColor() {
        int color = 0;

        if (Color.red(color) > 100 && JavaUtil.colorToSaturation(color) > 100 && JavaUtil.colorToValue(color) > 100) {
            return 1;
        }
        if (Color.green(color) > 100 && JavaUtil.colorToSaturation(color) > 100 && JavaUtil.colorToValue(color) > 100) {
            return 2;
        }
        if (Color.blue(color) > 100 && JavaUtil.colorToSaturation(color) > 100 && JavaUtil.colorToValue(color) > 100) {
            return 3;
        }
        return 0;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "Arm");
        hand = hardwareMap.get(Servo.class, "hand");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");
        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        hand.setPosition(0); /** check later if this start pos is correct **/

        // Put initialization blocks here.
        waitForStart();

        while(opModeIsActive()){
            RunSequence();
        }

        telemetry.update();
    }

    private void RunSequence(){

        HandControl();

        moveXY(-.2, 0, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 2.8)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        HandControl();

        moveXY(0, -.2, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 1.4)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        //arm up

        HandControl();

        //arm down

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
        while(opModeIsActive() && (runtime.seconds() < 2.8)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        HandControl();
    }

    private void TerminateMovement(){
        ArmControl(0);
        moveXY(0, 0, 0);
        sleep(100);
    }

    /**
     * Describe this function...
     */
    private void ArmControl(double direction) {
        arm.setPower(direction);
//        sleep(duration);
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
}