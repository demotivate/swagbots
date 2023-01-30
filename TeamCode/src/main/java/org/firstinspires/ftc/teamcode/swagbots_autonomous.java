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
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Swagbots Autonomous", group = "swagbots")
public class swagbots_autonomous extends LinearOpMode {

    private CRServo arm;
    private Servo hand;
//    private ColorSensor Coloursensor;
    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;
    private DcMotor BottomLeft;
    private ElapsedTime runtime = new ElapsedTime();

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
//        int currentColorIndex;

        arm = hardwareMap.get(CRServo.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");
//        Coloursensor = hardwareMap.get(ColorSensor.class, "Coloursensor");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");
        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Put initialization blocks here.
        waitForStart();

        // Put loop blocks here.
        //.4 power for 1 second for 2 blocks
        // 17 seconds at full power for full arm raise

        while(opModeIsActive()){
            RunSequence();
        }

        telemetry.update();

        // move 2 blocks forward, turn 45 degrees depending on what side your on, and place a cone on the tallest tower. turn 135 degrees, go back 2 blocks and repeat.
    }

    private void RunSequence(){

        HandControl();

        moveXY(-0.4, 0, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 1)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        moveXY(0, -0.4, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < .25)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        HandControl();

        ArmControl(1);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 17)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        HandControl();
        TerminateMovement();

        ArmControl(-1);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 17)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        moveXY(0, 0.4, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < .25)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();

        moveXY(0.4, 0, 0);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < 1)){
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        TerminateMovement();
    }

    private void TerminateMovement(){
        ArmControl(0);
        moveXY(0, 0, 0);
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
//    private void DetectColor() {
//        int currentColor = 0;
//
//        telemetry.addData("Blue", Coloursensor.blue());
//        telemetry.addData("Green", Coloursensor.green());
//        telemetry.addData("Red", Coloursensor.red());
//        currentColor = Color.argb(Coloursensor.alpha(), Coloursensor.red(), Coloursensor.green(), Coloursensor.blue());
//        RobotLog.ii("DbgLog", String.valueOf(currentColor));
//        telemetry.addData("hue", JavaUtil.colorToHue(currentColor));
//        telemetry.addData("saturation", JavaUtil.colorToSaturation(currentColor));
//        telemetry.addData("value", JavaUtil.colorToValue(currentColor));
//        currentColor = JavaUtil.hsvToColor(JavaUtil.colorToHue(currentColor), JavaUtil.colorToHue(currentColor), JavaUtil.colorToHue(currentColor));
//        RobotLog.ii("DbgLog", String.valueOf(currentColor));
//    }

    /**
     * Describe this function...
     */
    private void moveXY(double xVal, double yVal, double rVal) {
        omnidirectional(xVal, yVal, rVal);
//        sleep(duration);
//        omnidirectional(0, 0, 0);
    }

    /**
     * Describe this function...
     */
    private void omnidirectional(double x, double y, double pivot) {


        TopRight.setPower(-pivot - y + x);
        BottomRight.setPower(pivot - y - x);
        TopLeft.setPower(-pivot - y - x);
        BottomLeft.setPower(pivot - y + x);
    }
}