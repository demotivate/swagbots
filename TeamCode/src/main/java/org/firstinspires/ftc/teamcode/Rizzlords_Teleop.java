package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Rizzlords_Teleop", group = "Rizzlords")
public class Rizzlords_Teleop extends LinearOpMode {

    //Drive Motors
    private DcMotor BottomLeft;
    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;


    //Arm and Hand Motors
    private DcMotor Arm;
    private DcMotor ForeArm;

    private double CurrRotation;

    private ElapsedTime runtime = new ElapsedTime();
    private double encoderPower;
    private int armEncoder;
    private int foreArmEncoder;


    @Override
    public void runOpMode() throws InterruptedException {

        //Drive motors
        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");

        //Arm hand motors
        Arm = hardwareMap.get(DcMotor.class, "Arm1");
        ForeArm = hardwareMap.get(DcMotor.class, "Hand");
        encoderPower = .8;
        RunUsingEncoder(Arm);
        RunUsingEncoder(ForeArm);

        CurrRotation = 0;

        // initilization blocks, right motor = front right, left motor = front left, arm = back right, hand = back left
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        TopLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                WheelControl();
                ArmControl();
                ForeArmControl();
                telemetry.update();
            }
        }
    }

    private void RunUsingEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(.1);
    }

    // arm/forearm control notes
    // top - default
    // ready to pick up - left
    // pick up pos - down
    // place onto board pos - right
    private void ArmControl(){
        int MAX = 933;
        int MIN = 0;
        if(gamepad1.dpad_up){
            armEncoder = 0;
        }
        else if(gamepad1.dpad_left){
            armEncoder = 0;
        }
        else if(gamepad1.dpad_down){
            armEncoder = 0;
        }
        else if(gamepad1.dpad_right){
            armEncoder = 933;
        }

        telemetry.addData("Arm Encoder:", armEncoder);
        Arm.setPower(encoderPower);
        Arm.setTargetPosition(armEncoder);
        if (armEncoder < MIN) {
            armEncoder = MIN;
        }
        else if (armEncoder > MAX) {
            armEncoder = MAX;
        }
    }

    private void ForeArmControl(){
        int MAX = 0;
        int MIN = -315;
        if(gamepad1.dpad_up){
            foreArmEncoder = 0;
        }
        else if(gamepad1.dpad_left){
            foreArmEncoder = -315;
        }
        else if(gamepad1.dpad_down){
            foreArmEncoder = -315;
        }
        else if(gamepad1.dpad_right){
            foreArmEncoder = -308;
        }

        telemetry.addData("Forearm Encoder:", foreArmEncoder);
        ForeArm.setPower(encoderPower);
        ForeArm.setTargetPosition(foreArmEncoder);
        if (foreArmEncoder < MIN) {
            foreArmEncoder = MIN;
        }
        else if (foreArmEncoder > MAX) {
            foreArmEncoder = MAX;
        }
    }

    private void HandControl(){
//        MAKE IT TOGGLE FUNCTION
    }

    private void WheelControl() {
        double vertical;
        double horizontal;
        double pivot;
        TopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speedDiv = (gamepad1.left_trigger > 0) ? 3.5 : 1;
        pivot = gamepad1.right_stick_x / speedDiv;
        horizontal = gamepad1.left_stick_x / speedDiv;
        vertical = -1 * gamepad1.right_stick_y / speedDiv;

        double topRight = -pivot + vertical - horizontal;
        double bottomRight = 2 * (-pivot + vertical + horizontal);
        double topLeft = 2 * (pivot + vertical + horizontal);
        double bottomLeft = 2 * (-pivot + vertical + horizontal);

//        double topRight = (-pivot - vertical + horizontal);
//        double bottomRight = 2 * (pivot - vertical - horizontal);
//        double topLeft = (-pivot - vertical - horizontal);
//        double bottomLeft = pivot - vertical + horizontal;

        TopRight.setPower(topRight);
        BottomRight.setPower(bottomRight);
        TopLeft.setPower(topLeft);
        BottomLeft.setPower(bottomLeft);

        telemetry.addData("BottomLeft", bottomLeft);
        telemetry.addData("TopRight", topRight);
        telemetry.addData("TopLeft", topLeft);
        telemetry.addData("bottomRight", bottomRight);
    }
}
