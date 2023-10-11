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
        RunUsingEncoder(Arm);
        RunUsingEncoder(ForeArm);

        CurrRotation = 0;

        // initilization blocks, right motor = front right, left motor = front left, arm = back right, hand = back left
//        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                WheelControl();
                telemetry.update();
            }
        }
    }

    private void RunUsingEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    // arm/forearm control notes
    // top - default
    // ready to pick up - left
    // pick up pos - down
    // place onto board pos - right
    private void ArmControl(){
        if(gamepad1.dpad_up){
            armEncoder = 0;
        }
        else if(gamepad1.dpad_left){
            armEncoder = 45;
        }
        else if(gamepad1.dpad_down){
            armEncoder = 40;
        }
        else if(gamepad1.dpad_right){
            armEncoder = 150;
        }

        telemetry.addData("Arm Encoder:", armEncoder);
        Arm.setPower(1);
        Arm.setTargetPosition(armEncoder);
        if (armEncoder < 0) {
            armEncoder = 0;
        }
        else if (armEncoder > 180) {
            armEncoder = 180;
        }
    }

    private void ForeArmControl(){
        if(gamepad1.dpad_up){
            foreArmEncoder = 0;
        }
        else if(gamepad1.dpad_left){
            foreArmEncoder = 270;
        }
        else if(gamepad1.dpad_down){
            foreArmEncoder = 270;
        }
        else if(gamepad1.dpad_right){
            foreArmEncoder = 90;
        }

        telemetry.addData("Forearm Encoder:", foreArmEncoder);
        ForeArm.setPower(1);
        ForeArm.setTargetPosition(foreArmEncoder);
        if (foreArmEncoder < 0) {
            foreArmEncoder = 0;
        }
        else if (foreArmEncoder > 330) {
            foreArmEncoder = 330;
        }
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
        pivot = gamepad1.left_stick_x / speedDiv;
        horizontal = gamepad1.right_stick_x / speedDiv;
        vertical = -1 * gamepad1.right_stick_y / speedDiv;

        double topRight = -pivot + vertical - horizontal;
        double bottomRight = -pivot + vertical + horizontal;
        double topLeft = pivot + vertical + horizontal;
        double bottomLeft = -pivot + vertical + horizontal;

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
