package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Calibrate Encoder", group = "swagbots")
public class CalibrateEncoder extends LinearOpMode {
    private DcMotor Arm;
    private DcMotor ForeArm;
    private int armEncoder;
    private int handEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm = hardwareMap.get(DcMotor.class, "Arm1");
        ForeArm = hardwareMap.get(DcMotor.class, "Hand");

        RunUsingEncoder(Arm);
        RunUsingEncoder(ForeArm);

        telemetry.addData("Current Arm Encoder", armEncoder);
        telemetry.addData("Current ForeArm Encoder", handEncoder);

        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                Arm.setTargetPosition(armEncoder);
                ForeArm.setTargetPosition(handEncoder);
                armEncoder += -gamepad1.right_stick_y;
                handEncoder += -gamepad1.left_stick_y;
                telemetry.addData("Current Arm Encoder", armEncoder);
                telemetry.addData("Current ForeArm Encoder", handEncoder);
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
}
