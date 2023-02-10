package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Reset Encoder", group = "swagbots")
public class ResetEncoder extends LinearOpMode {

    private DcMotor arm;
    private int encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "Arm");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                if(gamepad1.a){
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                arm.setTargetPosition(encoder);
                encoder += 30 * -gamepad1.right_stick_y;
            }
        }
    }
}
