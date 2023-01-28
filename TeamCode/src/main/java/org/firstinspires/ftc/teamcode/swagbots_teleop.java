package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name = "Swagbots Teleop", group = "swagbots")
public class swagbots_teleop extends LinearOpMode {

    private DcMotor BottomLeft;
    private Servo hand;
    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;
    private CRServo arm;

    @Override
    public void runOpMode() throws InterruptedException {
        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        hand = hardwareMap.get(Servo.class, "hand");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");
        arm = hardwareMap.get(CRServo.class, "arm");

        // initilization blocks, right motor = front right, left motor = front left, arm = back right, hand = back left
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                WheelControl();
                ArmControl();
                HandControl();
                telemetry.addData("hand pos", hand.getPosition());
                telemetry.addData("bot left wheel power", BottomLeft.getPower());
                telemetry.addData("bot right wheel power", BottomRight.getPower());
                telemetry.addData("top left wheel power", TopLeft.getPower());
                telemetry.addData("top right wheel power", TopRight.getPower());
                telemetry.update();
            }
        }
    }
    /**
     * Describe this function...
     */
    private void WheelControl() {
        double vertical;
        double horizontal;
        double pivot;

        horizontal = -.25 * gamepad1.right_stick_y;
        vertical = 0.25 * gamepad1.right_stick_x;
        pivot = 0.25 * gamepad1.left_stick_x;
        TopRight.setPower(-pivot + (vertical - horizontal));
        BottomRight.setPower(-pivot + vertical + horizontal);
        TopLeft.setPower(pivot + vertical + horizontal);
        BottomLeft.setPower(pivot + (vertical - horizontal));
    }

    /**
     * Describe this function...
     */
    private void ArmControl() {
        float power = gamepad1.right_trigger - gamepad1.left_trigger;
        arm.setPower(power);
    }

    /**
     * Describe this function...
     */
    private void HandControl() {
        if (gamepad1.left_bumper) {
            hand.setPosition(hand.getPosition() == 0 ? 1 : 0);
            sleep(250);
        }
    }
}
