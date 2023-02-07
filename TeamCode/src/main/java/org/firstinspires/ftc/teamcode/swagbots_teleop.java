package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Swagbots Teleop", group = "swagbots")
public class swagbots_teleop extends LinearOpMode {

    private DcMotor BottomLeft;
    private Servo hand;
    private DcMotor TopRight;
    private DcMotor BottomRight;
    private DcMotor TopLeft;
    private DcMotor arm;
    private double SpeedMult;

    private double CurrRotation;

    private ElapsedTime runtime = new ElapsedTime();
    private int encoder;

    @Override
    public void runOpMode() throws InterruptedException {

        BottomLeft = hardwareMap.get(DcMotor.class, "Bottom Left");
        hand = hardwareMap.get(Servo.class, "hand");
        TopRight = hardwareMap.get(DcMotor.class, "Top Right");
        BottomRight = hardwareMap.get(DcMotor.class, "Bottom Right");
        TopLeft = hardwareMap.get(DcMotor.class, "Top Left");
        arm = hardwareMap.get(DcMotor.class, "Arm");

        SpeedMult = 0.75;
        CurrRotation = 0;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        // initilization blocks, right motor = front right, left motor = front left, arm = back right, hand = back left
        BottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                SpeedControl();
                WheelControlBackup();
                ArmControl();
                HandControl();
                telemetry.addData("hand pos", hand.getPosition());
                telemetry.addData("bot left wheel power", BottomLeft.getPower());
                telemetry.addData("bot right wheel power", BottomRight.getPower());
                telemetry.addData("top left wheel power", TopLeft.getPower());
                telemetry.addData("top right wheel power", TopRight.getPower());
                telemetry.addData("curr power", SpeedMult);
                telemetry.addData("curr rotation", CurrRotation);
                telemetry.update();
            }
        }
    }

    private void SpeedControl(){
        if(gamepad1.right_bumper) {
            SpeedMult = (SpeedMult == 0.75) ? 0.30 : 0.75;
            sleep(250);
        }
    }

    /**
     * Describe this function...
     */
    private void WheelControl() {
        double relVertical, vertical;
        double relHorizontal, horizontal;
        double pivot;
        TopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivot = SpeedMult * gamepad1.right_stick_x;
        CurrRotation += Math.toRadians(pivot / 1371.955 * 134.528);
        double goalRotation = -CurrRotation;

        relHorizontal = SpeedMult * gamepad1.left_stick_y;

        double sin = Math.sin(goalRotation);
        double cos = Math.cos(goalRotation);

        double relHorX = ((sin == 0) ? 1 : sin) * relHorizontal;
        telemetry.addData("sin", Math.sin(goalRotation));
        double relHorY = ((cos == 0) ? 1 : cos) * relHorizontal;
        telemetry.addData("cos", Math.cos(goalRotation));

        relVertical = -SpeedMult * gamepad1.left_stick_x;
        double relVelX = ((sin == 0) ? 1 : sin) * relVertical;
        double relVelY = ((cos == 0) ? 1 : cos) * relVertical;

        horizontal = relHorY + relVelY;
        vertical = relHorX + relVelX;

        telemetry.addData("Horizontal",horizontal);
        telemetry.addData("Vertical",vertical);
        telemetry.addData("Pivot",pivot);
        telemetry.addData("relHor",relHorizontal);

        telemetry.addData("relVel",relVertical);
        TopRight.setPower(-pivot - vertical + horizontal);
        BottomRight.setPower(pivot - vertical - horizontal);
        TopLeft.setPower(-pivot - vertical - horizontal);
        BottomLeft.setPower(pivot - vertical + horizontal);

    }

    private void WheelControlBackup() {
        double vertical;
        double horizontal;
        double pivot;
        TopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivot = SpeedMult * gamepad1.right_stick_x;

        horizontal = SpeedMult * gamepad1.left_stick_y;
        vertical = -SpeedMult * gamepad1.left_stick_x;

        telemetry.addData("Horizontal",horizontal);
        telemetry.addData("Vertical",vertical);
        telemetry.addData("Pivot",pivot);
        TopRight.setPower(1.2 * (-pivot - vertical + horizontal));
        BottomRight.setPower(1.1 * (pivot - vertical - horizontal));
        TopLeft.setPower(-pivot - vertical - horizontal);
        BottomLeft.setPower(pivot - vertical + horizontal);
    }

    /**
     * Describe this function...
     */
    private void ArmControl() {
        telemetry.addData("Cheese:", encoder);
        arm.setPower(1);
        arm.setTargetPosition(encoder);
        if (encoder < 0) {
            encoder = 0;
        } else if (encoder > 6000) {
            encoder = 6000;
        } else {
            encoder += 30 * gamepad1.right_stick_y;
        }
        telemetry.update();
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
