package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.fasterxml.jackson.databind.ext.SqlBlobSerializer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.EverglowLibrary.ThreadHandleLib.SequenceInSequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceRunner;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "StupidMecanum", group = "test")
public class StupidMecanum extends LinearOpMode {


DcMotorEx leftFront, leftRear, rightRear, rightFront;

private boolean seq1_toggle = false;
private boolean seq2_toggle = false;
private boolean seq3_toggle = false;
private boolean seq4_toggle = false;
private boolean gwheel_toggle = false;
private boolean claw_toggle = false;
private boolean elevator_toggle = false;

@Override
public void runOpMode() throws InterruptedException {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
    leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
    rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
    rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


    leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    //FourBarSystem fourBarSystem = new FourBarSystem(this);
    //ClawSystem clawSystem = new ClawSystem(this);
    //ElevatorSystem elevatorSystem = new ElevatorSystem(this);
    //GWheelSystem gWheelSystem = new GWheelSystem(this);
    //SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
    //Servo planeServo = hardwareMap.get(Servo.class, "PlaneServo");
    //planeServo.setPosition(0); //close servo mode
    double servoPos = 0.15; //open servo mode

    //Sequence getReadyToDropSeq = sequenceControl.GetReadyToDropSeq();
    //Sequence setUpAndUnderBlockSeq = sequenceControl.SetUpAndUnderBlockSeq();
    //Sequence dropAndRetreatSeq = sequenceControl.DropAndRetreatSeq();
    //Sequence getUpSeq = sequenceControl.GetUpAndReadyToDrop();
    SequenceRunner sequenceRunner = new SequenceRunner();

    waitForStart();

    //fourBarSystem.setMotorPower(0.85);
    while (!isStopRequested()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Tile X", poseEstimate.getX() / 60.5);
            telemetry.addData("Tile Y", poseEstimate.getY() / 60.5);
            try {
                if (gamepad2.square && !seq1_toggle) {
                    //sequenceRunner.RunSequence(getReadyToDropSeq);
                }
                seq1_toggle = gamepad2.square;

                if (gamepad2.cross && !seq2_toggle) {
                    //sequenceRunner.RunSequence(dropAndRetreatSeq);
                }
                seq2_toggle = gamepad2.cross;

                if (gamepad2.circle && !seq3_toggle) {
                    //sequenceRunner.RunSequence(setUpAndUnderBlockSeq);
                }
                seq3_toggle = gamepad2.circle;

                if (gamepad2.triangle && !seq4_toggle) {
                   // sequenceRunner.RunSequence(getUpSeq);
                }
                seq4_toggle = gamepad2.triangle;
            } catch (Exception e) {
                telemetry.addData("exeption", e);
            }

            if (gamepad2.left_bumper && !claw_toggle) {
                //clawSystem.toggle();
            }
            claw_toggle = gamepad2.left_bumper;

            if (gamepad1.right_bumper && !gwheel_toggle) {
                //clawSystem.ChangePos(false);
                //gWheelSystem.toggle(true);
            }

            if (gamepad2.dpad_up) {
                //planeServo.setPosition(servoPos);
            }

            if (gamepad1.left_bumper && !gwheel_toggle) {
                //clawSystem.ChangePos(false);
                //gWheelSystem.toggle(false);
            }
            gwheel_toggle = gamepad1.right_bumper || gamepad1.left_bumper;

            if (gamepad1.square && !elevator_toggle) {
                //elevatorSystem.toggleMax();
            }
            elevator_toggle = gamepad1.square;

            driveMecanum(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x, -gamepad1.right_stick_x));
            telemetry.update();
            //sequenceRunner.Update();
        }
    }


    public void driveMecanum (Pose2d powers){

        // in order to make the driving the same velocity for the same power in the x and y directions,
        // reduce the y power slightly
        double y = powers.getX();
        double x = powers.getY();
        double angle = powers.getHeading();
        // Determine how much power each motor should receive.
        double frontRightPower = y + x + angle;
        double frontLeftPower = y - x - angle;
        double backRightPower = y - x + angle;
        double backLeftPower = y + x - angle;

        // The method motor.setPower() only accepts numbers between -1 and 1.
        // If any number that we want to give it is greater than 1,
        // we must divide all the numbers equally so the maximum is 1
        // and the proportions are preserved.
        double norm = max(max(abs(frontRightPower), abs(frontLeftPower)), max(abs(backRightPower), abs(backLeftPower)));
        if (norm > 1) {
            frontRightPower /= norm;
            frontLeftPower /= norm;
            backRightPower /= norm;
            backLeftPower /= norm;
        }

        rightFront.setPower(frontRightPower);
        leftFront.setPower(frontLeftPower);
        rightRear.setPower(backRightPower);
        leftRear.setPower(backLeftPower);

    }

}
