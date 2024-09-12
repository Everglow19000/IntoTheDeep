package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.EverglowLibrary.ThreadHandleLib.SequenceInSequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceRunner;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "DriveByAxis_Sequence", group = "main-drive")
public class DriveByAxis_Sequence extends LinearOpMode {

    private boolean seq1_toggle = false;
    private boolean seq2_toggle = false;
    private boolean seq3_toggle = false;
    private boolean seq4_toggle = false;
    private boolean gwheel_toggle = false;
    private boolean claw_toggle = false;
    private boolean elevator_toggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        ClawSystem clawSystem = new ClawSystem(this);
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        GWheelSystem gWheelSystem = new GWheelSystem(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        Servo planeServo = hardwareMap.get(Servo.class, "PlaneServo");
        planeServo.setPosition(0); //close servo mode
        double servoPos = 0.15; //open servo mode

        Sequence getReadyToDropSeq = sequenceControl.GetReadyToDropSeq();
        Sequence setUpAndUnderBlockSeq = sequenceControl.SetUpAndUnderBlockSeq();
        Sequence dropAndRetreatSeq = sequenceControl.DropAndRetreatSeq();
        Sequence getUpSeq = sequenceControl.GetUpAndReadyToDrop();
        SequenceRunner sequenceRunner = new SequenceRunner();
        Pose2d powerSet;

        sequenceControl = null; // no more use for that

        waitForStart();

        fourBarSystem.setMotorPower(0.85);
        //drive.setPoseEstimate(PoseInTiles(4, 4, 0));

        while (opModeIsActive()){
            try {
                if(gamepad2.square && !seq1_toggle){
                    sequenceRunner.RunSequence(getReadyToDropSeq);
                }
                seq1_toggle = gamepad2.square;

                if(gamepad2.cross && !seq2_toggle){
                    sequenceRunner.RunSequence(dropAndRetreatSeq);
                }
                seq2_toggle = gamepad2.cross;

                if(gamepad2.circle && !seq3_toggle){
                    sequenceRunner.RunSequence(setUpAndUnderBlockSeq);
                }
                seq3_toggle = gamepad2.circle;

                if(gamepad2.triangle && !seq4_toggle){
                    sequenceRunner.RunSequence(getUpSeq);
                }
                seq4_toggle = gamepad2.triangle;
            }catch (Exception e){
                telemetry.addData("exeption", e);
                telemetry.update();
            }

            if(gamepad2.right_bumper && !claw_toggle){
                clawSystem.toggle();
            }
            claw_toggle = gamepad2.right_bumper;

            if(gamepad1.right_bumper && !gwheel_toggle){
                gWheelSystem.toggle(true);
            }

            if(gamepad2.dpad_up){
                planeServo.setPosition(servoPos);
            }

            if (gamepad1.left_bumper && !gwheel_toggle){
                gWheelSystem.toggle(false);
            }
            gwheel_toggle = gamepad1.right_bumper || gamepad1.left_bumper;

            if(gamepad1.square && !elevator_toggle){
                elevatorSystem.toggleMax();
            }
            elevator_toggle = gamepad1.square;

            powerSet = TestControlledDrive.driveByAxis(new Pose2d(-gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x), TestControlledDrive.realAngle(drive.getPoseEstimate().getHeading()));

            drive.setWeightedDrivePower(TestControlledDrive.adjustedPowers(powerSet));

            drive.update();
            sequenceRunner.Update();
            fourBarSystem.updateP(0.8);
        }
        sequenceRunner.Interapt();
        sleep(1000);
    }

}

