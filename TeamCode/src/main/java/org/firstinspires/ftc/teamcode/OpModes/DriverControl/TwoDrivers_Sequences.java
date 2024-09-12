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
import org.firstinspires.ftc.teamcode.DrivingSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TwoDrivers_Sequences", group = "main-drive")
public class TwoDrivers_Sequences extends LinearOpMode {

    private boolean seq1_toggle = false;
    private boolean seq2_toggle = false;
    private boolean seq3_toggle = false;
    private boolean seq4_toggle = false;
    private boolean gwheel_toggle = false;
    private boolean claw_toggle = false;
    private boolean elevator_toggle = false;
    private boolean left_Claw = false;
    private boolean right_claw = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        ClawSystem clawSystem = new ClawSystem(this);
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        GWheelSystem gWheelSystem = new GWheelSystem(this);
        DrivingSystem drivingSystem = new DrivingSystem(this);
        drivingSystem.setLocationInTiles(4, 4, 0);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        Servo planeServo = hardwareMap.get(Servo.class, "PlaneServo");
        planeServo.setPosition(0); //close servo mode
        double servoPos = 0.15; //open servo mode

        Sequence getReadyToDropSeq = sequenceControl.GetReadyToDropSeq();
        Sequence setUpAndUnderBlockSeq = sequenceControl.SetUpAndUnderBlockSeq();
        Sequence dropAndRetreatSeq = sequenceControl.DropAndRetreatSeq();
        Sequence MiddleDrop = sequenceControl.GetMiddleDrop();
        SequenceRunner sequenceRunner = new SequenceRunner();

        boolean isRest;

        sequenceControl = null; // no more use for that

        waitForStart();

        fourBarSystem.setMotorPower(0.85);


        boolean ajust = true, axis = false, control = false, slow = false;
        Pose2d location;
        Pose2d powers;
        double Px, Py, Pangle;
        boolean toggle_slow = false, toggle_control = false, toggle_axis = false, toggle_ajust = false;


        while (opModeIsActive()){
            isRest = elevatorSystem.getCurrentPos() == ElevatorSystem.Level.DOWN
                    && fourBarSystem.getTargetLevel() == FourBarSystem.Level.PICKUP;
            try {
                if(gamepad2.square && !seq1_toggle){
                    if(!isRest && sequenceRunner.IsSequenceDone()){
                        elevatorSystem.goTo(ElevatorSystem.Level.UP.state);
                        seq1_toggle = gamepad2.square;
                        continue;
                    }
                    sequenceRunner.RunSequence(getReadyToDropSeq);
                }
                seq1_toggle = gamepad2.square;

                if(gamepad2.cross && !seq2_toggle && !isRest){
                    sequenceRunner.RunSequence(dropAndRetreatSeq);
                }
                seq2_toggle = gamepad2.cross;

                if(gamepad2.circle && !seq3_toggle){
                    if(!isRest && sequenceRunner.IsSequenceDone()){
                        elevatorSystem.goTo(ElevatorSystem.Level.DOWN.state);
                        seq3_toggle = gamepad2.circle;
                        continue;
                    }
                    sequenceRunner.RunSequence(setUpAndUnderBlockSeq);
                }
                seq3_toggle = gamepad2.circle;

                if(gamepad2.triangle && !seq4_toggle){
                    if(!isRest && sequenceRunner.IsSequenceDone()){
                        elevatorSystem.goTo(ElevatorSystem.Level.MED.state);
                        seq4_toggle = gamepad2.triangle;
                        continue;
                    }
                    sequenceRunner.RunSequence(MiddleDrop);
                }
                seq4_toggle = gamepad2.triangle;

            }catch (Exception e){
                telemetry.addData("exeption", e);
            }

            if(gamepad2.left_bumper && !claw_toggle){
                clawSystem.toggle();
            }
            claw_toggle = gamepad2.left_bumper;

            if(gamepad1.right_bumper && !gwheel_toggle){
                //clawSystem.ChangePos(true);
                gWheelSystem.toggle(true);
            }

            if(gamepad2.dpad_up){
                planeServo.setPosition(servoPos);
            }

            if (gamepad1.left_bumper && !gwheel_toggle){
                //clawSystem.ChangePos(true);
                gWheelSystem.toggle(false);
            }
            gwheel_toggle = gamepad1.right_bumper || gamepad1.left_bumper;

            if(gamepad1.dpad_up && !elevator_toggle){
                elevatorSystem.toggleMax();
            }
            elevator_toggle = gamepad1.dpad_up;

            if(gamepad2.left_trigger > 0.5 && !left_Claw){
                clawSystem.MoveOneClaw(false);
            }
            left_Claw = gamepad2.left_trigger > 0.5;

            if(gamepad2.right_trigger > 0.5 && !right_claw){
                clawSystem.MoveOneClaw(true);
            }
            right_claw = gamepad2.right_trigger > 0.5;

            if(gamepad1.circle && !toggle_ajust) ajust = !ajust;
            if(gamepad1.triangle && !toggle_axis) axis = !axis;
            //if(gamepad1.square && !toggle_control) control = !control;
            //if(gamepad1.cross && !toggle_slow) slow = !slow;

            toggle_axis = gamepad1.triangle;
            toggle_ajust = gamepad1.circle;
            toggle_control = gamepad1.square;
            toggle_slow = gamepad1.cross;

            if(ajust) telemetry.addLine("Ajusted powers is On");
            if(axis) telemetry.addLine("Axis powers is On");
            if(control) telemetry.addLine("Controlled powers is On");
            if(slow) telemetry.addLine("Slow powers is On");

            location = drivingSystem.getPoseEstimate();
            telemetry.addData("X ", location.getX());
            telemetry.addData("Y ", location.getY());
            telemetry.addData("Heading ", location.getHeading());
            location = drivingSystem.locationInTiles();
            telemetry.addData("X in Tiles", location.getX());
            telemetry.addData("Y in Tiles", location.getY());


            Px = -gamepad1.left_stick_y;
            Py = -gamepad1.left_stick_x;
            Pangle = -gamepad1.right_stick_x;

            Px = linearInputToExponential(Px);
            Py = linearInputToExponential(Py);
            Pangle = linearInputToExponential(Pangle);

            if(slow) {
                Px /= 3;
                Py /= 3;
                Pangle /= 5;
            }

            powers = new Pose2d(Px, Py, Pangle);

            drivingSystem.allDrives(powers, ajust, axis, control);
            drivingSystem.update();
            sequenceRunner.Update();
            if(fourBarSystem.getTargetPosition() == FourBarSystem.Level.DROP)
                fourBarSystem.set4BarPositionByLevel(fourBarSystem.getTargetPosition());

            telemetry.addData("is finished?",
                    fourBarSystem.isFinish(fourBarSystem.getTargetPosition()));
            //fourBarSystem.updateP(0.8);
            telemetry.update();
        }
        sequenceRunner.Interapt();
        sleep(1000);
    }

    public double linearInputToExponential(double power){
        double base = 6;
        return (Math.pow(base, Math.abs(power)) - 1) / (base - 1) * Math.signum(power);
    }

}
