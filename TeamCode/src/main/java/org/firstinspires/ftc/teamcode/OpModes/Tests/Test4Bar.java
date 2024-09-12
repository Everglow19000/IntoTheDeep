package org.firstinspires.ftc.teamcode.OpModes.Tests;



import static java.lang.Math.max;
import static java.lang.Math.min;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;

import java.nio.channels.AsynchronousCloseException;

@TeleOp(name = "Test4Bar", group = "test")
public class Test4Bar extends LinearOpMode {
    FourBarSystem fourBarSystem;
    @Override
    public void runOpMode() throws InterruptedException {
        fourBarSystem = new FourBarSystem(this);
        //ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        //ClawSystem clawSystem = new ClawSystem(this);
        /*Sequence sequence = new Sequence(false, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.REST, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

         */
        waitForStart();
        DcMotorEx fourBarMotor = fourBarSystem.getFourBarMotor();
        double positionServo = fourBarSystem.getCurrentServoPosition()
                , positionMotor = fourBarSystem.getCurrentMotorPosition();
        fourBarSystem.setMotorPower(1);
        PIDFCoefficients pidfOrig;
        boolean toggleI = false;
        boolean toggleD = false;
        boolean toggleP = false;
        boolean toggleF = false;

        while(opModeIsActive()) {
            positionServo += gamepad1.right_stick_y / 1000;
            positionMotor += gamepad1.left_stick_y / 20;
            fourBarSystem.set4BarPosition((int)positionMotor);
            fourBarSystem.setServoPosition(positionServo);
            pidfOrig = fourBarMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad1.triangle){
                positionMotor = FourBarSystem.Level.DROP.state;
            }

            if(gamepad1.circle){
                positionMotor = FourBarSystem.Level.PICKUP.state;

            }
            if(gamepad1.dpad_up && !toggleI){
                pidfOrig.i += 0.1;
            }
            toggleI = gamepad1.dpad_up;

            if(gamepad1.dpad_down && !toggleD){
                pidfOrig.d += 0.1;
            }
            toggleD = gamepad1.dpad_down;

            if(gamepad1.dpad_left && !toggleF){
                pidfOrig.f += 0.1;
            }
            toggleF = gamepad1.dpad_left;

            if(gamepad1.dpad_right && !toggleP){
                pidfOrig.p += 1;
            }
            toggleP = gamepad1.dpad_right;


            telemetry.addData("motor:", positionMotor);
            telemetry.addData("servo:", positionServo);

            if(toggleI || toggleD || toggleF || toggleP) {
                fourBarMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION
                        , pidfOrig);
            }

            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("motor power:", fourBarMotor.getPowerFloat());
//            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
//                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
            telemetry.update();
            //fourBarSystem.updateP(0.35);
        }
    }
}