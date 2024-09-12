package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.EverglowLibrary.Systems.ElevatorSystem;


@TeleOp(name = "TestEel", group = "test")
public class TestEel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //ElevatorSystem elevators = new ElevatorSystem(this);
        ElevatorSystem elevators = new ElevatorSystem(this);

        waitForStart();
        boolean toggle = false;
        boolean toggle2 = false;
        int pos = ElevatorSystem.Level.DOWN.state;

        while (opModeIsActive()) {
            pos += -gamepad1.left_stick_y/100;
            if(gamepad1.cross)
                elevators.setPower(-0.4);
            else if(gamepad1.circle){
                elevators.setPower(0.4);
            }

            elevators.goTo(pos);
            telemetry.addData("place", pos);
            telemetry.addData("gampad:", gamepad1.left_stick_y);
            telemetry.update();
        }

    }


}