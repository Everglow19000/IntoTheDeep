package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "servoTest", group = "test")
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "ClawR");
        double positionServo = 0;


        waitForStart();

        while (opModeIsActive()){
            positionServo += gamepad1.right_stick_y / 1000;
            if(gamepad1.square) positionServo = 1 - positionServo;
            servo.setPosition(positionServo);
            telemetry.addData("position:", positionServo);
            telemetry.update();
        }
    }
}
