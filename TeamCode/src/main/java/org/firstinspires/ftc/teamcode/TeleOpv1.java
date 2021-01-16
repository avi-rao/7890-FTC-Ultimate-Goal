package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="Tele Op")
public class TeleOpv1 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor center;

    /*
    ---DIRECTION SETUP---
     */
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        center = hardwareMap.dcMotor.get("center");

        /*
        ---DIRECTIONS---
         */
        //make it so that positive moves forward, negative moves backwards
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);
    }

    @Override
    public void loop() {

        //drive train ideas:
        /*
        on left joystick to drive move forward and backward on y axis and strafe on x axis of that joystick.
        right joystick use x axis to turn.

        strafe is on x axis on left joystick, forward and backwards on y, turning using buttons or bumpers or triggers.

        left joystick on y axis to move left side motors, right joystick on y axis to move right side motors, strafe on x axis joysticks
        */

        //claw ideas:
        /* - hardware: motor to rotate the claw, servo to open and close claw


        on right joystick y axis move motor to rotate claw, a button or a trigger or bumper (hold down or click)

        dpad up and down to rotate claw, dpad x axis to turn wheels, button/trigger/bumper to open and close

        triggers to rotate claw, bumpers to open and close claw !!!!!

         */

        /*on left joystick to drive move forward and backward on y axis and strafe on x axis of that joystick.
        right joystick use x axis to turn.

        triggers to rotate claw, bumpers to open and close claw
         */

        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        leftFront.setPower(drive + turn/2);
        leftBack.setPower(drive + turn/2);
        rightFront.setPower(drive - turn/2);
        rightBack.setPower(drive - turn/2);

        center.setPower(strafe);

    }
}
