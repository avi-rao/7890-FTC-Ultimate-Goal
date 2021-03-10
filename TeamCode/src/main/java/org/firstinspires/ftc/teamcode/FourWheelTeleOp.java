package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FourTeleOp", group="Tele Op")
public class FourWheelTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;




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


        /*
        DRIVE TRAIN
         */

        double drive = (double) gamepad1.left_stick_y; //sets drive to the value of the left joystick y axis
        double strafe = (double) gamepad1.left_stick_x; //sets strafe to the value of the left joystick x axis
        double turn = (double) gamepad1.right_stick_x; //sets turn to the value of the right joystick x axis

        leftFront.setPower(drive + turn/2);
        leftBack.setPower(drive + turn/2);
        rightFront.setPower(drive - turn/2);
        rightBack.setPower(drive - turn/2);




        //triggers return double values between 0-1
        //bumpers return boolean values (true if pressed, false if not pressed)
        //servo to open and close claw, motor to rotate claw
        //setPower to motor, setPosition for servo

        /*
        hold down trigger to bring claw down, button to open and close with default pos is down
        right trigger to move claw up
        left trigger to move claw down
         */






        //.setPosition();

        if(gamepad1.left_bumper == true) {

        }
        else if (gamepad1.left_bumper == true) {

        }

    }
}
