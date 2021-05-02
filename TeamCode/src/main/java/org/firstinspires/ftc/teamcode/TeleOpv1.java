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

    /*
    ---DRIVE TRAIN MOTORS---
     */
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor center;

    /*
    ---WOBBLE GOAL MOTORS & SERVOS---
     */
    DcMotor clawMotor;
    //CRServo clawServo;

    /*
    ---DIRECTION SETUP---
     */
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    @Override
    public void init() {

        /*
        ---HARDWARE MAPPING---
        */
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        center = hardwareMap.dcMotor.get("center");

        clawMotor = hardwareMap.dcMotor.get("claw motor");
        //clawServo = hardwareMap.crservo.get("claw servo");

        /*
        ---DIRECTIONS---
         */
        //So that positive values move forward and negative values move backwards
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);
        center.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        /*
        ---DRIVE TRAIN---
         */
        double drive = (double) - gamepad1.left_stick_y; //sets drive to the value of the left joystick y axis
        double strafe = (double) gamepad1.left_stick_x; //sets strafe to the value of the left joystick x axis
        double turn = (double) gamepad1.right_stick_x; //sets turn to the value of the right joystick x axis

        //Calculations so that we can do more complex movements than just the cardinal directions such as:
        //move diagonally, turn while moving, etc.
        leftFront.setPower(drive + turn/2);
        leftBack.setPower(drive + turn/2);
        rightFront.setPower(drive - turn/2);
        rightBack.setPower(drive - turn/2);

        center.setPower(strafe);

        /*
        ---WOBBLE GOAL MECHANISM---
         */
        //We control the movement of the "wrist" using a trigger
        clawMotor.setPower((double) -gamepad1.left_trigger);

        //isOpen keeps track of if the claw is open or closed
        boolean isOpen = true;

        /*
        //Let's us open and close the claw using just one button (the left bumper)
        if(isOpen == true && gamepad1.left_bumper == true) {
            clawServo.setPower(-1);
            isOpen = false;
        }
        else if (isOpen == false && gamepad1.left_bumper == true) {
            clawServo.setPower(1);
            isOpen = true;
        }

         */
    }
}
