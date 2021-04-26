package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


    @TeleOp(name="ShooterTest", group="Tele Op")
    public class ShooterTest extends OpMode {
        private ElapsedTime runtime = new ElapsedTime();


        DcMotor ShooterWheel;


        @Override
        public void init() {
            ShooterWheel = hardwareMap.dcMotor.get("shooter motor");

        }

        /*
        ---DIRECTION SETUP---
         */




        /*
        ---DIRECTIONS---
         */
        //make it so that positive moves forward, negative moves backwards

        @Override
        public void loop() {
            ShooterWheel.setPower((double) gamepad1.right_trigger);
        }
    }
            /*

           if (gamepad1.a == true) {
               wheel.setPower(1);
            }
            else if (gamepad1.a == false) {
                wheel.setPower(0);
            }
            if (gamepad1.b == true) {
                wheel.setPower(-1);
            }
          //  else if (gamepad1)

             */


