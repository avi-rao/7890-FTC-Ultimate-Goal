package org.firstinspires.ftc.teamcode;

//trying to test stuff without the wobble goal


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

/*
7890 Space Lions 2019 "auton target red"
author: 7890 Software
GOALS: move to the target zone, park
DESCRIPTION: This code moves us to the target zone using only a color sensor and then parks. We don't use the wobble goal mech in this code
 */
@Autonomous(name="encoder test", group="Iterative Opmode")
public class EncoderTest extends OpMode
{



    /*
    ---MOTORS---
     */
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor center;
    CRServo wobble;

    /*
    ---SENSORS---
     */
    ColorSensor tapeSensor;
    ModernRoboticsI2cRangeSensor distSensor;

    // This helps us code for the red side specifically.
    String side = "red";


    /*
    ---STATES---
     */
    private StateMachine machine;

    // Moves our robot forward using encoders in order to sense the rings.
    EncoderState moveForwardState;
    // Senses rings.



    public void init() {

        /*
        ---HARDWARE MAP---
         */
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        center = hardwareMap.dcMotor.get("center");


        /*
        ---MOTOR DIRECTIONS---
         */
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        /*
        ---GROUPING---
         */
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        motors.add(center);


        /*
        ---USING STATES---
         */
        //Our robot is as big as a field tile, so we don't really need to move, especially with how our phone is placed.
      moveForwardState = new EncoderState(motors, 10, 1.0, "forward");
        //TODO: measure field for this, test camera angle of phone.



        /*
        ---ORDERING STATES---
         */
        moveForwardState.setNextState(null);


        /*
        moveForwardState.setNextState(tfodState);
        tfodState.setNextState(targetZoneState);
        targetZoneState.setNextState(null);
        //releaseWobbleGoal.setNextState(park);
       // park.setNextState(null);
        tfodState.setNextState(zeba);
        zeba.setNextState(null);
        //moveForwardState.setNextState(null);
        */




    }


    @Override
    public void start(){

        //wobble.setPower(1); //test value

        machine = new StateMachine(moveForwardState);

    }



    public void loop()  {
        /* telemetry used during testing
        telemetry.addData("target", moveForwardState.GetTarget());
        telemetry.addData("position", moveForwardState.GetPos());
        telemetry.update();
        telemetry.addData("did it work?", zeba.getSuccess());
        telemetry.update();
         */


        machine.update();

        telemetry.addData("target", moveForwardState.GetTarget());
        telemetry.addData("position", moveForwardState.GetPos());

        /* telemetry used during testing
        telemetry.addData("did it work?", zeba.getSuccess());
        telemetry.update();
        telemetry.addData("cntr value", tzone.getCntr());
        telemetry.update();
         */


    }

    public void wait(int time) {
        try {
            Thread.sleep(time * 1000);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}