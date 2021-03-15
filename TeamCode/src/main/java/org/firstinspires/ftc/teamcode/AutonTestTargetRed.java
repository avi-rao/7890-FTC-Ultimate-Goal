package org.firstinspires.ftc.teamcode;

//trying to test stuff without the wobble goal


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.ArrayList;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
7890 Space Lions 2019 "auton target red"
author: 7890 Software
GOALS: move to the target zone, park
DESCRIPTION: This code moves us to the target zone using only a color sensor and then parks. We don't use the wobble goal mech in this code
 */
@Autonomous(name="auton target red", group="Iterative Opmode")
public class AutonTestTargetRed extends OpMode
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

    String side = "red";
    // This helps us code for the red side specifically.

    /*
    ---STATES---
     */
    private StateMachine machine;

    EncoderState moveForwardState;
    // Moves our robot forward using encoders in order to sense the rings.
    TensorFlowState tfodState;
    // Senses rings.
    RunToTargetZoneState targetZoneState;
    // CRServoState releaseWobbleGoal;
    ColorSenseStopState park;
    // Stops the robot at the white tape in order to park.
    RunToTargetZoneStateColor tzone;

    //EncoderState moveState;
    //TensorFlowState testState;


    public void init() {

        /*
        ---HARDWARE MAP---
         */
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        center = hardwareMap.dcMotor.get("center");

        tapeSensor =  hardwareMap.get(ColorSensor.class, "color sensor");

        //distSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distance sensor");

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
        moveForwardState = new EncoderState(motors, 5, .5, "forward"); //change calculations

        /*
        moveForwardState = new EncoderState(motors, 2, 1.0, "forward"); //change calculations
        tfodState = new TensorFlowState(hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        targetZoneState = new RunToTargetZoneState(motors, tapeSensor, distSensor, "red");
        //releaseWobbleGoal = new CRServoState(wobble, 2, 100);

         */

       // tzone = new RunToTargetZoneStateColor(motors, tapeSensor, "red");
        //park = new ColorSenseStopState(motors, tapeSensor, "white", .5, "backward");



        /*
        moveForwardState.setNextState(tfodState);
        tfodState.setNextState(targetZoneState);
        targetZoneState.setNextState(null);
        */

        //releaseWobbleGoal.setNextState(park);
       // park.setNextState(null);
        //tzone.setNextState(null);
        moveForwardState.setNextState(null);

    }


    @Override
    public void start(){

        //wobble.setPower(1); //test value
        telemetry.addData("target", moveForwardState.GetTarget());
        telemetry.addData("position", moveForwardState.GetPos());
        telemetry.update();
        machine = new StateMachine(moveForwardState);

    }



    public void loop()  {
        telemetry.addData("target", moveForwardState.GetTarget());
        telemetry.addData("position", moveForwardState.GetPos());
        telemetry.update();
        machine.update();
       // telemetry.addData("cntr value", tzone.getCntr());
       // telemetry.update();


    }

    public void wait(int time) {
        try {
            Thread.sleep(time * 1000);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}