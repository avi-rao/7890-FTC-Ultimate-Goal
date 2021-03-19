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
    //TODO: testing all 4 states together
    //TODO: commenting everything
    //TODO: blue side
    //TODO: use range sensor


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
    //EncoderState strafeState;
    // Senses rings.
    TensorFlowState tfodState;

    RunToTargetZoneStateColor targetZoneState;

    CRServoState releaseWobbleGoal;

    EncoderState strafeState;

    // Stops the robot at the white tape in order to park.
    ColorSenseStopState park;

    /*
    TimerTestState zeba;



    RunToTargetZoneStateColor tzone;

    EncoderState moveState;
    TensorFlowState testState;
     */


    public void init() {

        /*
        ---HARDWARE MAP---
        erin and avi are big dummyheads >:)
         */
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        center = hardwareMap.dcMotor.get("center");

        tapeSensor =  hardwareMap.get(ColorSensor.class, "color sensor");

        wobble = hardwareMap.crservo.get("claw servo");

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
        //Our robot is as big as a field tile, so we don't really need to move, especially with how our phone is placed.

        //TODO: measure field for this, test camera angle of phone.

        tfodState = new TensorFlowState(hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

        targetZoneState = new RunToTargetZoneStateColor(motors, tapeSensor, "red");
        //TODO: make sure robot is actually in target zone when tapeSensor senses red
        //TODO: make sure strafing a reasonable amount in the beginning of the code

        releaseWobbleGoal = new CRServoState(wobble,1.0,10);

        strafeState = new EncoderState(motors, 10, .5, "left");

        park = new ColorSenseStopState(motors, tapeSensor, "yellow", .5, "backward");
        //TODO: the direc in this move method might be wrong, check the state if it moves forward

        /* states we tested lol
        zeba = new TimerTestState();
        moveForwardState = new EncoderState(motors, 2, 1.0, ""); //change calculations

        tfodState = new TensorFlowState(hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        targetZoneState = new RunToTargetZoneState(motors, tapeSensor, distSensor, "red");
        releaseWobbleGoal = new CRServoState(wobble, 2, 100);
        tzone = new RunToTargetZoneStateColor(motors, tapeSensor, "red");
        park = new ColorSenseStopState(motors, tapeSensor, "white", .5, "backward");

         */

        /*`
        ---ORDERING STATES---
         */
        //strafeState.setNextState(tfodState);
        tfodState.setNextState(targetZoneState);
        targetZoneState.setNextState(releaseWobbleGoal);
        releaseWobbleGoal.setNextState(strafeState);
        strafeState.setNextState(park);
        park.setNextState(null);

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

        machine = new StateMachine(tfodState);

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