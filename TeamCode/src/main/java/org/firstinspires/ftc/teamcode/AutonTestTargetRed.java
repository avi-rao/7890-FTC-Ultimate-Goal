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
7890 Space Lions 2019 "FULL AUTO BLUTRAY"
author: 7890 Software
GOALS: Move the foundation, navigate under the bridge
DESCRIPTION: This code is used for our autonomous when we are located on the side of with the foundation tray.
 */
@Autonomous(name="auton target red", group="Iterative Opmode")
public class AutonTestTargetRed extends OpMode
{


    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor center;
    CRServo wobble;

    ColorSensor tapeSensor;
    ModernRoboticsI2cRangeSensor distSensor;

    String side = "red";


    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

    private StateMachine machine;

    EncoderState moveForwardState;
    TensorFlowState tfodState;
    RunToTargetZoneState targetZoneState;
    //CRServoState releaseWobbleGoal;
    ColorSenseStopState park;

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

        /*
        moveForwardState = new EncoderState(motors, 2, 1.0, "forward"); //change calculations
        tfodState = new TensorFlowState(hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        targetZoneState = new RunToTargetZoneState(motors, tapeSensor, distSensor, "red");
        //releaseWobbleGoal = new CRServoState(wobble, 2, 100);

         */

        tzone = new RunToTargetZoneStateColor(motors, tapeSensor, "red");
        park = new ColorSenseStopState(motors, tapeSensor, "white", .5, "backward");



        /*
        moveForwardState.setNextState(tfodState);
        tfodState.setNextState(targetZoneState);
        targetZoneState.setNextState(null);
        */

        //releaseWobbleGoal.setNextState(park);
        park.setNextState(null);
        tzone.setNextState(null);


    }


    @Override
    public void start(){

        //wobble.setPower(1); //test value

        machine = new StateMachine(tzone);

    }



    public void loop()  {

        machine.update();
        telemetry.addData("cntr value", tzone.getCntr());
        telemetry.update();

    }

    public void wait(int time) {
        try {
            Thread.sleep(time * 1000);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}