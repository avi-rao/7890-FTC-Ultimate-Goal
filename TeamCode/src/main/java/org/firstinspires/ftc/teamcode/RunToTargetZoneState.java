package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.StateMachine.State;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

/*
7890 Space Lions 2019 "color sense stop state"
author: 7890 Software
DESCRIPTION: robot uses the value of targetZone to move to the correct target zone
 */
public class RunToTargetZoneState implements State{

    State nextState;

    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor center;

    /*
    ---SENSORS---
     */
    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor distSensor;

    //side allows us to switch the code depending on what color side we're on
    String side;
    //power is the power we will set our motors to
    double power = 1;

    //Allows us to get the variable targetZone
    TensorFlowState t = new TensorFlowState(1);

    public static boolean a = false;

   /*
   This is a constructor. In this constructor, we set up parameters for our RunToTargetZoneState
   so that we can assign specific objects to it in our AutonTestRed class.
    */
   public RunToTargetZoneState(ArrayList<DcMotor> motor, ColorSensor cs, ModernRoboticsI2cRangeSensor ds, String s) {
       //Setting up the drive chain motors
       leftFront = motor.get(0);
       rightFront = motor.get(1);
       leftBack = motor.get(2);
       rightBack = motor.get(3);
       center = motor.get(4);

       //Setting up color sensors
       colorSensor = cs;
       distSensor = ds;

       //Setting up side variable
       side = s;
    }

    public void setNextState(State state) {
        nextState  = state;
    }

    /*
    This if statement allows the robot to determine which target zone to run to after scanning
    the rings. We use TensorFlow state to sense the number of rings, and depending on the number of
    rings it senses, the robot will run the method that corresponds to that number of rings.
     */
    public void start(){
        if (t.targetZone == 2) {
            runToC();
        }
        else if (t.targetZone == 1) {
            runToB();
        }
        else {
            runToA();
        }
    }

    public State update() {

        return nextState;
    }

    //This is the method run when we need to go to target zone A.
    public void runToA() {
        a = true;
        /*if(side.equals("red")) {
            center.setPower(-1);
            wait(500);
            center.setPower(0);
            runToTape();
        }
        if(side.equals("blue")) {
            center.setPower(-1);
            wait(500);
            runToTape();
        }

         */

        runToTape();

    }

    //This is the method run when we need to go to target zone B.
    public void runToB() {
        /*
        if(side.equals("red")) {
            //center.setPower(1); //this value is a test value
            wait(500); //this value is a test value
            runToTape();
        }

        if(side.equals("blue")) {
            //center.setPower(-1); //this value is a test value
            wait(500); //this value is a test value

         */
            runToTape();
    }

    //This is the method run when we need to go to target zone C. Unlike the other two, it uses a
    // distance sensor to get to the target zone.
    public void runToC() {
        /*
        if(side.equals("red")) {
            center.setPower(-1);
            wait(500);
            center.setPower(0);
        }
        if(side.equals("blue")) {
            center.setPower(1);
            wait(500);
        }
         */
        while(distSensor.getDistance(DistanceUnit.INCH) >= 20) {
            move("forward");
        }
        move("stop");
    }

    // This method is used in all the previous methods, and it allows us to sense and stop at tape.
    public void runToTape() { //Robot runs forward until the first colored tape it senses
        if (side.equals("red")) { //the colored tape will stop at red tape
            while (colorSensor.blue() > colorSensor.red() || colorSensor.red() < 70 || colorSensor.red() > 110/*colorSensor.green() > colorSensor.red()*/) {
                move("forward");
            }
            move("stop");
        }
        if(side.equals("blue")) { //the robot will stop at blue tape
            while (colorSensor.red() > colorSensor.blue() || colorSensor.green() > colorSensor.blue()) {
                move("forward");
            }
            move("stop");
        }
    }

    /*
    ---MOVE---
     */
    // This method allows our robot to make basic movements.
    public void move(String direc) {
        if(direc.equals("forward")) {
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
        }
        if(direc.equals("backward")) {
            leftFront.setPower(-power);
            rightFront.setPower(-power);
            leftBack.setPower(-power);
            rightBack.setPower(-power);
        }
        if(direc.equals("left")) {
            center.setPower(power);
        }
        if(direc.equals("right")) {
            center.setPower(-power);
        }
        if(direc.equals("stop")) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            center.setPower(0);
        }
    }

    // Allows robot to wait before continuing on to the next method.
    public void wait(int time) { //time is in milliseconds
        try {
            Thread.sleep(time);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}
