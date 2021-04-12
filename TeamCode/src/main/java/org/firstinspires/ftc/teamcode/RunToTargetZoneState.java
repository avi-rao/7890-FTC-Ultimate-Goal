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

    String side;

    double power = 0.5;

   TensorFlowState t = new TensorFlowState(1);

   /* This is a constructor. In this constructor, we set up parameters for our RunToTargetZoneState
   so that we can assign specific objects to it in our AutonTestRed class.
    */

   public RunToTargetZoneState(ArrayList<DcMotor> motor, ColorSensor cs, ModernRoboticsI2cRangeSensor ds, String s) {
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        center = motor.get(4);

        colorSensor = cs;
        distSensor = ds;

        side = s;



    }

    public void setNextState(State state) {
        nextState  = state;

    }

    /* This if statement allows the robot to determine which target zone to run to after scanning
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
        if(side.equals("red")) {
            center.setPower(-1);
            wait(500);
            runToTape();
        }
        if(side.equals("blue")) {
            center.setPower(1);
            wait(500);
            runToTape();
        }

    }

    //This is the method run when we need to go to target zone B.

    public void runToB() {
        if(side.equals("red")) {
            center.setPower(1); //this value is a test value
            wait(500); //this value is a test value
            runToTape();
        }

        if(side.equals("blue")) {
            center.setPower(-1); //this value is a test value
            wait(500); //this value is a test value
            runToTape();
        }
    }

    //This is the method run when we need to go to target zone C. Unlike the other two, it uses a
    // distance sensor to get to the target zone.

    public void runToC() {
        if(side.equals("red")) {
            center.setPower(-1);
            wait(500);
        }
        if(side.equals("blue")) {
            center.setPower(1);
            wait(500);
        }

        while(distSensor.getDistance(DistanceUnit.INCH) > 5) {
            move("forward");
        }
        move("stop");
    }

    // This method is used in all the previous methods, and it allows us to sense and stop at tape.

    public void runToTape() { //Robot runs forward until the first colored tape it senses
        if (side.equals("red")) { //the colored tape will stop at red tape
            while (colorSensor.blue() > colorSensor.red() || colorSensor.green() > colorSensor.red()) {
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
