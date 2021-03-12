package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

//The same as RunToTargetZoneState but doesn't use an MR Range (distance) sensor
public class RunToTargetZoneStateColor implements State{

    State nextState;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor center;
    ColorSensor colorSensor;
    String side;

    int cntr = 0;
    //ModernRoboticsI2cRangeSensor distSensor;

    //int targetZone = 0;

    int power = 1; //might need to change, but if change, must be positive value

   TensorFlowState t = new TensorFlowState(1);

    public RunToTargetZoneStateColor(ArrayList<DcMotor> motor, ColorSensor cs, String s) {
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        center = motor.get(4);

        colorSensor = cs;
        //distSensor = ds;

        side = s;



    }

    public void setNextState(State state) {
        nextState  = state;

    }

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

    public void runToA() {
        if(side.equals("red")) {
            center.setPower(-1);
            wait(500);
            move("stop");
        }
        if(side.equals("blue")) {
            center.setPower(1);
            wait(500);
            move("stop");
        }

        runToTape(1);



    }

    public void runToB() {
        if(side.equals("red")) {
            center.setPower(1); //this value is a test value
            wait(500); //this value is a test value
            move("stop");
        }

        if(side.equals("blue")) {
            center.setPower(-1); //this value is a test value
            wait(500); //this value is a test value
            move("stop");
        }

        runToTape(1);
    }

    public void runToC() {
        if(side.equals("red")) {
            center.setPower(-1);
            wait(500);
            move("stop");
        }
        if(side.equals("blue")) {
            center.setPower(1);
            wait(500);
            move("stop");
        }

        runToTape(3);
    }

    public void runToTape(int cntrTarget) { //Robot runs forward until the first colored tape it senses


        if (side.equals("red")) { //the colored tape will stop at red tape
            while (cntr < cntrTarget) {
                //these values are test for setPower. Might want to lower them?
                move("forward");
                if(colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                    cntr++;
                    //wait(500); might need lol
                    if(cntr >= cntrTarget)
                        move("stop");
                    wait(500);
                }
            }

        }
        if(side.equals("blue")) { //the robot will stop at blue tape
            while (cntr < cntrTarget) {
                //these values are test for setPower. Might want to lower them?
                move("forward");
            }
            if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                cntr++;
                //wait(500); might need lol
                if (cntr >= cntrTarget)
                    move("stop");
            }
        }
    }

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


    public void wait(int time) { //time is in milliseconds
        try {
            Thread.sleep(time);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public int getCntr() {
        return cntr;
    }

}
