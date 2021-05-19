package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.StateMachine.State;

/*
7890 Space Lions 2019 "color sense stop state"
author: 7890 Software
DESCRIPTION: robot moves in a direction until the color sensor senses the appropriate color
 */
public class ColorSenseStopState implements State {
    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor center;
    ArrayList<DcMotor> moto = new ArrayList<DcMotor>();

    //Color we're trying to sense for
    String cval;
    //Color sensor we use to sense colors
    ColorSensor cs1;
    //Direction we move in until we sense the color cval
    String dir;
    //Power that our motors move at
    double power;

    ModernRoboticsI2cRangeSensor mr;

    State NextState;


    public ColorSenseStopState(ArrayList<DcMotor> motor, ColorSensor colorSensor, String color, double p, String direction){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        center = motor.get(4);

        moto = motor;
        cs1 = colorSensor;
        cval = color;
        power = p;
        dir = direction;
    }

    public void setNextState(State state) {
        NextState = state;
    }

    public State update(){

        if(cval.equals("red")){

            move(dir);

            if(/*cs1.red()> 1000 && */cs1.red()>=cs1.blue() && cs1.red()>=cs1.green()){
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return NextState;
            }
            return this;
        }
        else if(cval.equals("blue")){

            move(dir);

            if(/*cs1.blue()> 1000 &&*/ cs1.blue()>=cs1.red() && cs1.blue()>=cs1.green()){
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return NextState;
            }
            return this;
        }
        else if (cval.equals("white")) {

            move(dir);

            if(cs1.blue() > 500 && cs1.green() > 500 && cs1.red() > 500) { //might need to change value depending on the values we get when testing
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return NextState;
            }
            return this;
        }
        else if (cval.equals("yellow")) {
            RunToTargetZoneState r = new RunToTargetZoneState(moto, cs1, mr, "red");
            if(r.a) {
                move("left");
                wait(500);
                move("forward");
            }
            else {
                move(dir);
            }

            if(cs1.green() > cs1.blue() && cs1.red() > cs1.blue() && cs1.red() >= 100) { //might need to change value depending on the values we get when testing
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return NextState;
            }
            return this;
        }
        return this;
    }

    //Basic movement method
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

    public void wait(int time) {
        try {
            Thread.sleep(time);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void start() {

    }
}