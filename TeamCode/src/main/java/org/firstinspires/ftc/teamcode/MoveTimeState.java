package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

public class MoveTimeState implements State{

    String direc;
    int time;
    double power;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor center;

    TensorFlowState t = new TensorFlowState(1);

    State nextState;

    MoveTimeState(ArrayList<DcMotor> motor, String d, int t, double p) {
        direc = d;
        time = t;
        power = p;

        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        center = motor.get(4);
    }

    public void setNextState(State state) {
        nextState  = state;
    }

    public void start(){

    }

    public State update(){
        if(t.targetZone == 1)
        {

        }
        else {
            move(direc);
            wait(time);
            move("stop");
        }
        return nextState;
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

    public void wait(int time) {
        try {
            Thread.sleep(time);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }



}
