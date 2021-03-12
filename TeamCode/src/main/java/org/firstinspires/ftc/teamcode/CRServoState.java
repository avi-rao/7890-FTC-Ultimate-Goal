package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.StateMachine.State;

public class CRServoState implements State{

    State nextState;

    CRServo servo;

    double power;

    int time;

    public CRServoState(CRServo s, double p, int t) {
        servo = s;
        power = p;
        time = t;
    }

    public void setNextState(State state) {
        nextState  = state;

    }

    //.setPosition(double);
    public void start(){

    }

    public State update() {
        servo.setPower(power);
        wait(time); //we can wait

        return nextState;
    }

    public void wait(int time) { //time is in milliseconds
        try {
            Thread.sleep(time);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
