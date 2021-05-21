package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

public class MotorState implements State{

    DcMotor motor;
    int time;
    double power;

    State nextState;

    MotorState(DcMotor m, int t, double p) {
        motor = m;
        time = t;
        power = p;
    }

    public void setNextState(State state) {
        nextState  = state;
    }

    public void start(){

    }

    public State update() {
        motor.setPower(power);
        wait(500);
        motor.setPower(0);
        return nextState;
    }

    public void wait(int time) {
        try {
            Thread.sleep(time);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
