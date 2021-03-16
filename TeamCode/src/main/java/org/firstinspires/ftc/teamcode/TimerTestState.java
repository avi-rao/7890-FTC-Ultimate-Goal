package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;
import java.util.List;
/*
7890 Space Lions 2019 "auton target red"
author: 7890 Software
DESCRIPTION: filler state used to test if the time worked in tfodState
 */

public class TimerTestState implements State {

    State nextState;

    String success = "no";

    public TimerTestState() {

    }

    public void setNextState(State state) {
        nextState = state;
    }

    public void start(){
        success = "yes";

    }

    public State update() {
        return nextState;

    }

    public String getSuccess() {
        return success;
    }


}
