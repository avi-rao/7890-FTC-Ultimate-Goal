package org.firstinspires.ftc.teamcode;

//trying to test stuff without the wobble goal


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.Servo;

/*
7890 Space Lions 2021r "auton target red"
author: 7890 Software
GOALS: move to the target zone, park
DESCRIPTION: This code moves us to the target zone using only a color sensor and then parks. We don't use the wobble goal mech in this code
 */
@Autonomous(name="servo test", group="Iterative Opmode")
public class ServoTest extends OpMode {
    CRServo wobble;

    DcMotor wobbleMotor;

    CRServoState releaseWobbleGoal;

    private StateMachine machine;

    @Override
    public void init() {

        wobble = hardwareMap.crservo.get("claw servo");

        releaseWobbleGoal = new CRServoState(wobble, 1.0, 500);

        releaseWobbleGoal.setNextState(null);
        //wobbleMotor = hardwareMap.dcMotor.get("wobble motor");

    }

    @Override
    public void start() {
        machine = new StateMachine(releaseWobbleGoal);


    }

    public void loop() {

        machine.update();

        telemetry.addLine("we in loop bois");
        telemetry.update();

    }

}
