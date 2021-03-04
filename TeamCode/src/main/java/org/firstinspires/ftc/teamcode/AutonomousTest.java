package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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
@Autonomous(name="FULL AUTO BLUTRAY", group="Iterative Opmode")
public class AutonomousTest extends OpMode
{


    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

    private StateMachine machine;

    EncoderState moveState;
     TensorFlowState testState;


    public void init() {

        /*
        ---HARDWARE MAP---
         */
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        
        /*
        ---MOTOR DIRECTIONS---
         */
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        /*
        ---GROUPING---
         */
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
       

        /*
        ---USING STATES---
         */

        moveState = new EncoderState(motors, 10, 1.0);

        moveState.setNextState(null);


        /*
        The auton plan:

        1. close the claw (servo)
        2. move to position to sense rings
            - strafe to the right (or we can turn)
            - move forward (a set distance)
        3. sense rings somehow
        4. figure out which target zone we go to
        5.


        How do we know if we're in the target zone?
            - use distance sensor
            - use color sensor
            - use encoders to know how far we've travelled

        How can we differentiate each target zone?
            - use color sensor to sense how many tiles we've moved (by counting the number of times we cross a red tape)
            - target C : use the distance sensor
              target zone A and B: use color sensor. (phone is on side of the robot, so we can either strafe backwards or just move backwards)
            - use an x-rail and a color sensor on the bottom to sense each line to know which target zone we're at


        Plan A: Write color sensor state and implement it. When the distance sensors (range sensors) come in, use that to sense target zone C
        Plan B: Just use color sensors
        Plan C: We don't have any sensors so we use encoders
         */
        testState = new TensorFlowState(hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));



    }


    @Override
    public void start(){
        
        machine = new StateMachine(moveState);

    }



    public void loop()  {

        machine.update();

    }

    public void wait(int time) {
        try {
            Thread.sleep(time * 1000);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}