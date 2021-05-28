package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.ArrayList;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
7890 Space Lions 2019 "auton test red"
author: 7890 Software
GOALS: move wobble goal to the target zone, park
DESCRIPTION: This code moves us to the target zone using a color sensor and dist sensor and then parks
 */
@Autonomous(name="auton red", group="Iterative Opmode")
public class AutonTestRed extends OpMode
{

    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor center;


    DcMotor wobble;
    CRServo clawServo;

    /*
    ---SENSORS---
     */

    ColorSensor tapeSensor;
    ModernRoboticsI2cRangeSensor distSensor;

    /*
    The string "side" allows us to define when we're on the red or blue side of the field,
    as the code for the blue side requires the color sensor to sense blue and our robot to strafe
    (move sideways) in a different direction than when we are on the red side. In this case,
    we are coding for the red side.
     */

    String side = "red";

    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

    private StateMachine machine;

    // Here we have declared our states and ordered our states.

    /* EncoderState tells the wheels of the robot to complete a certain amount of rotations, rather
    than moving for specific time intervals. For more information, see EncoderState class.
     */

    EncoderState moveForwardState;
    /*
    * Here, we create a new instance of EncoderState called "MoveForwardState." EncoderState allows us to move forward
    * a specified amount of inches. This allows us to be very precise with how far we move and makes our autonomous very consistent.
    * For more information, please reference the EncoderState class.
    */
    TensorFlowState tfodState;
    /*
     *Here, we create a new  instance of TensorFlowState called "tfodState." This is the state in which we activate our
     * TensorFlow Object Detection system, allowing us to consistently identify the number of rings every time, even with randomization.
     * In it, we use our phone camera and compare it with a library of pictures of rings for indentification
     * Then, we assign a value for the target zone we have to go to, which we can use later in our code.
     * For more information, please reference the TensorFlowState class.
     */
    MoveTimeState strafeState;
    /**
     * Here, we create a new instance of strafeState called "strafeState." In it, we strafe a certain number of seconds to the right
     * in order to get in position to deliver the wobble goal. This state uses the target zone values from tfodState
     * to correctly determine when and how long to strafe
     * That way, we can use just this one state for all three target zones
     * For more information, please reference the MoveTimeState class
     */
    RunToTargetZoneState targetZoneState;

    /*
     * After strafing(or not strafing) the correct amount, we need to actually get to the target zone.
     * That's where RunToTargetZoneState comes in. Here we create an instance of it called targetZoneState.
     * This state uses the values that we get from TensorFlowState for the target zones to decide which target zone we need to go to
     * It then uses the color sensor or distance sensor to pinpoint and park in the target zone
     * For more information, please reference the RunToTargetZoneState class
     */

    MotorState turnWobbleGoal;

    CRServoState releaseWobbleGoal;
    /*
     *We create a new CRServoState called releaseWobbleGoal that does exactly what it sounds like; it releases the wobble goal!
     * Very simple state, since it just opens our mechanism and lets go.
     * For more information, please reference the CRServoState class
     */
    ColorSenseStopState park;
    /**
     * Finally, our last state is park, an instance of ColorSenseStopState. Essentially, park just moves us backwards
     * from any target zone until we sense yellow tape with our color sensor
     * At that point, it tells the robot to stop, letting us park on the launch line with ease
     * For more information, please reference the ColorSenseStopState class.
     */

    //EncoderState tfodTest;
    public void init() {

        /*
        ---HARDWARE MAP---
         */
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        center = hardwareMap.dcMotor.get("center");


        tapeSensor =  hardwareMap.get(ColorSensor.class, "color sensor");
        distSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dist sensor");

        wobble = hardwareMap.dcMotor.get("claw motor");
        clawServo = hardwareMap.crservo.get("claw servo");

        /*
        ---MOTOR DIRECTIONS---
         */
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        /*
        ---GROUPING---
         */
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        motors.add(center);


        /*
        ---USING STATES---
         */

        /*
        Having declared our states, we now initialize the states and set parameters. For more
        information, look at each respective state class.
         */

        moveForwardState = new EncoderState(motors, 20, .6, "forward");
        //We now specify the parameters of the aforementioned EncoderState: It uses our motors, goes 10 inches, has a speed of .3, and goes forward
        tfodState = new TensorFlowState(hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        //We specify the paramaters of our TensorFlowState here. Most of them are just logistical, such as the name of our ID and so on.
        //tfodTest = new EncoderState(motors, 10, 0.3, "forward");
        strafeState = new MoveTimeState(motors, "right", 750, 1.0);
        //Here for strafeState we set the parameters as our motors, moving right for 750 ms at a power of one.
        targetZoneState = new RunToTargetZoneState(motors, tapeSensor, distSensor, "red");
        //Here, for targetZoneState we set the parameters as our motors, color and distance sensor, and trying to sense red.
        turnWobbleGoal = new MotorState(wobble, 250, -2.5);

        releaseWobbleGoal = new CRServoState(clawServo, 1.0, 500);
        //Here, we set the parameters of the Continous Rotation Servo to power 1 for 500 ms
        park = new ColorSenseStopState(motors, tapeSensor, "yellow", .5, "backward");
        //Finally, for our park state, we use our motors at power .5 going backwards and our color sensor to sense yellow(for the tape).
        /*`
        ---ORDERING STATES---
         */

        /*
         * Down there is where the magic happens. Just like block coding(i.e Scrath, Hopscotch), we order our states
         * like blocks and have our robot run through them one by one. The code is pretty simple, but essentially when
         * one state stops running, we tell it to call the next, and so on. The last state just says "null," because we're ending on it.
         */

        moveForwardState.setNextState(tfodState);
        tfodState.setNextState(strafeState);
        strafeState.setNextState(targetZoneState);
        targetZoneState.setNextState(turnWobbleGoal);
        turnWobbleGoal.setNextState(releaseWobbleGoal);
        releaseWobbleGoal.setNextState(park);
        park.setNextState(null);

    }


    @Override
    public void start(){

        // Starts the robot.



        //clawServo.setPower(-1);


        machine = new StateMachine(moveForwardState);

    }



    public void loop()  {

        // Constantly checks for updates and makes sure our robot is always running a state.

        machine.update();

    }


    // Allows robot to wait before continuing on to the next method.

    public void wait(int time) {
        try {
            Thread.sleep(time);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
