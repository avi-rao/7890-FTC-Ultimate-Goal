package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
import java.util.ArrayList;
@Autonomous(name="tfod test", group="Iterative Opmode")
public class StateTest extends OpMode {


    /*
    7890 Space Lions 2019 "FULL AUTO BLUTRAY"
    author: 7890 Software
    GOALS: Move the foundation, navigate under the bridge
    DESCRIPTION: This code is used for our autonomous when we are located on the side of with the foundation tray.
     */



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


            testState = new TensorFlowState(hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

            testState.setNextState(null);

        }


        @Override
        public void start(){
            machine = new StateMachine(testState);

        }



        public void loop()  {

            machine.update();
            telemetry.addData("Number of Rings", testState.getTargetZone());
            telemetry.update();
        }

        public void wait(int time) {
            try {
                Thread.sleep(time * 1000);//milliseconds
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

}
