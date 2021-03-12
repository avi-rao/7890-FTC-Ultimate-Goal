package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//MOTORS + SERVOS
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

//SENSORS
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by wenhuizhao on 11/24/18.
 */


//tests the color sensor. displays color sensor RGB values
@Autonomous(name="Color Test", group="LinearOpMode")
public class ColorTest extends LinearOpMode{
    private ElapsedTime     runtime = new ElapsedTime();

    ColorSensor sensorColor;


    public void runOpMode() {
        sensorColor = hardwareMap.get(ColorSensor.class, "color sensor");


        waitForStart();

        while(opModeIsActive()) {
//            if (depotSensor.red() > depotSensor.blue() && depotSensor.red() > depotSensor.green()) {
//                telemetry.addData("color: ", "red");
//                telemetry.update();
//            }
//            else if (depotSensor.blue() > depotSensor.red() && depotSensor.blue() > depotSensor.green()) {
//                telemetry.addData("color: ", "blue");
//                telemetry.update();
//            }
//            else {
//                telemetry.addData("color: ", "nothing");
//                telemetry.update();
//            }
//            telemetry.addData("blueL ", stoneSensorL.blue());
//            telemetry.addData("redL ", stoneSensorL.red());
//            telemetry.addData("greenL ", stoneSensorL.green());
//            telemetry.addData("blueR ", stoneSensorR.blue());
//            telemetry.addData("redR ", stoneSensorR.red());
//            telemetry.addData("greenR ", stoneSensorR.green());
            telemetry.addData("blue ", sensorColor.blue());
            telemetry.addData("red ", sensorColor.red());
            telemetry.addData("green ", sensorColor.green());
            telemetry.update();
        }

    }

}