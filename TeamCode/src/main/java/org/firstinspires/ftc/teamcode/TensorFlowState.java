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
7890 Space Lions 2019 "color sense stop state"
author: 7890 Software
DESCRIPTION: robot senses the number of rings and stores that value in targetZone
 */
public class TensorFlowState implements State{

    public static int targetZone = 0; //using a test value for now

    State nextState;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AT8b7FH/////AAABmd8/7XcIoUBMtVMfRzYIuoYZALWWwdnKWnAlLIT+xtlUtZeKRk98LaKAY1J5HJLfne2s0zlTX6YfKQn4SPqRBq38+5GB4HPJhuGpQ3Hqf7pypMrp2A9/vsW36SQN29hq3qrH28ovtZHatXuBBkl8WjU6saCmbL+aLR856WDBlvnN3CiQyJ6lrU10v0fxuiLYIr2p5LG40jYC31kYy+SRrsHgnA0EGlcwrY/ajR0goX4eNC4hhhLVaK0k24Db1IpRvVO6drv0fxxV6fCxMqsAu8GwJCejUB5IiU+GsNeZI7/hGhu0qmi8LSBBO4ThY+6njXF5y/HkXxrWOZJYnfC0N3jNwLrfC1Bph5X+CZbSi1cI";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public int tfodID;

    private ElapsedTime runtime = new ElapsedTime();

    public TensorFlowState(int t) {
        tfodID = t;
    }

    public void setNextState(State state) {
        nextState  = state;
    }

    public void start(){
    initVuforia();
    initTfod();
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
    }

    public State update() {
        runVuphoria(2); //this is a test value

        return nextState;
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = tfodID;
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    private void runVuphoria(double timeout){
        runtime.reset();
        boolean active = true;
        while (active == true && runtime.seconds() < timeout) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() == 0 ) {

                            /*
                            targetZone = 0;
                            active = false; //TODO: let's try this
                            break;

                             */

                            // empty list.  no objects recognized.
                        } else {
                            // list is not empty.
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                // check label to see which target zone to go after.
                                if (recognition.getLabel().equals("Single")) {
                                    targetZone = 1;
                                    active = false;
                                }
                                 else if (recognition.getLabel().equals("Quad")) {
                                    targetZone = 2;
                                     active = false;
                                } else {
                                     targetZone = 0;
                                }
                                }
                            }
                        }

                }
            }
        }
        public static int getTargetZone() {
        return targetZone;
        }
}
