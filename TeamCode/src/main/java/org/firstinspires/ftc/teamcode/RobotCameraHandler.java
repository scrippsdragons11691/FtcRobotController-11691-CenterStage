package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.atomic.AtomicBoolean;

public class RobotCameraHandler {

    static final String TAG = "RobotCameraHandler";

    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;

    public OpenCvCamera parkingCVCamera;
    public OpenCvCamera frontCVCCamera;
    SpikeLocationDetectionPipeline spikeLocationDetectionPipeline;
    public AtomicBoolean parkingCameraIsOpen = new AtomicBoolean(false);
    public AtomicBoolean parkingCameraIsFaulty = new AtomicBoolean(false);
    public AtomicBoolean frontCameraIsOpen = new AtomicBoolean(false);
    public AtomicBoolean frontCameraIsFaulty = new AtomicBoolean(false);
    final boolean enableParkingCamera = false;
    final boolean showParkingCameraStream = true;
    final boolean enableFrontCamera = false;
    final boolean showFrontCameraStream = false;
    final boolean showOnFTCDashboard = false;



    public RobotCameraHandler(RobotHardwareMap robotHardwareMap, LinearOpMode opmode) {
        this.opMode = opmode;
        this.robotHardwareMap = robotHardwareMap;
    }

    /**
     * Async camera listener for parking camera
     */
    /*
    OpenCvCamera.AsyncCameraOpenListener parkingCameraOpenListener
            = new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
            Log.d(TAG, "Parking Camera opened.");
            Log.d(TAG, "Setting parking pipeline as colorDetectionPipeline");
            //spikeLocationDetectionPipeline = new SpikeLocationDetectionPipeline(opMode.telemetry);
            //parkingCVCamera.setPipeline(spikeLocationDetectionPipeline);
            Log.d(TAG, "Starting parking streaming");
            parkingCVCamera.startStreaming(
                    320,
                    240,
                    OpenCvCameraRotation.UPRIGHT);
            if (showOnFTCDashboard) {
                FtcDashboard.getInstance().startCameraStream(parkingCVCamera, 0);
            }
            Log.d(TAG, "Streaming parking started");
            parkingCameraIsOpen.set(true);
        }

        @Override
        public void onError(int errorCode) {
            /*
             * This will be called if the camera could not be opened
             */
    /*
            Log.e(TAG, "Parking camera failed to initialize.");
            parkingCameraIsFaulty.set(true);
        }
    };*/

    /**
     * Async camera listener for front camera
     */
    OpenCvCamera.AsyncCameraOpenListener frontCameraOpenListener
            = new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
            Log.d(TAG, "Front Camera opened.");
            Log.d(TAG, "Setting Front pipeline as junction detection");
            spikeLocationDetectionPipeline = new SpikeLocationDetectionPipeline(opMode.telemetry);
            frontCVCCamera.setPipeline(spikeLocationDetectionPipeline);
            Log.d(TAG, "Starting junction streaming");
            frontCVCCamera.startStreaming(
                    320,
                    240,
                    OpenCvCameraRotation.UPSIDE_DOWN);
            Log.d(TAG, "Streaming front started");
            frontCameraIsOpen.set(true);
        }

        @Override
        public void onError(int errorCode) {
            /*
             * This will be called if the camera could not be opened
             */
            Log.e(TAG, "Front camera failed to initialize.");
            frontCameraIsFaulty.set(true);
        }
    };

    /***
     * Camera initialization code
     */
    public void initialize() {

        //====== Create 1 viewports
        int cameraMonitorViewId = robotHardwareMap.baseHMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robotHardwareMap.baseHMap.appContext.getPackageName());

        //split for both camera
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        int parkingCameraInitAttemptCount = 1;
        int frontCameraInitAttemptCount = 1;
        parkingCameraIsFaulty.set(true);
        parkingCameraIsOpen.set(false);
        frontCameraIsFaulty.set(true);
        frontCameraIsOpen.set(false);
        while ( !parkingCameraIsOpen.get() && enableParkingCamera
                || !frontCameraIsOpen.get() && enableFrontCamera) {


            if (enableParkingCamera) {
                if(!parkingCameraIsOpen.get() && parkingCameraIsFaulty.get()) {
                    if (showParkingCameraStream) {
                        if(parkingCameraInitAttemptCount == 1) {
                            parkingCVCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardwareMap.backCamera, viewportContainerIds[0]);
                            parkingCVCamera.showFpsMeterOnViewport(false);
                        }
                    } else {
                        parkingCVCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardwareMap.backCamera);
                    }

                    Log.i(TAG, String.format("Initializing parking camera. Attempt #%d", parkingCameraInitAttemptCount));
                    parkingCameraIsFaulty.set(false);
                    parkingCameraInitAttemptCount++;
                    Log.d(TAG, "starting camera open device");
                    //parkingCVCamera.openCameraDeviceAsync(parkingCameraOpenListener);
                }
            }

            if (enableFrontCamera) {
                if(!frontCameraIsOpen.get() && frontCameraIsFaulty.get()) {
                    if (showFrontCameraStream) {
                        if(frontCameraInitAttemptCount == 1) {
                            //parkingCVCamera = OpenCvCameraFactory.getInstance().createWebcam(theHardwareMap.parkingCamera, viewportContainerIds[0]);
                            frontCVCCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardwareMap.frontCamera, viewportContainerIds[1]);
                            frontCVCCamera.showFpsMeterOnViewport(false);
                        }
                    } else {
                        frontCVCCamera = OpenCvCameraFactory.getInstance().createWebcam(robotHardwareMap.frontCamera);
                    }

                    Log.i(TAG, String.format("Initializing front camera. Attempt #%d", frontCameraInitAttemptCount));
                    frontCameraIsFaulty.set(false);
                    frontCameraInitAttemptCount++;
                    Log.d(TAG, "starting camera open device");
                    frontCVCCamera.openCameraDeviceAsync(frontCameraOpenListener);
                }
            }

            opMode.sleep(1);

            if (opMode.isStopRequested())
                break;
            if (opMode.isStarted())
                break;
        }
        Log.d(TAG, "Camera initialization done.");
    }
}