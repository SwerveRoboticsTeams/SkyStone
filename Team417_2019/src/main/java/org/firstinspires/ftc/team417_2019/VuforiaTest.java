/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team417_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name="Vuforia Movement", group ="Concept")
public class VuforiaTest extends MasterAutonomous {


    @Override public void runOpMode() throws InterruptedException {

        // intializes Vuforia components
       InitializeDetection();
       // initializes robot components
       initializeHardware();

       List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
       allTrackables.addAll(targetsSkyStone);

        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        cameraYRotate = -90;

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is 0 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 4.5f * Constants.mmPerInch;   // eg: Camera is 4.5 Inches above ground
        final float CAMERA_HORIZONTAL_DISPLACEMENT  = 9f * Constants.mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_HORIZONTAL_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, cameraYRotate, cameraZRotate, cameraXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        targetsSkyStone.activate();
        //double refAngle = imu.getAngularOrientation().firstAngle;

        //sleep(1000);

        while(targetVisible == false){
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    telemetry.update();
                    targetVisible = true;
                    targetInView = trackable;
                    vuMark = trackable.getFtcFieldFromTarget();
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    else {
                        telemetry.addData("Null",0);
                    }
                    telemetry.update();
                    //break;
                }
            }
        }

        if (targetVisible)
        {
            // express position (translation) of robot in inches.

            //telemetry.addData("VuMark",vumark.get)
            // express the rotation of the robot in degrees.
            angles = Orientation.getOrientation(lastLocation,EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            translation = lastLocation.getTranslation();

            telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) , translation.get(1) , translation.get(2));
            //rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            // double angletoTarget = Math.atan2(translation.get(1) ,translation.get(0));

            //pivotWithReference(rotation.thirdAngle,refAngle,0.5,0.7);



        }
        else {
            telemetry.addData("Visible Target", "none");
        }

        telemetry.update();

        waitForStart();

        runtime.startTime();

        double x = translation.get(0);
        double y = translation.get(1);
        double z = translation.get(2);


        // x is inverted
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                x, y, z);
        telemetry.update();
        moveMaintainHeading(x,y,angles.thirdAngle, 0.2,0.8,5.0);
        targetsSkyStone.deactivate();
    }
}
