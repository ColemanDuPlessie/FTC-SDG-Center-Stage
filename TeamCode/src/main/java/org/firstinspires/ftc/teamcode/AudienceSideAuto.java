/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SetDrivingStyle.isBlue;
import static org.firstinspires.ftc.teamcode.SetDrivingStyle.startAudienceSide;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.ArmAwareSetSlides;
import org.firstinspires.ftc.teamcode.backend.commands.DriverAssistedAutoTargetedDeposit;
import org.firstinspires.ftc.teamcode.backend.commands.EnableIntakeSafe;
import org.firstinspires.ftc.teamcode.backend.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.commands.ReadyArmCarefully;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;


/**
 * I should probably document this...
 */

@Autonomous(name="Audience Side Auto")
@Config
public class AudienceSideAuto extends CommandbasedOpmode {


    SampleMecanumDrive drive;

    TrajectorySequence startLTraj;
    TrajectorySequence startCTraj;
    TrajectorySequence startRTraj;

    TrajectorySequence intakeTraj;

    private static final double REVERSE = Math.toRadians(180);
    private static double CLOCKWISE90 = Math.toRadians(-90);

    public static double STARTX = -36;
    public static double STARTY = -63;
    public static double STARTTHETA = Math.toRadians(-90);
    public static double LRPURPLEDEPOSITX = -43.5;
    public static double LRPURPLEDEPOSITXOFFSET = 11.5; // This is correct for Red R, Bule L, and must be negated for Red L, Blue R
    public static double LRPURPLEDEPOSITY = -36;
    public static double LRPURPLEDEPOSITTHETA = REVERSE;
    public static double CPURPLEDEPOSITY = -24.5;
    public static double PIXELINTAKEX = -58;
    public static double PIXELINTAKEY = -24;

    double startHeading;

    @Override
    public void init() {
        robot.init(hardwareMap, false);

        if (isBlue) {
            CLOCKWISE90 *= -1;
            STARTY *= -1;
            STARTTHETA -= REVERSE;
            CPURPLEDEPOSITY *= -1;
            LRPURPLEDEPOSITY *= -1;
            LRPURPLEDEPOSITXOFFSET *= -1;
            LRPURPLEDEPOSITTHETA -= REVERSE;
        }

        startHeading = robot.drivetrain.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, STARTTHETA);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        startRTraj = drive.trajectorySequenceBuilder(startPose) // This is actually the left trajectory on blue side
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(STARTX-4, LRPURPLEDEPOSITY*0.3+STARTY*0.7), STARTTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(LRPURPLEDEPOSITX, LRPURPLEDEPOSITY, REVERSE), LRPURPLEDEPOSITTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(LRPURPLEDEPOSITX+LRPURPLEDEPOSITXOFFSET, LRPURPLEDEPOSITY, REVERSE), LRPURPLEDEPOSITTHETA+REVERSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.purplePixel.activate())
                .setReversed(false)
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(LRPURPLEDEPOSITX, LRPURPLEDEPOSITY), LRPURPLEDEPOSITTHETA)
                .splineToConstantHeading(new Vector2d(PIXELINTAKEX, PIXELINTAKEY), REVERSE+CLOCKWISE90/2)
                .build();

        startCTraj = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(STARTX-4, CPURPLEDEPOSITY*0.2+STARTY*0.8), STARTTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(STARTX-4, CPURPLEDEPOSITY*0.7+STARTY*0.3, REVERSE), STARTTHETA+REVERSE)
                .splineToConstantHeading(new Vector2d(STARTX-4, CPURPLEDEPOSITY), STARTTHETA+REVERSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.purplePixel.activate())
                .setReversed(false)
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(PIXELINTAKEX, PIXELINTAKEY), REVERSE)
                .build();

        startLTraj = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(STARTX-4, LRPURPLEDEPOSITY*0.3+STARTY*0.7), STARTTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(LRPURPLEDEPOSITX-LRPURPLEDEPOSITXOFFSET, LRPURPLEDEPOSITY, REVERSE), LRPURPLEDEPOSITTHETA+CLOCKWISE90/2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.purplePixel.activate())
                .waitSeconds(0.75)
                .setReversed(false)
                .lineTo(new Vector2d(PIXELINTAKEX-1.5, LRPURPLEDEPOSITY))
                .lineTo(new Vector2d(PIXELINTAKEX-1.5, PIXELINTAKEY))
                .lineTo(new Vector2d(PIXELINTAKEX, PIXELINTAKEY))
                .build();

        intakeTraj = drive.trajectorySequenceBuilder(new Pose2d(PIXELINTAKEX, PIXELINTAKEY, REVERSE))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scheduler.schedule(new EnableIntakeSafe(robot.intake, robot.arm, robot.wrist, timer));
                    robot.intake.lowerDropdown(4);
                })
                .lineTo(new Vector2d(PIXELINTAKEX-3.5, PIXELINTAKEY))
                .waitSeconds(2.0)
                .lineTo(new Vector2d(PIXELINTAKEX, PIXELINTAKEY))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.hold();
                    robot.intake.raiseDropdown();
                })
                .build();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("Prop detection", robot.camera.getPropPosition());
        telemetry.addData("Prop detection confidence", robot.camera.getPropConfidence());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.camera.propDetected();
        if (isBlue) {
            TrajectorySequence temp = startLTraj;
            startLTraj = startRTraj;
            startRTraj = temp;
        }
        ArrayList<Command> auto = new ArrayList<>();
        switch (robot.camera.getPropPosition()) {
            case LEFT:
                auto.add(new FollowRRTraj(robot.drivetrain, drive, startLTraj));
                break;
            case RIGHT:
                auto.add(new FollowRRTraj(robot.drivetrain, drive, startRTraj));
                break;
            case CENTER:
            default: // This shouldn't do anything if everything is working
                auto.add(new FollowRRTraj(robot.drivetrain, drive, startCTraj));
                break;
        }
        auto.add(new FollowRRTraj(robot.drivetrain, drive, intakeTraj));
        scheduler.schedule(false, new SequentialCommandGroup(auto.toArray(new Command[0])));
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void end() {
        AutoToTeleopContainer.getInstance().setAngleDelta(startHeading-robot.drivetrain.getHeading()+Math.toRadians(180));
    }
}