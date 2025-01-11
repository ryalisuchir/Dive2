package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 80, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-18, 66.43, Math.toRadians(-90)))
                .splineTo(
                        new Vector2d(-35, 15), Math.toRadians(270.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .setReversed(true)
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(-40, 30), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-40, 53), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                //pushing second one:
                .splineToConstantHeading(
                        new Vector2d(-40, 20), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(-50, 30), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-50, 53), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                //pushing third one:
                .splineToConstantHeading(
                        new Vector2d(-50, 20), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(-60, 30), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-60, 53), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-60, 85)
                )

                //first dropoff
                .splineToLinearHeading(new Pose2d(-7, 37, Math.toRadians(-90)), Math.toRadians(90))
                .build());

        Image fieldBackground = null;
        try {
            fieldBackground = ImageIO.read(new File("/Users/rrrr/Juice-INTO-THE-DEEP-Black.png"));
        } catch (IOException ignored) {
        }

        meepMeep.setBackground(fieldBackground)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}