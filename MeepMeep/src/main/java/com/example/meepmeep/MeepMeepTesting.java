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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-7, 34, Math.toRadians(-90)))
                .setReversed(true)
                .strafeToLinearHeading(
                        new Vector2d(-6, 42), Math.toRadians(180),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .strafeToLinearHeading(
                        new Vector2d(-31, 42), Math.toRadians(180),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .strafeToLinearHeading(
                        new Vector2d(-31, 10), Math.toRadians(90),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .setTangent(180)
                .splineToConstantHeading(
                        new Vector2d(-46, 20), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-46, 55), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-46, 10), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-55, 20), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-55, 55), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-55, 10), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-60, 20), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )
                .splineToConstantHeading(
                        new Vector2d(-60, 55), Math.toRadians(90.00),
                        null,
                        new ProfileAccelConstraint(-85, 85)
                )

                .splineToLinearHeading(new Pose2d(-28, 58, Math.toRadians(90)), Math.toRadians(90))
                .splineTo(
                        new Vector2d(-28, 62), Math.toRadians(90)
                )
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