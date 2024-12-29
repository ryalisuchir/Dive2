package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
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
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-3, -46, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-26.73, -67.82, Math.toRadians(179)), Math.toRadians(179))
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