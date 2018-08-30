/*
 * Copyright 2016 Ollie Bown
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package my_classes;

import net.beadsproject.beads.data.Buffer;
import net.beadsproject.beads.ugens.*;
import net.happybrackets.core.HBAction;
import net.happybrackets.device.HB;
import net.happybrackets.device.sensors.*;

/**
 * For this example we want to look at the accelerometer and use it to trigger a sound when you turn over the
 * accelerometer.
 */
public class WooshSine implements HBAction {

    public enum Orientation {UP, DOWN}

    Orientation currentOri = Orientation.UP;
    Orientation previousOri = Orientation.DOWN;

    float gainMult = 2f;

    @Override
    public void action(HB hb) {

        hb.reset();
//        hb.testBleep();

        Glide modFreq = new Glide(hb.ac, 20);
        WavePlayer mod = new WavePlayer(hb.ac, modFreq, Buffer.SAW);
        Glide baseFreq = new Glide(hb.ac, 300);
        Function f = new Function(mod, baseFreq) {
            @Override
            public float calculate() {
                return x[1] + 15 * x[0];
            }
        };

        WavePlayer wp = new WavePlayer(hb.ac, f, Buffer.SAW);
        Glide env = new Glide(hb.ac, 0);
        Gain g = new Gain(hb.ac, 1, env);
        g.addInput(wp);


        BiquadFilter bf = new BiquadFilter(hb.ac, 1, BiquadFilter.Type.LP);
        Glide freq = new Glide(hb.ac, 800);
        bf.setFrequency(freq);
        bf.setQ(2f);
        bf.setGain(2f);
        PolyLimit pl = new PolyLimit(hb.ac, 1, 8);
        bf.addInput(pl);

        pl.addInput(g);

        BiquadFilter hpf = new BiquadFilter(hb.ac, 1, BiquadFilter.HP);
        hpf.setFrequency(100);
        hpf.setQ(1f);
        hpf.setGain(1f);
        hpf.addInput(g);

        Reverb rb = new Reverb(hb.ac);
        rb.setDamping(0.9f);
        rb.addInput(bf);

        hpf.addInput(bf);
        hpf.addInput(rb);
        hb.sound(hpf);

//        Gyroscope mySensor = (MiniMU) hb.getSensor(MiniMU.class);
//        mySensor.addListener(new SensorUpdateListener() {

        new GyroscopeListener(hb) {

            double prevMag = 0;
            double val = 0;

            @Override
            public void sensorUpdated(float pitch, float roll, float yaw) {

                double zAxis = yaw;
                double yAxis = roll;
                double xAxis = pitch;

                modFreq.setValue((float)Math.abs(zAxis*zAxis) * 3f);
                baseFreq.setValue((float)(xAxis*xAxis) * 100f + 20); //
                System.out.println("z " + zAxis);

                double mag = Math.sqrt(xAxis*xAxis + yAxis*yAxis + zAxis*zAxis);
                mag /= 20f;
                if(prevMag < mag) {
                    val += (mag - val) * 0.7f;
                } else {
                    val += (mag - val) * 0.0001f;
                }
                env.setValue((float)val*10f*gainMult);
                freq.setValue((float)val * 400);
                prevMag = mag;

            }
        };
    }
}
