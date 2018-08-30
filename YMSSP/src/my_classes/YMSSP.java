package my_classes;

import de.sciss.net.OSCListener;
import de.sciss.net.OSCMessage;
import net.beadsproject.beads.core.Bead;
import net.beadsproject.beads.core.UGen;
import net.beadsproject.beads.data.Buffer;
import net.beadsproject.beads.ugens.*;
import net.happybrackets.core.HBAction;
import net.happybrackets.core.control.ControlScope;
import net.happybrackets.core.control.FloatBuddyControl;
import net.happybrackets.device.HB;
import net.happybrackets.device.sensors.GyroscopeListener;
import org.apache.commons.math3.transform.DftNormalization;
import org.apache.commons.math3.transform.FastFourierTransformer;
import org.apache.commons.math3.transform.TransformType;

import java.net.SocketAddress;

public class YMSSP implements HBAction {

    enum Mode {
        DISJOINT,SOLO,BASELINE,UNITY
    }

    Mode mode;

    final int GYRO_HISTORY_LEN = 512;
    final int PERIOD_HISTORY_LEN = 100;
    final int STEPS_BETWEEN_UPDATE = 10;
    final int PADDING = 1;

    float gyroMag;          //abs mag of gyro movements
    double[] gyroHistory;    //ring buffer storing history of gyro
    int gyroHistoryWritePos;
    double[] cosineWindow;
    double[][] fftPeriod;

    float period;

    float[] periodHistory;
    int periodHistoryWritePos;

    Envelope beepLevel;
    Envelope freq;

    long count;
    long updateIntervalMS;
    float sampleFreq;
    long previousTimeMS;

    int errorCount = 0;

    float intensity, periodStrength, deviation, theOtherPeriod = -1, theOtherStability, integratedPeriod;
    FloatBuddyControl intensityControl, periodControl, periodStrengthControl, deviationControl;

    @Override
    public void action(HB hb) {
        hb.reset();
        //data arrays
        gyroHistory = new double[GYRO_HISTORY_LEN];
        cosineWindow = new double[GYRO_HISTORY_LEN];
        fftPeriod = new double[2][GYRO_HISTORY_LEN];
        for(int i = 0; i < cosineWindow.length; i++) {
            cosineWindow[i] = Math.cos(Math.PI*(double)i/GYRO_HISTORY_LEN);
        }
        periodHistory = new float[PERIOD_HISTORY_LEN];
        mode = Mode.DISJOINT;

        //audio stuff
        beepLevel = new Envelope(0);
        Gain beepGain = new Gain(1, beepLevel);
        freq = new Envelope(500);
        WavePlayer wp = new WavePlayer(freq, Buffer.SINE);
        beepGain.addInput(wp);
        hb.ac.out.addInput(beepGain);

        setupControls();

        hb.pattern(new Bead() {
            @Override
            protected void messageReceived(Bead message) {
                if(hb.clock.isBeat()) {
                    if(hb.clock.getBeatCount() % 2 == 0) {
//                        freq.clear();
//                        freq.addSegment(3750, 20);
                        beepLevel.clear();
                        beepLevel.addSegment(0.2f, 50);
                        beepLevel.addSegment(0, 50);
                    } else {
                        freq.clear();
                        freq.addSegment(500, 20);
                        beepLevel.clear();
                        beepLevel.addSegment(0.2f, 50);
                        beepLevel.addSegment(0, 50);
                    }
                }
            }
        });

        hb.addBroadcastListener(new OSCListener() {
            @Override
            public void messageReceived(OSCMessage oscMessage, SocketAddress socketAddress, long l) {
                //period strength
                if(oscMessage.getName().equals("S_"+ hb.myIndex())) {
                    //ignore, this is self
                } else if(oscMessage.getName().startsWith("S_")) {
                    //this must be the other device
                    Mode newMode = null;
                    theOtherStability = (float)oscMessage.getArg(0);
                           if(theOtherStability < 0  && deviation < 0 ) {
                               newMode = Mode.DISJOINT;
                    } else if(theOtherStability < 0  && deviation >= 0) {
                               newMode = Mode.BASELINE;
                    } else if(theOtherStability >= 0 && deviation < 0 ) {
                               newMode = Mode.SOLO;
                    } else if(theOtherStability >= 0 && deviation >= 0) {
                               newMode = Mode.UNITY;
                    }
                    if(newMode != mode) {
                        modeUpdated();
                        mode = newMode;
                        hb.setStatus(mode.toString());
                    }
                }
                //period
                else if(oscMessage.getName().equals("P_"+ hb.myIndex())) {
                    //ignore, this is self
                } else if(oscMessage.getName().startsWith("P_")) {
                    //this must be the other device
                    theOtherPeriod = (float)oscMessage.getArg(0);
                    integratedPeriod = (theOtherPeriod + period) / 2;
                }
            }
        });

        //gyro
        new GyroscopeListener(hb) {
            @Override
            public void sensorUpdated(float pitch, float roll, float yaw) {
                //extract overall mag and put into history
                gyroMag = (float)Math.sqrt(pitch * pitch + roll * roll + yaw * yaw);
                gyroHistory[gyroHistoryWritePos] = gyroMag;
                gyroHistoryWritePos = (gyroHistoryWritePos + 1) % GYRO_HISTORY_LEN;
                //recalculate new features
                if(count % STEPS_BETWEEN_UPDATE == 0) {
                    //get autocorrelation value
                    float[] autocorellationFeatures = findPeakPeriod();
                    float tempPeriod = autocorellationFeatures[0];
                    if(tempPeriod > 0 && tempPeriod < 10000) {
                        periodHistory[periodHistoryWritePos] = tempPeriod;
                        periodHistoryWritePos = (periodHistoryWritePos + 1) % PERIOD_HISTORY_LEN;
                        period += (tempPeriod - period) * 0.5f;
                        if(theOtherPeriod > 0) {
                            integratedPeriod = (period + theOtherPeriod) / 2;
                        } else {
                            integratedPeriod = period;
                        }
                    }
                    UGen clockInterval =  hb.clock.getIntervalUGen();
                    if(clockInterval != null && period > 0 && period < 100000) { //looking out for bad numbers
                        clockInterval.setValue(integratedPeriod * 1f);
                    } else {
                        errorCount++;
                    }
                    //check regularity
                    float periodDeviation = calculatePeriodDeviation();
                    //set core variables
                    intensity = gyroMag;
                    periodStrength = autocorellationFeatures[1];
                    deviation = periodDeviation;          //TODO make deviation range go from < 0 to > 0
                    //set the global controls, if we're using them
                    if(intensityControl != null) {
                        intensityControl.setValue(intensity);
                        periodControl.setValue(period);
                        periodStrengthControl.setValue(periodStrength);
                        deviationControl.setValue(deviation);
                    }

                    //keep time
                    long now = System.currentTimeMillis();
                    updateIntervalMS = (now - previousTimeMS) / STEPS_BETWEEN_UPDATE;
                    sampleFreq = 1000f / updateIntervalMS;
                    previousTimeMS = now;
                }
                //keep time
                count++;
            }
        };
    }

    private void modeUpdated() {
        //TODO
    }

    private void setupControls() {
        //shared variables
        intensityControl = new FloatBuddyControl(this, "intensity", 0, 0, 5) {
            @Override
            public void valueChanged(double control_val) {
            }
        };
        intensityControl.setControlScope(ControlScope.GLOBAL);
        periodControl = new FloatBuddyControl(this, "period", 0, 0, 20000) {
            @Override
            public void valueChanged(double control_val) {
            }
        };
        periodControl.setControlScope(ControlScope.GLOBAL);
        periodStrengthControl = new FloatBuddyControl(this, "pstrength", 0, 0, 30) {
            @Override
            public void valueChanged(double control_val) {
            }
        };
        periodStrengthControl.setControlScope(ControlScope.GLOBAL);
        deviationControl = new FloatBuddyControl(this, "deviation", 0, 0, 300) {
            @Override
            public void valueChanged(double control_val) {
            }
        };
        deviationControl.setControlScope(ControlScope.GLOBAL);
    }

    private float calculatePeriodDeviation() {
        float deviation = 0;
        float average = 0;
        for(int i = 0; i < PERIOD_HISTORY_LEN; i++) {
            average += periodHistory[i];
        }
        average /= PERIOD_HISTORY_LEN;
        for(int i = 0; i < PERIOD_HISTORY_LEN; i++) {
            deviation += (periodHistory[i] - average) * (periodHistory[i] - average);
        }
        return (float)Math.sqrt(deviation);
    }

    private float[] findPeakPeriod() {
        for(int i = 0; i < GYRO_HISTORY_LEN; i++) {
            fftPeriod[0][i] = gyroHistory[(gyroHistoryWritePos + i) % GYRO_HISTORY_LEN] * cosineWindow[i];
            fftPeriod[1][i] = 0;
        }
        FastFourierTransformer fft = new FastFourierTransformer(DftNormalization.STANDARD);
        fft.transformInPlace(fftPeriod, DftNormalization.STANDARD, TransformType.FORWARD);
        float peak = -Float.MAX_VALUE;
        int bestIndex = -1;
        float average = 0;
        for(int i = 0; i < fftPeriod[0].length/2; i++) {
            float val = (float) fftPeriod[0][i];
            average += val;
        }
        average /= fftPeriod[0].length / 2;
        for(int i = PADDING; i < fftPeriod[0].length/2 - PADDING; i++) {
            float val = (float) fftPeriod[0][i];
            if(peak < val) {
                peak = val;
                bestIndex = i;
            }
        }
        float freq = bestIndex * sampleFreq / (fftPeriod[0].length/2); // f_bin = i*f_s/N
        float period = 1000 / freq;
        return new float[] {period, (peak - average)};
    }

}
