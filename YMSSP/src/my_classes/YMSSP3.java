package my_classes;

import de.sciss.net.OSCListener;
import de.sciss.net.OSCMessage;
import net.beadsproject.beads.core.Bead;
import net.beadsproject.beads.core.UGen;
import net.beadsproject.beads.data.SampleManager;
import net.beadsproject.beads.ugens.*;
import net.happybrackets.core.HBAction;
import net.happybrackets.core.control.ControlScope;
import net.happybrackets.core.control.FloatBuddyControl;
import net.happybrackets.device.HB;
import net.happybrackets.device.sensors.GyroscopeListener;
import org.apache.commons.math3.transform.DftNormalization;
import org.apache.commons.math3.transform.FastFourierTransformer;
import org.apache.commons.math3.transform.TransformType;

import java.lang.invoke.MethodHandles;
import java.net.SocketAddress;
import java.util.ArrayList;
import java.util.Arrays;


public class YMSSP3 implements HBAction {

    enum Mode {
        DISJOINT,SOLO,BASELINE,UNITY
    }

    Mode mode;

    final int GYRO_HISTORY_LEN = 100;           //100 x 20ms = 2s possible interval range
    final int INTERVAL_HISTORY_LEN = 10;
    final int PERIOD_HISTORY_LEN = 10;
    final int STEPS_BETWEEN_UPDATE = 5;
    final int PADDING = 1;
    final float PMIN = 100;
    final float PMAX = 5000;
    final float DEVIATION_THRESH = 1.5f;

    float gyroMag;          //abs mag of gyro movements
    double[] gyroHistory;    //ring buffer storing history of gyro
    int gyroHistoryWritePos;
    float[] intervalHistory;
    int intervalHistoryWritePos;
    double[] cosineWindow;
    double[][] fftPeriod;

    float period;

    float[] periodHistory;
    int periodHistoryWritePos;

    long count;
    long updateIntervalMS;
    float sampleFreq;
    long previousTimeMS;

    int errorCount = 0;

    float intensity, periodStrength, deviation, theOtherPeriod = -1, theOtherDeviation, integratedPeriod;
    FloatBuddyControl intensityControl, periodControl, periodStrengthControl, deviationControl;

    //audio stuff
    Envelope level, rate;
    Glide bfFreq;
    GranularSamplePlayer gsp;
    String regularBell = "data/audio/Bells_004.46.wav";
    String irregularBell = "data/audio/Bells_009.114.wav";
    float irregLen = (float)SampleManager.sample(irregularBell).getLength();
    float maxLevel = 5;

    HB hb;

    @Override
    public void action(HB hb) {
        this.hb = hb;
        hb.reset();

        //audio controls
        level = new Envelope(0f);
        rate = new Envelope(1);
        bfFreq = new Glide(10000, 500);

        //data arrays
        gyroHistory = new double[GYRO_HISTORY_LEN];
        intervalHistory = new float[INTERVAL_HISTORY_LEN];
        cosineWindow = new double[GYRO_HISTORY_LEN];
        fftPeriod = new double[2][GYRO_HISTORY_LEN];
        for(int i = 0; i < cosineWindow.length; i++) {
            cosineWindow[i] = Math.cos(Math.PI*(double)i/GYRO_HISTORY_LEN);
        }
        periodHistory = new float[PERIOD_HISTORY_LEN];


        setupAudioSystem();

        mode = Mode.UNITY;
        modeUpdated();
        setupControls();

        hb.pattern(new Bead() {
            @Override
            protected void messageReceived(Bead message) {
                if(hb.clock.isBeat()) {
                    switch (mode) {
                        case SOLO:
//                            level.clear();
//                            level.addSegment(maxLevel, 50);
                            break;
                        case UNITY:
                            level.clear();
                            level.addSegment(maxLevel, 200);
                            level.addSegment(maxLevel, 100);
                            level.addSegment(0.1f, 200);
                            break;
                        case BASELINE:
                            level.clear();
                            level.addSegment(maxLevel, 200);
                            level.addSegment(maxLevel, 100);
                            level.addSegment(0.1f, 200);
                            break;
                        case DISJOINT:
//                            level.clear();
//                            level.addSegment(maxLevel, 50);
                            break;
                    }
                }
            }
        });

        hb.addBroadcastListener(new OSCListener() {
            @Override
            public void messageReceived(OSCMessage oscMessage, SocketAddress socketAddress, long l) {
                //period strength
                if(oscMessage.getName().equals("D_"+ hb.myIndex())) {
                    //ignore, this is self
                } else if(oscMessage.getName().startsWith("D_")) {
                    //this must be the other device
                    theOtherDeviation = (float)oscMessage.getArg(0);
                   checkMode();
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
            public void sensorUpdated(float x, float y, float z) {

                try {
                    //extract overall mag and put into history
                    gyroMag = (float) Math.sqrt(x * x + y * y + z * z);

                    gyroHistory[gyroHistoryWritePos] = gyroMag;
                    gyroHistoryWritePos = (gyroHistoryWritePos + 1) % GYRO_HISTORY_LEN;

                    //recalculate new features
                    if (count % STEPS_BETWEEN_UPDATE == 0) {
                        //get autocorrelation value
                    float[] autocorellationFeatures = findPeakPeriodXCross();
                    float tempPeriod = autocorellationFeatures[0];

                    if(tempPeriod > 0 && tempPeriod < 10000) {
                        period += (tempPeriod - period) * 0.5f;
                        //convert period history to -1:1 range before storing
                        float normalisedPeriod = (2 * (tempPeriod - PMIN) / (PMAX - PMIN)) - 1;
                        normalisedPeriod = (float)Math.tanh(normalisedPeriod);
                        periodHistory[periodHistoryWritePos] = normalisedPeriod;
                        periodHistoryWritePos = (periodHistoryWritePos + 1) % PERIOD_HISTORY_LEN;
                        if(theOtherPeriod > 0 && mode == Mode.UNITY) {
                            integratedPeriod = (period + theOtherPeriod) * 0.5f;
                        } else {
                            integratedPeriod = period;
                        }
                    }
                    UGen clockInterval = hb.clock.getIntervalUGen();
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
                    deviation = periodDeviation;
                    checkMode();
                    //set the global controls, if we're using them
                    if(intensityControl != null) {
                        intensityControl.setValue(intensity);
                        periodControl.setValue(period);
                        periodStrengthControl.setValue(periodStrength);
                        deviationControl.setValue(deviation);
                    }
                    //keep time
                    long now = System.currentTimeMillis();
                    float updateIntervalMSTmp = (now - previousTimeMS) / STEPS_BETWEEN_UPDATE;
                    previousTimeMS = now;
                    intervalHistory[intervalHistoryWritePos] = updateIntervalMSTmp;
                    intervalHistoryWritePos = (intervalHistoryWritePos + 1) % INTERVAL_HISTORY_LEN;
                    //compute new average
                    updateIntervalMS = 0;
                    for(int i = 0; i < INTERVAL_HISTORY_LEN; i++) {
                        updateIntervalMS += intervalHistory[i];
                    }
                    updateIntervalMS /= INTERVAL_HISTORY_LEN;
                    sampleFreq = 1000f / updateIntervalMS;
                    //send values
                    hb.broadcast("D_"+hb.myIndex(), deviation);
                    hb.broadcast("P_"+hb.myIndex(), period);

                        statusReport();
                    }


                    //keep time
                    count++;
                } catch(Exception e) {
                    hb.setStatus("Exception: " + e.getMessage());
                    e.printStackTrace();
                }
            }
        };

        //Direct controls
        new GyroscopeListener(hb) {
            @Override
            public void sensorUpdated(float x, float y, float z) {
                float intensity = (float)Math.sqrt(x * x + y * y + z * z);
                //now do some direct manipulation
                if(mode == Mode.DISJOINT || mode == Mode.SOLO) {
                    level.clear();
                    level.addSegment(y * maxLevel, 50);
                    gsp.getGrainIntervalUGen().setValue(Math.abs(x) * Math.abs(x) * 50 + 30);
                    gsp.getRandomnessUGen().setValue(Math.abs(y) * Math.abs(y) * 0.001f);
                    gsp.getRateUGen().setValue(Math.abs(z) * Math.abs(z) * 0.01f);
                }
                else if(mode == Mode.BASELINE) {
//                    gsp.getPitchUGen().setValue(x * 0.1f);
                    gsp.getGrainIntervalUGen().setValue(Math.abs(x) * 120 + 30);
                }
                else if(mode == Mode.UNITY) {
                    bfFreq.setValue(Math.min(Math.abs(x), 1) * 5000 + 200);
                }

            }
        };
    }

    private void checkMode() {
        Mode newMode = null;
        if(theOtherDeviation >= DEVIATION_THRESH  && deviation >= DEVIATION_THRESH ) {
            newMode = Mode.DISJOINT;
        } else if(theOtherDeviation >= DEVIATION_THRESH  && deviation < DEVIATION_THRESH) {
            newMode = Mode.BASELINE;
        } else if(theOtherDeviation < DEVIATION_THRESH && deviation >= DEVIATION_THRESH ) {
            newMode = Mode.SOLO;
        } else if(theOtherDeviation < DEVIATION_THRESH && deviation < DEVIATION_THRESH) {
            newMode = Mode.UNITY;
        }
        if(newMode != mode) {
//            modeUpdated();
//            mode = newMode;
        }
    }

    private void statusReport() {
        hb.setStatus("Mode="+mode.toString() + ": period=" + period + ": otherPeriod=" + theOtherPeriod);
    }

    private void modeUpdated() {
        switch(mode) {
            case SOLO:
                gsp.setSample(SampleManager.sample(irregularBell));
                gsp.getPitchUGen().setValue(1f);
                gsp.getGrainIntervalUGen().setValue(30f);
                gsp.getRateUGen().setValue(0.01f);
                gsp.getLoopStartUGen().setValue(0);
                gsp.getLoopEndUGen().setValue(irregLen);
                gsp.setPosition(irregLen/2);
                bfFreq.setValue(10000);
                 break;
            case UNITY:
                gsp.setSample(SampleManager.sample(regularBell));
                gsp.getPitchUGen().setValue(0.5f);
                gsp.getGrainIntervalUGen().setValue(80f);
                gsp.getGrainSizeUGen().setValue(200f);
                gsp.getRandomnessUGen().setValue(0.001f);
                gsp.getRateUGen().setValue(0.1f);
                gsp.getLoopStartUGen().setValue(50);
                gsp.getLoopEndUGen().setValue(200);
                bfFreq.setValue(10000);
                break;
            case BASELINE:
                gsp.setSample(SampleManager.sample(regularBell));
                gsp.getPitchUGen().setValue(0.5f);
                gsp.getGrainIntervalUGen().setValue(20f);
                gsp.getGrainSizeUGen().setValue(70f);
                gsp.getRandomnessUGen().setValue(0);
                gsp.getRateUGen().setValue(2);
                gsp.getLoopStartUGen().setValue(50);
                gsp.getLoopEndUGen().setValue(70);
                bfFreq.setValue(10000);
                break;
            case DISJOINT:
                gsp.setSample(SampleManager.sample(irregularBell));
                gsp.getPitchUGen().setValue(2f);
                gsp.getGrainIntervalUGen().setValue(30f);
                gsp.getGrainSizeUGen().setValue(70f);
                gsp.getRandomnessUGen().setValue(0);
                gsp.getRateUGen().setValue(1f);
                gsp.getLoopStartUGen().setValue(0);
                gsp.getLoopEndUGen().setValue(irregLen);
                gsp.setPosition(irregLen/2);
                bfFreq.setValue(10000);
                break;
        }
    }

    private void setupAudioSystem() {
        //audio system
        gsp = new GranularSamplePlayer(SampleManager.sample(irregularBell));
        Gain g = new Gain(1, level);
        g.addInput(gsp);
        gsp.setLoopType(SamplePlayer.LoopType.LOOP_ALTERNATING);
        gsp.setPitch(new Glide(1, 200));
        gsp.getPitchUGen().setValue(1);
        gsp.getRateUGen().setValue(2f);
        gsp.getLoopStartUGen().setValue(0);
        gsp.getLoopEndUGen().setValue(irregLen);
        gsp.setPosition(irregLen/2);
        BiquadFilter bf = new BiquadFilter(1, BiquadFilter.Type.LP);
        bf.setFrequency(bfFreq);
        bf.setQ(1f);
        bf.setGain(1f);
        bf.addInput(g);
        hb.ac.out.addInput(bf);
    }

    private void setupControls() {
        //shared variables
        intensityControl = new FloatBuddyControl(this, "intensity"+hb.myIndex(), 0, 0, 5) {
            @Override
            public void valueChanged(double control_val) {
            }
        };
        intensityControl.setControlScope(ControlScope.GLOBAL);
        periodControl = new FloatBuddyControl(this, "period"+hb.myIndex(), 0, 0, 20000) {
            @Override
            public void valueChanged(double control_val) {
            }
        };
        periodControl.setControlScope(ControlScope.GLOBAL);
        periodStrengthControl = new FloatBuddyControl(this, "pstrength"+hb.myIndex(), 0, 0, 30) {
            @Override
            public void valueChanged(double control_val) {
            }
        };
        periodStrengthControl.setControlScope(ControlScope.GLOBAL);
        deviationControl = new FloatBuddyControl(this, "deviation"+hb.myIndex(), 0, 0, 300) {
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

    private float[] findPeakPeriodXCross() {
        float theperiod = 0;
        float theconfidence = 0;
        //get average
        float average = 0;
        for(int i = 0; i < GYRO_HISTORY_LEN; i++) {
            double val = gyroHistory[(gyroHistoryWritePos - 1 - i + GYRO_HISTORY_LEN) % GYRO_HISTORY_LEN];
            average += val;

        }
        average /= GYRO_HISTORY_LEN;
        //determine all the zerocross times
        ArrayList<Integer> crosstimes = new ArrayList<>();      //TODO inefficient
        boolean up = (gyroHistory[(gyroHistoryWritePos - 1 + GYRO_HISTORY_LEN) % GYRO_HISTORY_LEN] - average) > 0;
        boolean firstUp = true;
        int lastUptime = 0;
        for(int i = 1; i < GYRO_HISTORY_LEN; i++) {
            boolean newUp = (gyroHistory[(gyroHistoryWritePos - i + GYRO_HISTORY_LEN) % GYRO_HISTORY_LEN] - average) > 0;
            if(newUp && !up) {
                //it's a zero cross
                if(firstUp) {
                    firstUp = false;
                } else {
                    int time = i - lastUptime;
                    crosstimes.add(time);
                }
                lastUptime = i;
            }
            up = newUp;
        }
        //determine the average cross time, ignoring the outliers
        Integer[] sortedCrosstimes = crosstimes.toArray(new Integer[0]);        //TODO inefficient
        Arrays.sort(sortedCrosstimes);
        float averageCrosstime = 0;
        if(sortedCrosstimes.length > 6) {
            for (int i = 3; i < sortedCrosstimes.length - 3; i++) {
                averageCrosstime += sortedCrosstimes[i];
            }
            averageCrosstime /= (sortedCrosstimes.length - 6);
        } else {
            for (int i = 0; i < sortedCrosstimes.length; i++) {
                averageCrosstime += sortedCrosstimes[i];
            }
            averageCrosstime /= (sortedCrosstimes.length);
        }
        theperiod = averageCrosstime * updateIntervalMS;
        return new float[] {theperiod, theconfidence};
    }

    private float[] findPeakPeriodFFT() {
        for(int i = 0; i < GYRO_HISTORY_LEN; i++) {
            fftPeriod[0][i] = gyroHistory[(gyroHistoryWritePos -1 - i + GYRO_HISTORY_LEN) % GYRO_HISTORY_LEN] * cosineWindow[i];
            fftPeriod[1][i] = 0;
        }
        FastFourierTransformer fft = new FastFourierTransformer(DftNormalization.STANDARD);
        fft.transformInPlace(fftPeriod, DftNormalization.STANDARD, TransformType.FORWARD);
        //transform done, now find peaks
        float peak = -Float.MAX_VALUE;
        int bestIndex = -1;
        float average = 0;
        for(int i = 0; i < fftPeriod[0].length/2; i++) {
            float val = (float) fftPeriod[0][i];
            average += val;
        }
        average /= fftPeriod[0].length / 2f;
        for(int i = PADDING; i < fftPeriod[0].length/2 - PADDING; i++) {
            float val = (float) fftPeriod[0][i];
            if(peak < val) {
                peak = val;
                bestIndex = i;
            }
        }
        float freq = bestIndex * sampleFreq / (fftPeriod[0].length/2f); // f_bin = i*f_s/N
        float theperiod = 1000 / freq;
        return new float[] {theperiod, (peak - average)};
    }

    public static void main(String[] args) {

        try {
            HB.runDebug(MethodHandles.lookup().lookupClass());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
