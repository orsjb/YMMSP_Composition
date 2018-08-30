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


//        clicky settings
//        freq=30
//        modRatio=0.05
//        modLevel=1			(up to 10 or even 100)
//        bffreq =800
//
//        drone settings
//        freq=300
//        modRatio=0.50001
//        modLevel=0.8			 (crank up to 10 or even 100 goes nuts)
//        bfFreq=200-250
//
//        Space drone
//        freq=300
//        modRatio=0.050001
//        modLevel=1
//        bfFreq=200-250

public class YMSSP implements HBAction {

    enum Mode {
        DISJOINT,SOLO,BASELINE,UNITY
    }

    Mode mode;

    final int GYRO_HISTORY_LEN = 512;
    final int INTERVAL_HISTORY_LEN = 10;
    final int PERIOD_HISTORY_LEN = 100;
    final int STEPS_BETWEEN_UPDATE = 10;
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

    Envelope level, freq, modRatio, modLevel, bfFreq;

    long count;
    long updateIntervalMS;
    float sampleFreq;
    long previousTimeMS;

    int errorCount = 0;

    float intensity, periodStrength, deviation, theOtherPeriod = -1, theOtherDeviation, integratedPeriod;
    FloatBuddyControl intensityControl, periodControl, periodStrengthControl, deviationControl;

    HB hb;

    @Override
    public void action(HB hb) {
        this.hb = hb;
        hb.reset();

        //audio controls
        level = new Envelope(0f);
        freq = new Envelope(300);
        modRatio = new Envelope(0.50001f);
        modLevel = new Envelope(2f);
        bfFreq = new Envelope(250);

        //data arrays
        gyroHistory = new double[GYRO_HISTORY_LEN];
        intervalHistory = new float[INTERVAL_HISTORY_LEN];
        cosineWindow = new double[GYRO_HISTORY_LEN];
        fftPeriod = new double[2][GYRO_HISTORY_LEN];
        for(int i = 0; i < cosineWindow.length; i++) {
            cosineWindow[i] = Math.cos(Math.PI*(double)i/GYRO_HISTORY_LEN);
        }
        periodHistory = new float[PERIOD_HISTORY_LEN];
        mode = Mode.DISJOINT;
        hb.setStatus("Mode="+mode.toString());

        setupAudioSystem();
        setupControls();

        hb.pattern(new Bead() {
            @Override
            protected void messageReceived(Bead message) {
                if(hb.clock.isBeat()) {
//                    if(hb.clock.getBeatCount() % 2 == 0) {
////                        freq.clear();
////                        freq.addSegment(3750, 20);
                        level.clear();
                        level.addSegment(0.6f, 50);
                        level.addSegment(0, 50);
//                    } else {
//                        freq.clear();
////                        freq.addSegment(500, 20);
////                        level.clear();
//                        level.addSegment(0.6f, 50);
//                        level.addSegment(0, 50);
//                    }
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
                        period += (tempPeriod - period) * 0.1f;
                        //convert period history to -1:1 range before storing
                        float normalisedPeriod = (2 * (period - PMIN) / (PMAX - PMIN)) - 1;
                        normalisedPeriod = (float)Math.tanh(normalisedPeriod);
                        periodHistory[periodHistoryWritePos] = normalisedPeriod;
                        periodHistoryWritePos = (periodHistoryWritePos + 1) % PERIOD_HISTORY_LEN;
                        if(theOtherPeriod > 0) {
                            integratedPeriod = (period + theOtherPeriod) / 2;
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
                }
                //keep time
                count++;
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
            modeUpdated();
            mode = newMode;
            hb.setStatus("Mode="+mode.toString());
        }
    }

    private void modeUpdated() {
        switch(mode) {
            case SOLO:
                modRatio.setValue(4f);
                 break;
            case UNITY:
                modRatio.setValue(0.06677f);
                break;
            case BASELINE:
                modRatio.setValue(2.5f);
                break;
            case DISJOINT:
                modRatio.setValue(0.0501f);
                break;
        }
    }

    private void setupAudioSystem() {
        //audio system
        Gain beepGain = new Gain(1, level);
        WavePlayer carrier = new WavePlayer(new Mult(modRatio, freq), Buffer.SAW);
        Function mod = new Function(freq, carrier, modLevel) {
            @Override
            public float calculate() {
                return x[0] * (1 + x[1] * x[2]);
            }
        };
        WavePlayer wp = new WavePlayer(mod, Buffer.SAW);
        beepGain.addInput(wp);
        BiquadFilter bf = new BiquadFilter(hb.ac, 1, BiquadFilter.Type.LP);
        bf.setFrequency(bfFreq);
        bf.setQ(2f);
        bf.setGain(5f);
        bf.addInput(beepGain);
        hb.sound(bf);
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
        average /= fftPeriod[0].length / 2f;
        for(int i = PADDING; i < fftPeriod[0].length/2 - PADDING; i++) {
            float val = (float) fftPeriod[0][i];
            if(peak < val) {
                peak = val;
                bestIndex = i;
            }
        }
        float freq = bestIndex * sampleFreq / (fftPeriod[0].length/2f); // f_bin = i*f_s/N
        float period = 1000 / freq;
        return new float[] {period, (peak - average)};
    }

}
