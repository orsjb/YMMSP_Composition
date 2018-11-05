package my_classes;

import net.beadsproject.beads.core.AudioContext;
import net.beadsproject.beads.core.Bead;
import net.beadsproject.beads.core.UGen;
import net.beadsproject.beads.data.SampleManager;
import net.beadsproject.beads.ugens.*;
import org.jaudiolibs.beads.AudioServerIO;

import java.io.File;
import java.io.IOException;
import java.util.Random;

public class SSTest2 {

    static final String audioPath = "/Users/ollie/Desktop/SSS_Vox";
    static final int NUM_VOICES = 400;
    static final int BUFFERSIZE = 1024;
    static final Random rng = new Random();

    static final int[] SCALE_NOTES = new int[]{0,4,7,12,16,5,11,2,3,9,1,6,8,14,10,13,15};



    public static void main(String[] args) throws IOException {

        SampleManager.group("Breath", audioPath + "/Breath");
        SampleManager.group("Hmmm", audioPath + "/Hmmm");
        SampleManager.group("Pop", audioPath + "/Pop");
        SampleManager.group("ShortAh", audioPath + "/ShortAh");
        SampleManager.group("Stab", audioPath + "/Stab");

        SampleManager.group("Shhh", audioPath + "/Shhh");

        JackManager.run(BUFFERSIZE);
        AudioContext ac = new AudioContext(new AudioServerIO.Jack(), BUFFERSIZE);
        ac.start();
        UGen.setDefaultContext(ac);
        ac.out.setGain(5f / NUM_VOICES);

        SamplePlayer[] voices = new SamplePlayer[NUM_VOICES];
        for(int i = 0; i < NUM_VOICES; i++) {
            voices[i] = new SamplePlayer(1);
            voices[i].setKillOnEnd(false);
            float rand = rng.nextFloat();
            Gain lGain = new Gain(1, rand);
            Gain rGain = new Gain(1, 1 - rand);
            lGain.addInput(voices[i]);
            rGain.addInput(voices[i]);
            ac.out.addInput(0, lGain, 0);
            ac.out.addInput(1, rGain, 0);
        }

        Clock c = new Clock();
        c.getIntervalUGen().setValue(500);
        c.addMessageListener(new Bead() {
            @Override
            protected void messageReceived(Bead message) {
                if(c.getCount() % 2 == 0) {
                    for(int i = 0; i < NUM_VOICES; i++) {
                        if(rng.nextFloat() < 0.05f) {
                            if(rng.nextFloat() < 0.5f) {
                                voices[i].setSample(SampleManager.fromGroup("Stab", SCALE_NOTES[rng.nextInt(2)]));
                            } else {
//                                voices[i].setSample(SampleManager.fromGroup("Shhh", rng.nextInt(2)));
                                voices[i].setSample(SampleManager.fromGroup("ShortAh", SCALE_NOTES[rng.nextInt(2)]));
                            }
//                            voices[i].setPosition(rng.nextFloat() * 10);
                            voices[i].setPosition(0);
                        }
                    }
                }
            }
        });
        ac.out.addDependent(c);

        RecordToFile rtf = new RecordToFile(2, new File("file1.wav"));
        rtf.addInput(ac.out);
        ac.out.addDependent(rtf);
        DelayTrigger dt = new DelayTrigger(5000, new Bead() {
            @Override
            protected void messageReceived(Bead message) {
                rtf.kill();
                ac.stop();
                System.exit(0);
            }
        });
        ac.out.addDependent(dt);

    }
}
