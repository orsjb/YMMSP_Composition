package my_classes;

import de.sciss.net.OSCListener;
import de.sciss.net.OSCMessage;
import net.beadsproject.beads.data.Buffer;
import net.beadsproject.beads.ugens.Gain;
import net.beadsproject.beads.ugens.WavePlayer;
import net.happybrackets.core.HBAction;
import net.happybrackets.core.control.IntegerTextControl;
import net.happybrackets.core.control.TriggerControl;
import net.happybrackets.device.HB;

import java.lang.invoke.MethodHandles;
import java.net.SocketAddress;

public class HowManyOscillators implements HBAction {
    // Change to the number of audio Channels on your device
    final int NUMBER_AUDIO_CHANNELS = 1;
    int numOscillators = 0;
    Gain masterGain;

    @Override
    public void action(HB hb) {
        // remove this code if you do not want other compositions to run at the same time as this one
        hb.reset();
        hb.setStatus(this.getClass().getSimpleName() + " Loaded");

        masterGain = new Gain(1, 0.1f);
        hb.ac.out.addInput(masterGain);


        /*************************************************************
         * Create an integer type Dynamic Control that displays as a text box
         * Simply type intTextControl to generate this code
         *************************************************************/
        IntegerTextControl numOscDisplay = new IntegerTextControl(this, "Num Oscillators", 0) {
            @Override
            public void valueChanged(int control_val) {
                /*** Write your DynamicControl code below this line ***/

                /*** Write your DynamicControl code above this line ***/
            }
        };/*** End DynamicControl numOscDisplay code ***/

/*************************************************************
 * Create a Trigger type Dynamic Control that displays as a button
 * Simply type triggerControl to generate this code
 *************************************************************/
        TriggerControl triggerControl = new TriggerControl(this, "Add") {
            @Override
            public void triggerEvent() {
                /*** Write your DynamicControl code below this line ***/
                numOscillators++;
                masterGain.setGain(0.1f / numOscillators);
                WavePlayer wp = new WavePlayer(500+hb.rng.nextFloat()*50, Buffer.SINE);
                masterGain.addInput(wp);
                numOscDisplay.setValue(numOscillators);
                /*** Write your DynamicControl code above this line ***/
            }
        };/*** End DynamicControl triggerControl code ***/


        for(int i = 0; i < 15; i++) {
            triggerControl.send();
        }

    }

    //<editor-fold defaultstate="collapsed" desc="Debug Start">

    /**
     * This function is used when running sketch in IntelliJ IDE for debugging or testing
     *
     * @param args standard args required
     */
    public static void main(String[] args) {

        try {
            HB.runDebug(MethodHandles.lookup().lookupClass());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    //</editor-fold>
}
