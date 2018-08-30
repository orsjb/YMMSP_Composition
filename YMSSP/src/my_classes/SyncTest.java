package my_classes;

import net.beadsproject.beads.core.Bead;
import net.happybrackets.core.HBAction;
import net.happybrackets.core.HBReset;
import net.happybrackets.device.HB;

import java.lang.invoke.MethodHandles;

public class SyncTest implements HBAction, HBReset {
    // Change to the number of audio Channels on your device
    final int NUMBER_AUDIO_CHANNELS = 1;

    @Override
    public void action(HB hb) {
        // remove this code if you do not want other compositions to run at the same time as this one
        hb.reset();
        hb.setStatus(this.getClass().getSimpleName() + " Loaded");
        //
        hb.pattern(new Bead() {
            @Override
            protected void messageReceived(Bead message) {
                if (hb.clock.isBeat()) {

                }
            }
        });

    }


    /**
     * Add any code you need to have occur when a reset occurs
     */
    @Override
    public void doReset() {
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
