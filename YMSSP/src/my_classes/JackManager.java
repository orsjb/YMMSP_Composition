package my_classes;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public abstract class JackManager {

    public static String run(int bufferSize) {
        boolean jackWasStartedByThisProcess = false;
        boolean error = false;
        Process process;
        StringBuffer stringBuffer = new StringBuffer("");
        try {
            ProcessBuilder pb = new ProcessBuilder();
            pb.command(new String("/usr/local/bin/jackdmp --realtime -s -d coreaudio -p "+bufferSize).split(" "));
            process = pb.start();
            BufferedReader reader =
                    new BufferedReader(new InputStreamReader(process.getInputStream()));
            String line = null;
            while ( (line = reader.readLine()) != null) {
                System.out.println(line);
                stringBuffer.append(line);
                if(line.contains("command not found")) {
                    break;
                }
                if(line.contains("driver is running")) {
                    jackWasStartedByThisProcess = true;
                    Runtime.getRuntime().addShutdownHook(new Thread() {
                        public void run() {
                            process.destroyForcibly();
                        }
                    });
                    break;
                } else if(line.contains("Failed to open server")) {
                    jackWasStartedByThisProcess = false;
                    break;
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
            error = true;
        }
        if(error) {
            return null;
        } else {
            return stringBuffer.toString();
        }

    }

}
