import java.lang.System;
import java.lang.Integer;
import java.lang.Long;
import java.lang.Double;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Random;
import java.util.Scanner;


public class Main
{
    // Generator for producing random bit sequences.
    private static final Random generator = new Random();

    // Scanner for user input
    static Scanner scanner = new Scanner(System.in);


    // Creates an expression of milliseconds as (hours : minutes : seconds : milliseconds) and returns it as a string.
    public static String clockFormat(long ms)
    {
        long sec;
        long min;
        long h;

        sec = ms / 1_000;
        ms %= 1_000;

        min = sec / 60;
        sec %= 60;

        h = min / 60;
        min %= 60;

        return String.format("(%02dh : %02dmin : %02dsec : %03dms)", h, min, sec, ms);
    }


    // The main program.
    public static void main(String[] args)
    {
        // The length of the data blocks.
        final int K;

        // The length of the total sequence to be transmitted.
        final int N;

        // The bit error rate
        final double BER;

        // The total number of signals (of N bits each) to be transmitted.
        final long totalSignals;

        // The divisor used in Cyclic Redundancy Check, with len(P) = N - K + 1.
        final FixedBitSet P;

        FixedBitSet dataBlock;

        // The object for computing and transmitting the total sequences of the data blocks.
        SignalTransmitter transmitter;

        // The object for receiving a sequence from the transmitter
        // and checking for any possible alterations in the signal.
        SignalReceiver receiver;

        // The channel in which the signals will be transmitted in.
        SignalChannel channel;

        System.out.print("Input data blocks' number of bits: ");
        K = Integer.parseInt(scanner.nextLine());

        System.out.print("Input divisor (in binary): ");
        P = FixedBitSet.parseBitSet(scanner.nextLine());

        // Len(P) = N - K + 1 => N = K + Len(P) - 1
        N = K + P.length() - 1;

        System.out.print("Input bit error rate (BER): ");
        BER = Double.parseDouble(scanner.nextLine().replace(",", "."));

        System.out.print("Input total signals for transmission: ");
        totalSignals = Long.parseLong(scanner.nextLine());

        System.out.println();

        transmitter = new SignalTransmitter(P);

        receiver = new SignalReceiver(P);

        channel = new SignalChannel(BER);


        // Total bits flipped due to noise disturbances.
        long bitErrors = 0;

        // Total signals who did not pass the receiver's validity check.
        long estimatedAlteredSignals = 0;

        // Total signals actually affected by noise.
        long actualAlteredSignals = 0;

        // Signals that got altered by noise but were not detected by CRC procedure.
        long undetectedAlteredSignals;

        // Variables for monitoring elapsed time.
        long start;
        long stop;

        start = System.currentTimeMillis();

        for (long i = 0; i < totalSignals; ++i)
        {
            // New data block of K bits (initialised to 0) is created.
            dataBlock = new FixedBitSet(K);

            // Random flipping of bits
            for (int j = 0; j < K; ++j) {
                dataBlock.set(j, generator.nextBoolean());
            }
            // Sequence is transferred to the transmitter.
            transmitter.setDataBlock(dataBlock);

            // Computes the FCS of the given data block.
            if (!transmitter.computeFrameCheckSequence()) {
                System.err.println("Data Block has been set to null or it has not been set yet");
            }

            // Transmits the given sequence with its FCS to the receiver and
            // returns the bit errors that occurred during transmission.
            bitErrors += channel.transmitSignal(transmitter, receiver);

            // The receiver checks the validity of the received signal.
            if (receiver.signalAltered()) {
                estimatedAlteredSignals += 1;
            }

            // Absolute validity check.
            if (!transmitter.getTotalSequence().equals(receiver.getReceivedSignal())) {
                actualAlteredSignals += 1;
            }
        }

        stop = System.currentTimeMillis();

        // Number formatting (not important).
        DecimalFormat df = new DecimalFormat(); // ignore

        DecimalFormatSymbols ds = DecimalFormatSymbols.getInstance(); // ignore

        ds.setDecimalSeparator(',');    // ignore
        ds.setGroupingSeparator('.');   // ignore

        df.setDecimalFormatSymbols(ds); // ignore


        // Output of the results.
        System.out.format(
                "Total Bits altered: %s (%s%%)%n",
                df.format(bitErrors), df.format(100.0 * bitErrors / (totalSignals * N))
        );
        System.out.println();

        int width = df.format(actualAlteredSignals).length(); // ignore

        System.out.format(
                "-> Estimated signals altered:  %" + width + "s (%s%%)%n",
                df.format(estimatedAlteredSignals), df.format(100.0 * estimatedAlteredSignals / totalSignals)
        );
        System.out.format(
                "-> Total signals altered:      %" + width + "s (%s%%)%n",
                df.format(actualAlteredSignals), df.format(100.0 * actualAlteredSignals / totalSignals)
        );
        undetectedAlteredSignals = actualAlteredSignals - estimatedAlteredSignals;

        System.out.format(
                "-> Undetected altered signals: %" + width + "s (%s%%)%n",
                df.format(undetectedAlteredSignals), df.format(100.0 * undetectedAlteredSignals / totalSignals)
        );
        System.out.println();

        System.out.println("Process finished after " + clockFormat(stop - start));
    }
}