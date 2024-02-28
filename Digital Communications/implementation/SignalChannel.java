import java.util.Random;


/**
 *  The following class' instances represent a transmission
 *  medium with a specific bit error rate. Binary sequences
 *  are transmitted from the <b>signal transmitters</b> to
 *  the <b>signal receivers</b> only through <b>signal channels</b>
 *  and by no other means.
 *
 *  @author Yfantidis Dimitris
 */
public class SignalChannel
{
    // Generator for causing disturbances in the transmitted signal.
    private static final Random noise = new Random();

    // The bit error rate of the medium.
    private final double BER;

    public static class SecuritySignature
    {
        private SecuritySignature() {}
    }
    // The signature will ensure that receivers will only receive signals through the use of SignalChannels,
    // as they cannot be constructed outside the class. Failing to provide an SignatureSecurity to the
    // receiver will result in a compilation error.
    private final SecuritySignature key;


    public SignalChannel(double BER)
    {
        if (BER < 0 || BER > 1) {
            throw new IllegalArgumentException("Bit error rate must have value in range [0, 1]");
        }
        this.BER = BER;

        key = new SecuritySignature();
    }

    /**
     *  <p>
     *      Transmits the total sequence (data + FCS) from a <b>signal transmitter</b>
     *      to a <b>signal receiver</b> through the current medium. It is important that
     *      the transmitter and the receiver have the same divisor.
     *  </p>
     *  <p>
     *      The signal has a chance of being altered due to noise, dictated by
     *      the bit error rate.
     *  </p>
     *  @param transmitter the object responsible for transmitting the data and the FCS to the receiver.
     *  @param receiver the object responsible for receiving the signal and checking its validity afterwards.
     *  @return the number of bit errors that occurred during transmission due to noise.
     */
    public int transmitSignal(SignalTransmitter transmitter, SignalReceiver receiver)
    {
        int bitErrors = 0;

        if (transmitter.getTotalSequence() == null) {
            throw new IllegalStateException(
                    "CRC signal has not been computed yet. Execute SignalTransmitter.computeTotalSequence() first."
            );
        }
        if (!transmitter.getDivisor(key).equals(receiver.getDivisor(key))) {
            throw new IllegalArgumentException("Receiver object must have the same divisor as the transmitter");
        }

        FixedBitSet totalSequence = new FixedBitSet(transmitter.getTotalSequence());

        int N = totalSequence.length();

        for (int i = 0; i < N; ++i)
        {
            if (noise.nextDouble(0, 1) < BER) {
                bitErrors += 1;
                totalSequence.flip(i);
            }
        }
        receiver.setReceivedSignal(totalSequence, key);

        return bitErrors;
    }
}
