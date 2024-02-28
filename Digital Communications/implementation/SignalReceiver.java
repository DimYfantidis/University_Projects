/**
 * <p>
 *      <b>Signal receivers</b> are responsible for receiving signals
 *      of binary sequences by their corresponding <b>signal transmitters.</b>
 *      They are constructed with a (N-K+1)-bit sequence that will be
 *      constant throughout the object's lifespan and will be used to
 *      validate the received signal.
 * </p>
 *  <b>Clarification</b>: a signal receiver is a correspondent of the
 *  signal transmitter when they have <b>the same divisor</b>.
 *
 *  @author Yfantidis Dimitris
 *  @see SignalTransmitter
 *  @see FixedBitSet
 */
public class SignalReceiver
{
    // The divisor used for validating the received signal.
    private final FixedBitSet divisor;

    // The signal received from the divisor
    private FixedBitSet receivedSignal;

    public SignalReceiver(FixedBitSet divisor) {
        this.divisor = new FixedBitSet(divisor);
    }

    public FixedBitSet getReceivedSignal() {
        return receivedSignal;
    }

    // A copy of the divisor is returned so that it will not be edited (read only)
    public FixedBitSet getDivisor() {
        return new FixedBitSet(divisor);
    }

    // For performance boost without loss of security, the pointer of the divisor is returned if the proper
    // argument is provided by a SignalChannel.
    public FixedBitSet getDivisor(SignalChannel.SecuritySignature dummy) {
        return (dummy == null ? getDivisor() : divisor);
    }


    // An SignatureSecurity instance is required to receive a signal,
    // ensuring receivers will receive signals only through SignalChannels.
    public void setReceivedSignal(FixedBitSet receivedSignal, SignalChannel.SecuritySignature ignoredDummy) {
        this.receivedSignal = new FixedBitSet(receivedSignal);
    }

    public boolean signalAltered()
    {
        FixedBitSet remainder = receivedSignal.modulo2Remainder(divisor);

        for (int i = 0; i < remainder.length(); ++i) {
            if (remainder.get(i)) {
                return true;
            }
        }
        return false;
    }
}
