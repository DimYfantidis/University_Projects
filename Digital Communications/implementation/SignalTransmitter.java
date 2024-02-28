/**
 * <p>
 *      <b>Signal transmitters</b> are responsible for transmitting
 *      signals of binary sequences to their corresponding receivers.
 *      They are constructed with a (N-K+1)-bit sequence that will be
 *      constant throughout the object's lifespan and will be used to
 *      compute the CRC sequences for transmission.
 *      More specifically, the following steps must be executed to
 *      transmit a signal to a <b> signal receiver</b>:
 * </p>
 *  <ul>
 *      <li>
 *          Definition of the bit error rate of the transmission medium (ranges from
 *          0 to 1, defaults to 0).
 *      </li>
 *      <li>
 *          Definition the K-bit data block containing the information to be transmitted.
 *      </li>
 *      <li>
 *          Computation of the total sequence to be transmitted using cyclic redundancy
 *          check with the use of the transmitter's divisor.
 *      </li>
 *      <li>
 *          Transmission of the total sequence to the corresponding signal receiver.
 *      </li>
 *  </ul>
 *  <b>Clarification</b>: a signal receiver is a correspondent of the
 *  signal transmitter when they have <b>the same divisor</b>.
 *
 *  @author Yfantidis Dimitris
 *  @see SignalReceiver
 *  @see FixedBitSet
 */
public class SignalTransmitter
{
    // The K-bit data block.
    private FixedBitSet dataBlock;

    // The N-bit sequence equal to the concatenation of 2^(N-K) * dataBlock and the Frame Check Sequence (FCS).
    private FixedBitSet totalSequence;

    // The (N-K+1)-bit sequence for computing the FCS.
    private final FixedBitSet divisor;

    private int K;

    private int N;


    // A signal transmitter sends data blocks whose FCS correspond to a specific divisor.
    public SignalTransmitter(FixedBitSet divisor) {
        this.divisor = new FixedBitSet(divisor);
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

    public void setDataBlock(FixedBitSet data)
    {
        dataBlock = new FixedBitSet(data);

        totalSequence = null;

        K = dataBlock.length();

        // Len(P) = N - K + 1 => N = Len(P) + K - 1
        N = divisor.length() + K - 1;
    }

    public FixedBitSet getTotalSequence() {
        return totalSequence;
    }

    /**
     *  <p>
     *      Creates the N-bit sequence T to be transmitted.
     *      T is initialized to 2^(N-K) * D, where D the given data block.
     *      Then the (N-K)-bit FCS is defined as the remainder of modulo-2
     *      division of T and the transmitter's divisor.
     *  </p>
     *  <p>
     *      Finally, the FCS is assigned to last (N-K) bits of T.
     *  </p>
     *  @see FixedBitSet FixedBitSet.modulo2Remainder(FixedBitSet divisor)
     */
    public boolean computeFrameCheckSequence()
    {
        if (dataBlock == null) {
            return false;
        }
        totalSequence = new FixedBitSet(N);

        int i;

        for (i = 0; i < K; ++i) {
            this.totalSequence.set(i, dataBlock.get(i));
        }

        FixedBitSet FCS = totalSequence.modulo2Remainder(divisor);

        for (i = dataBlock.length(); i < this.totalSequence.length(); ++i) {
            totalSequence.set(i, FCS.get(i - dataBlock.length()));
        }
        return true;
    }
}
