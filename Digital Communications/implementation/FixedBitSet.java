import java.util.BitSet;
import java.util.InputMismatchException;


/**
 *  <p>
 *      The following class' instances represent <b>binary
 *      sequences</b> of a static length. The class inherits
 *      from the built in <b>BitSet</b> class and it is used as
 *      an improvement of the BitSet class, resolving problems
 *      regarding to length, the <b>toString()</b> method and provides
 *      methods for extra utility.
 *  </p>
 *
 *  @author Yfantidis Dimitris
 *  @see java.util.BitSet
 */
public class FixedBitSet extends BitSet
{
    // The length of the binary string.
    private final int length;

    public FixedBitSet(int length)
    {
        super(length);
        this.length = length;
    }

    // Copy constructor.
    public FixedBitSet(FixedBitSet other)
    {
        super(other.length);
        this.length = other.length();

        for (int i = 0; i < length; ++i) {
            this.set(i, other.get(i));
        }
    }

    // Parses a string to a BitSet.
    public static FixedBitSet parseBitSet(String bitSetStr) throws InputMismatchException
    {
        FixedBitSet b;
        String binarySequence;

        b = new FixedBitSet(bitSetStr.length());

        int i = 0;

        for (char bit : bitSetStr.toCharArray()) {
            if (bit != '1' && bit != '0') {
                throw new InputMismatchException("Input is not a binary sequence");
            }
            b.set(i++, bit == '1');
        }
        return b;
    }

    // Returns a subsequence starting from the origin-th bit from the MSB up to and including the bound-th bit from the MSB.
    public FixedBitSet slice(int origin, int bound)
    {
        if (origin < 0 || bound >= this.length || origin > bound) {
            return null;
        }
        FixedBitSet s = new FixedBitSet(bound - origin + 1);

        for (int i = origin; i <= bound; ++i) {
            s.set(i - origin, this.get(i));
        }
        return s;
    }

    // Sets the value of the least significant bit.
    public void setLSB(boolean flag) {
        set(length - 1, flag);
    }

    // Shifts the sequence's bits by n to the left (sequence <<= n).
    public void shiftLeft(int n)
    {
        for (int i = 0; i < length; ++i) {
            this.set(i, (i + n < length && this.get(i + n)));
        }
    }

    // Returns the remainder of the modulo-2 arithmetic division of the sequence and the given argument.
    public FixedBitSet modulo2Remainder(FixedBitSet divisor)
    {
        int i = divisor.length();

        FixedBitSet remainder = new FixedBitSet(divisor.length() - 1);

        FixedBitSet step = slice(0, divisor.length() - 1);

        while (i < length)
        {
            step.xor(divisor);

            while (!step.get(0) && i < length)
            {
                step.shiftLeft(1);
                step.setLSB(get(i));
                i += 1;
            }
        }

        for (i = 0; i < remainder.length(); ++i) {
            remainder.set(i, step.get(i + 1));
        }
        return remainder;
    }

    // Returns the fixed length of the bit sequence.
    @Override
    public int length() {
        return length;
    }

    // Returns the bit string represented by the object.
    @Override
    public String toString()
    {
        StringBuilder sb = new StringBuilder();

        for (int i = 0; i < length; ++i) {
            sb.append((this.get(i) ? '1' : '0'));
        }
        return sb.toString();
    }

    @Override
    public boolean equals(Object obj)
    {
        if (this == obj)
            return true;

        if (!(obj instanceof BitSet set))
            return false;

        if (length != set.length())
            return false;

        // Check words in use by both BitSets
        for (int i = 0; i < length; i++)
            if (get(i) != set.get(i))
                return false;

        return true;
    }
}
