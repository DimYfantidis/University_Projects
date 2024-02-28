package MessagingApp.Server;

import java.io.PrintStream;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;


public class Logger
{
    /** The output in which system actions will be reported */
    private final PrintStream out;

    /**
     *  <b>Timestamp formatter</b>
     *  @see #timestamp()
     */
    private static final DateTimeFormatter timeFormatter;

    /** <b>[IGNORE]</b> Auxiliary String for aesthetic purposes during logging.*/
    private static final String space;


    static
    {
        final String format = "dd/MM/yyyy - HH:mm:ss";

        timeFormatter = DateTimeFormatter.ofPattern(format);
        space = new String(new char[format.length()]).replace('\0', ' ');
    }

    /** Returns the current timestamp. Useful for <b>logging actions</b> on the server's terminal. */
    private static String timestamp() {
        return LocalDateTime.now().format(timeFormatter);
    }

    public Logger(PrintStream out) {
        this.out = out;
    }

    /**
     *  <p>
     *      <b>Logs the given string</b> to the output stream, {@link #out}, alongside the
     *      exact <b>date and time</b> in the manner {@link #timeFormatter} dictates.
     *  </p>
     *  @param format The format string to be logged
     *  @param args The format arguments
     *  @return <b>this</b> object to support chaining.
     */
    public Logger report(String format, Object... args)
    {
        out.printf("[%s] > ", timestamp());
        out.printf(format, args);
        out.println();
        return this;
    }

    /**
     *  Logs the given client's argument.
     *  @return <b>this</b> object to support chaining.
     */
    public Logger argument(Object arg)
    {
        out.println(space + "   >>> arg: " + arg);
        return this;
    }
}
