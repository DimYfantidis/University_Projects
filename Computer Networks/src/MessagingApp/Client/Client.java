package MessagingApp.Client;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.Socket;


/**
 *  The following class implements the client program. <br>
 *  It is meant to be instantiated once in the {@link #main(String[]) main} function
 */
public class Client
{
    /** Client's socket for establishing connection with server. */
    private final Socket clientSocket;

    /** <b>Output stream:</b> sends messages to the client. */
    private final PrintWriter out;

    /** <b>Input stream:</b> receives messages from the server. */
    private final BufferedReader in;


    public Client(String ip, int port) throws IOException
    {
        clientSocket = new Socket(ip, port);
        out = new PrintWriter(new OutputStreamWriter(clientSocket.getOutputStream()), true);
        in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
    }

    /** @param arg Transmits an integer value to the server. */
    public void transmit(int arg) {
        out.println(arg);
    }

    /** @param arg Transmits a line of text to the server. */
    public void transmit(String arg) {
        out.println(arg);
    }

    /** Prints every line of text to received from the server. */
    public void receiveAll() throws IOException
    {
        String line;

        while ((line = in.readLine()) != null) {
            System.out.println(line);
        }
    }

    /** Terminates connection with the Server */
    public void shutdown() throws IOException
    {
        if (!clientSocket.isClosed()) {
            clientSocket.close();
        }
        in.close();
        out.close();
    }

    public static void main(String[] args)
    {
        final String ip = args[0];
        final int port = Integer.parseInt(args[1]);
        final int FN_ID = Integer.parseInt(args[2]);

        try
        {
            /* Sets up the client with the target ip, port
             * and declares the desired function id, as provided
             * from the command line. */
            Client client = new Client(ip, port);
            client.transmit(FN_ID);

            // Transmits the arguments after the function id.
            for (int i = 3; i < args.length; ++i) {
                client.transmit(args[i]);
            }
            client.receiveAll();

            client.shutdown();
        }
        catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
