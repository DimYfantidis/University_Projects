package MessagingApp.Server;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;


/**
 * <p>
 *      Implementation of the server side. Extends the {@link java.lang.Thread Thread} class
 *      for terminating the execution asynchronously without raising <b>Ctrl+C exception</b>.
 *      Although it is meant to be instantiated once in the {@link #main(String[]) main} function.
 * </p>
 * <p>
 *     The <b>six functions</b> required for the project are stated inside the Server's
 *     {@link Server.ConnectionHandler connection handlers}. Each handler utilises a {@link BufferedReader} to
 *     listen for the client's expected arguments (listed below for each function) and a {@link PrintWriter} to
 *     send it the calculated results.
 *     <ol>
 *         <li>{@link ConnectionHandler#createAccount() Create an Account} &lt;&#0;username&#0;&gt;</li>
 *         <li>{@link ConnectionHandler#printAccounts() Print all Accounts} &lt;&#0;authToken&#0;&gt;</li>
 *         <li>{@link ConnectionHandler#sendMessage() Send a Message} &lt;&#0;authToken&#0;&gt; &lt;&#0;recipient&#0;&gt; &lt;&#0;message_body&#0;&gt;</li>
 *         <li>{@link ConnectionHandler#showInbox() Show Inbox} &lt;&#0;authToken&#0;&gt;</li>
 *         <li>{@link ConnectionHandler#readMessage() Read selected Message} &lt;&#0;authToken&#0;&gt; &lt;&#0;message_id&#0;&gt;</li>
 *         <li>{@link ConnectionHandler#deleteMessage() Delete selected Message} &lt;&#0;authToken&#0;&gt; &lt;&#0;message_id&#0;&gt;</li>
 *     </ol>
 * </p>
 */
public class Server extends Thread
{
    /** <b>List of clients</b> that are being served concurrently. */
    private final ArrayList<ConnectionHandler> connections;

    /** The server's endpoint. */
    private final ServerSocket serverSocket;

    /** Terminates the server's connection when evaluated to true. */
    private volatile boolean done;


    /** Client request error message: <b>Invalid token</b>. */
    private static final String AUTH_TOKEN_ERR = "Invalid Auth Token";

    /** <b>Logs actions</b> to the server's terminal. */
    private static final Logger logger = new Logger(System.out);


    /**
     *  <p>
     *      Inside the {@link ConnectionHandler} class resides the implementation for
     *      establishing connection with a client and <b>performing its requests</b>,
     *      depending on different <i>FN_IDs</i> and arguments (as the project intends).
     *  </p>
     *  <p>
     *      A {@link Server} instance <u>does nothing more but create many connection
     *      handlers</u> in order to serve many clients concurrently.
     *  </p>
     *  @see MessagingApp.Server.Server#run() Server.run()
     *  @see MessagingApp.Server.Server.ConnectionHandler#run() ConnectionHandler.run()
     */
    public class ConnectionHandler extends Thread
    {
        /** The client's endpoint. */
        private final Socket clientSocket;

        /** <b>Input stream:</b> receives messages from the client. */
        private final BufferedReader in;

        /** <b>Output stream:</b> sends messages to the client. */
        private final PrintWriter out;


        private ConnectionHandler(Socket client) throws IOException
        {
            this.clientSocket = client;
            in = new BufferedReader(new InputStreamReader(client.getInputStream()));
            out = new PrintWriter(new OutputStreamWriter(client.getOutputStream()), true);
        }

        /**
         *  Implementation of function for <b>FN_ID = 1</b>.
         */
        public void createAccount() throws IOException
        {
            String username = in.readLine();

            // Logs client request and received argument.
            Server.logger.report(
                    "Client %s requested action 1 (Account creation)", clientSocket.getInetAddress()
            ).argument(username);


            // Returns true if a non-word character ([^a-zA-Z0-9]) is found in the username.
            if (Pattern.compile("\\W").matcher(username).find())
            {
                out.println("Invalid Username");
                return;
            }
            // Creates the new account
            Account account = Account.store(username);

            if (account == null)
            {
                out.println("Sorry, the user already exists");
                return;
            }
            // Transmits the authentication token to the client.
            out.println(account.getAuthToken());
        }

        /** Implementation of function for <b>FN_ID = 2</b>. */
        public void printAccounts() throws IOException
        {
            int authToken;

            // Logs client request.
            Server.logger.report("Client %s requested action 2 (Print accounts)", clientSocket.getInetAddress());

            try {
                authToken = Integer.parseInt(in.readLine());
            }
            catch (NumberFormatException e) {
                Server.logger.report("Parse error: Could not resolve authentication token");
                return;
            }
            Account account = Account.extract(authToken);

            // Logs received argument.
            Server.logger.argument(authToken);


            if (account == null)
                out.println(Server.AUTH_TOKEN_ERR);
            else
                Account.printUsernames(out);
        }

        /** Implementation of function for <b>FN_ID = 3</b>. */
        public void sendMessage() throws IOException
        {
            int authToken;

            // Logs client request.
            Server.logger.report("Client %s requested action 3 (Send message)", clientSocket.getInetAddress());

            try {
                authToken = Integer.parseInt(in.readLine());
            }
            catch (NumberFormatException e) {
                Server.logger.report("Parse error: Could not resolve authentication token");
                return;
            }

            Account sender = Account.extract(authToken);
            Account recipient = Account.extract(in.readLine());

            if (sender == null) {
                out.println(Server.AUTH_TOKEN_ERR);
                return;
            }
            if (recipient == null)
            {
                out.println("User does not exist");
                return;
            }
            String body = in.readLine();

            // Logs all received arguments.
            Server.logger.argument(authToken).argument(recipient.getUsername()).argument(body);

            // Stores a new message to the receiver's inbox.
            recipient.addMessage(
                    new Message.Builder()
                            .body(body)
                            .sender(sender.getUsername())
                            .receiver(recipient.getUsername())
                            .messageId(recipient.getMessageBox().size() + 1)     // ID is assigned list's size
                            .build()
            );
        }

        /** Implementation of function for <b>FN_ID = 4</b>. */
        public void showInbox() throws IOException
        {
            // Logs client request.
            Server.logger.report("Client %s requested action 4 (Show inbox)", clientSocket.getInetAddress());

            int authToken;

            try {
                authToken = Integer.parseInt(in.readLine());
            }
            catch (NumberFormatException e) {
                Server.logger.report("Parse error: Could not resolve authentication token");
                return;
            }

            // Logs received argument.
            Server.logger.argument(authToken);

            Account account = Account.extract(authToken);

            if (account == null)
            {
                out.println(Server.AUTH_TOKEN_ERR);
                return;
            }

            // Prints all non-deleted messages in the user's message box.
            for (Message message : account.getMessageBox())
            {
                if (message.isDeleted())
                    continue;

                out.printf("%d. from: %s%s%n",
                        message.id(),
                        message.sender(),
                        (message.isRead() ? "" : "*")
                );
            }
        }

        /** Implementation of function for <b>FN_ID = 5</b>. */
        public void readMessage() throws IOException
        {
            int authToken;
            int messageId;

            // Logs client request.
            Server.logger.report("Client %s requested action 5 (Read message)", clientSocket.getInetAddress());

            try {
                authToken = Integer.parseInt(in.readLine());
                messageId = Integer.parseInt(in.readLine());
            }
            catch (NumberFormatException e) {
                Server.logger.report("Parse error: Could not resolve authentication token or message id");
                return;
            }
            // Logs received arguments.
            Server.logger.argument(authToken).argument(messageId);

            Account account = Account.extract(authToken);

            if (account == null)
            {
                out.println(Server.AUTH_TOKEN_ERR);
                return;
            }
            List<Message> messages = account.getMessageBox();

            // Message id is either not in the database, or hase been deleted.
            if (messageId < 1 || messageId > messages.size() || messages.get(messageId - 1).isDeleted())
            {
                out.println("Message ID does not exist");
                return;
            }
            Message message = messages.get(messageId - 1);

            // Shows message.
            out.printf("(%s)%s%n", message.sender(), message.text());
            // Changes message's state to read.
            message.read();
        }

        /** Implementation of function for <b>FN_ID = 6</b>. */
        public void deleteMessage() throws IOException
        {
            int authToken;
            int messageId;

            Server.logger.report("Client %s requested action 6 (Delete message)", clientSocket.getInetAddress());

            try {
                authToken = Integer.parseInt(in.readLine());
                messageId = Integer.parseInt(in.readLine());
            }
            catch (NumberFormatException e) {
                Server.logger.report("Parse error: Could not resolve authentication token or message id");
                return;
            }
            // Logs received arguments.
            Server.logger.argument(authToken).argument(messageId);

            Account account = Account.extract(authToken);

            if (account == null)
            {
                out.println(Server.AUTH_TOKEN_ERR);
                return;
            }
            List<Message> messages = account.getMessageBox();

            // Message id is either not in the database, or hase been deleted.
            if (messageId < 1 || messageId > messages.size() || messages.get(messageId - 1).isDeleted())
            {
                out.println("Message ID does not exist");
                return;
            }
            messages.get(messageId - 1).delete();
        }

        /** Executes when a client requests service. */
        @Override
        public void run()
        {
            try
            {
                int FN_ID;

                try
                {
                    // Listens to the clients demand (FN_ID expected).
                    FN_ID = Integer.parseInt(in.readLine());
                }
                catch (NumberFormatException e)
                {
                    Server.logger.report("Invalid client argument: expected integer value");
                    shutdown();
                    return;
                }

                // Transfers control to the desired procedure.
                switch (FN_ID)
                {
                    case 1 -> createAccount();
                    case 2 -> printAccounts();
                    case 3 -> sendMessage();
                    case 4 -> showInbox();
                    case 5 -> readMessage();
                    case 6 -> deleteMessage();
                    default -> Server.logger.report("Invalid client function request: FN_ID: %d", FN_ID);
                }
                // Client is satisfied and connection terminates.
                shutdown();
                // Client is removed from the list.
                connections.remove(this);
            }
            catch (IOException e) {
                e.printStackTrace();
            }
        }

        /**
         *  <b>Terminates</b> the thread of service when the client is satisfied. <br>
         *  De-allocates communication resources.
         */
        public void shutdown() throws IOException
        {
            Server.logger.report("Client %s disconnecting.", clientSocket.getInetAddress());

            if (!clientSocket.isClosed()) {
                clientSocket.close();
            }
            in.close();
            out.close();

            this.interrupt();
        }
    }

    public Server(int port) throws IOException
    {
        serverSocket = new ServerSocket(port);
        connections = new ArrayList<>();
        done = false;
    }

    @Override
    public void run()
    {
        // Prompts the server's terminal.
        logger.report(
                "Server started @%s - port:%s", serverSocket.getInetAddress(), serverSocket.getLocalPort()
        );

        // Executes until interrupted from keyboard (see main()).
        while (!done)
        {
            try
            {
                // Accepts the next incoming client.
                Socket clientSocket = serverSocket.accept();

                logger.report(
                        "Client %s has connected successfully.", clientSocket.getInetAddress()
                );
                // Serves the client asynchronously while awaiting the next client.
                ConnectionHandler connectionThread = new ConnectionHandler(clientSocket);
                connections.add(connectionThread);
                connectionThread.start();
            }
            catch (IOException e)
            {
                if (done) continue;

                logger.report("Connection Error: Could not establish connection with client.");
            }
        }
    }

    /**
     *  <p>
     *      <b>Terminates</b> the entire server when prompted from its terminal.
     *      De-allocates communication resources, thus rejecting new clients and
     *      disconnects existing clients before shutting down completely.
     *  </p>
     */
    public void shutdown() throws IOException
    {
        logger.report("Shutting connection down...");
        done = true;

        try {
            Thread.sleep(1000);
        }
        catch (InterruptedException e) { /* Ignore */ }

        if (!serverSocket.isClosed()) {
            serverSocket.close();
        }
        logger.report("Awaiting clients to finish...");

        for (ConnectionHandler connection : connections) {
            connection.shutdown();
        }
        this.interrupt();
    }

    public static void main(String[] args)
    {
        int port = Integer.parseInt(args[0]);

        try (BufferedReader keyboard = new BufferedReader(new InputStreamReader(System.in)))
        {
            Server server = new Server(port);
            server.start();

            // Blocks execution to prevent termination of server until prompted otherwise.
            keyboard.readLine();

            server.shutdown();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }
}
