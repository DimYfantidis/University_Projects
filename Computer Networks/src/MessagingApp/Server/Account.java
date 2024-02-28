package MessagingApp.Server;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


/**
 *  <b>Account</b> class, used for representing the users' accounts in the system. <br>
 *  It provides static members and methods for implementing the account database.
 */
public class Account
{
    private final String username;

    private final int authToken;

    private final List<Message> messageBox;


    private Account(String username, int authToken)
    {
        this.username = username;
        this.authToken = authToken;
        messageBox = new ArrayList<>();
    }

    public int getAuthToken(){ return authToken; }

    public String getUsername() { return username; }

    public List<Message> getMessageBox(){ return messageBox; }

    public void addMessage(Message message) { messageBox.add(message); }



    /** All saved accounts, accessed by their <b>tokens</b>. */
    private static final HashMap<Integer, Account> databaseByToken = new HashMap<>();

    /** All saved accounts, accessed by their <b>usernames</b>. */
    private static final HashMap<String, Account> databaseByUsername = new HashMap<>();


    /**
     *  Searches for the account with the given <b>username</b> in the database.
     *  @return The account with the corresponding username. If the username is not found, <b>null</b> is returned.
     */
    public static Account extract(String username)
    {
        if (databaseByUsername.containsKey(username)) {
            return databaseByUsername.get(username);
        }
        return null;
    }

    /**
     *  Searches for the account with the given <b>authentication token</b> in the database.
     *  @return The account with the corresponding token. If the token is not found, <b>null</b> is returned.
     */
    public static Account extract(int authToken)
    {
        if (databaseByToken.containsKey(authToken)) {
            return databaseByToken.get(authToken);
        }
        return null;
    }

    /**
     *  Stores a newly created account in the database.
     *  @return The new account. If the account exists already, <b>null</b> is returned.
     */
    public static Account store(String username)
    {
        int token = username.hashCode();

        while (databaseByToken.containsKey(token))
        {
            if (databaseByToken.get(token).getUsername().equals(username)) {
                // User already exists
                return null;
            }
            else {
                /* A different user with the same token exists.
                 * Increments token's value to guarantee its uniqueness 
                 * and performs the same check again. */
                token += 1;
            }
        }
        Account newAccount = new Account(username, token);

        databaseByToken.put(token, newAccount);
        databaseByUsername.put(username, newAccount);

        return newAccount;
    }

    /**
     *  Prints all account usernames from the database to the given print stream.
     *  @param out The output stream.
     */
    public static void printUsernames(PrintWriter out)
    {
        int i = 1;

        for (String username : databaseByUsername.keySet())
        {
            out.println(i + ". " + username);
            i += 1;
        }
    }
}
