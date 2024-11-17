## Computer Networks (Java)
The course's target is to introduce the students to internet protocols (TCP, HTTP etc.) and network programming. The assignment requests the implementation of a program/service that consists of a **Server** side and a **Client** side using sockets or Remote Method Invocation in Java. The two sides of the service can be executed in different computers and interact with each other through access to the internet.

<br/>

## Code Specifics - Project Report (Deliverable)

### **Introduction**

The current project was implemented in the **Java (v19.0.1)** programming language using **Sockets** (Socket, ServerSocket, Thread, I/O Streams).  
Initially the _Client side_ will be presented, followed by the _Server side_ of the implementation.  
Below, the classes that were implemented are listed, along with their organizational structure within the project:

**MessagingApp** package:
* **Client** package:
    * _Client_ class 
* **Server** package:
    * _Message_ class
        * _Message.Builder_ class
    * _Account_ class
    * _Logger_ class
    * _Server_ class
        * _Server.ConnectionHandler_ class

**<span style="color:red">
It should be noted that neither the Remote Method Invocation (RMI) technology, nor Serialization were used.  
The communication between Server and Client is performed exclusively through the transmission of<u>_strings_</u>  
through <u>_text streams_</u> and not object streams.
</span>**

<hr>

### **_Client Side_**
The Client Side is implemented within the Client class which has three members: 

```java
private final Socket clientSocket;  // For communicating through the TCP protocol
private final PrintWriter out;      // Sends messages from Client to Server
private final BufferedReader in;    // Receives messages sent by Server
```

The Client object is instantiated, and through class methods communication is performed.  
Through the static function **Client.main(String[] args)** it is evident that the implementation is broken down in three segments:
> 1. Proper **parsing** is performed on the provided command line arguments.  
> 2. Through them the Client object is constructed and, with the utilization of **transmit(...)**, the following are sent:
>    * the FN_ID
>    * the expected arguments that follow after the corresponding FN_ID
> 3. The expected results are returned through **receiveAll()** and printed, according to the Server's calculations, and then the client will terminate.

**<span style="color:red">
The Client's side of implementation has no awareness of other classes
</span>** (i.e. Message, Account etc).  
It just awaits to receive lines of text in order to print them, i.e. it functions as a terminal.

<hr>

### **_Server Side_**

<br>

> **Κλάση Message:**  
implements the requested class members, with the inclusion of the **isDeleted** boolean member.
```java
private boolean isRead;         // If the message was read by the receiver
private boolean isDeleted;      // The messages are not deleted from the conversation, just marked as such
private final int messageId;    // Assigned to its location in the user's inbox
private final String sender;    // The sender's username
private final String receiver;  // The receiver's username
private final String body;      // The message's text
```
The nested **Builder** builder class is present as this was the followed designed pattern for building the messages.  

<br> <br>

> **Κλάση Account:**  
Represents the app's users. The requested class members are:  
```java
private final String username;          // User's name
private final int authToken;            // User's password
private final List<Message> messageBox; // Message inbox
```

Includes different static meethods and members for interacting with the app's virtual database, e.g.:
```java
private static final HashMap<Integer, Account> databaseByToken;    // Quick password-based lookup of users
private static final HashMap<String, Account> databaseByUsername;  // Quick username-based lookup of users
```

Direct use of class constructor is prohibited. Methods such as **extract(...)** and **store(String)** are used instead.  

<br> <br>

> **Κλάση Logger:**  
Simply prints the different events on the server's terminal such as
* Boot-up and Termination
* Client connection (_requedsted function, parameters, etc_)
* Date and time of event (timestamp)  

<br> <br>

> **Κλάση ConnectionHandler** (Nested within Server):  
Every instance serves a new client.  

Derived by the **Thread** class so that the server can accommodate many clients concurrently. Its class members are:
```java
private final Socket clientSocket;  // The client's TCP socket
private final BufferedReader in;    // Reads messages from the client
private final PrintWriter out;      // Sends messages to the client
```
By using the overloaded **run()** function, the server reads the provided FN_ID and, through the use of a *switch* statement,  
its respective method is called. Each method implements the corresponding functionality which is requested in the header.  
Thus, the ConnectionHandler class has **6 other basic methods** for this exact reason.  
```java
public void createAccount() throws IOException;
public void printAccounts() throws IOException;
public void sendMessage() throws IOException;
public void showInbox() throws IOException;
public void readMessage() throws IOException;
public void deleteMessage() throws IOException;
```
Each one of these functions utilizes the input stream as many times as the arguments are expected to be.  
In the end, the results are transmitted to the client. 

*There is no explicit check for detecting insufficient number of arguments, but in this case there will an error will be raised either way,  
which will not affect negatively the server, nor its database*.

<br> <br>

> **Κλάση Server:**  
The implementation of the Server Side, along with the nested **ConnectionHandler** class.

Extends the **Thread** class so that it can terminate normally, without using an exception.  
I.e., an asynchronous press of the _Enter_ button is expected for terminating the server. Contains the following members:
```java
private final ServerSocket serverSocket; // The TCP socket
private volatile boolean done; // Terminal condition of the overridden run() method
private final ArrayList<ConnectionHandler> connections; // The list of concurrent clients that are currently being accommodated
```
In essence, the server's interaction with a client is implemented in the ConnectionHandler.  
The practical functionality of the Server class is less complicated and is situated in the overriden **run()** method:
1. Expects the arrival of a client
2. Assigns them to a ConnectionHandler to **ασύγχρονα** take care of its request
3. Repeats the whole process until the terminal condition that was mentioned previously is sufficed.
