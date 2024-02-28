package MessagingApp.Server;


/**
 *  <p>
 *      Instances of the following class represent records of messages sent
 *      between the users. The <B>{@link Message.Builder Builder} design pattern</B>
 *      is implemented for creating instances, as {@link Message} contains many
 *      fields and most of them be declared <b>final</b>.
 *  </p>
 */
public class Message
{
    /** States whether the message has been acknowledged by its receiver. */
    private boolean isRead;

    /** States whether the receiver has deleted this message. */
    private boolean isDeleted;

    /**
     *  <p>
     *      Message ID is unique for each message in the user's inbox but not unique for
     *      every instance. In the current implementation, it is synonymous with its position in
     *      the user's {@link Account#getMessageBox() message box}.
     *  </p>
     *  @see Server.ConnectionHandler#sendMessage()
     */
    private final int messageId;

    /** The message's transmitting user. */
    private final String sender;

    /** The message's recipient. */
    private final String receiver;

    /** Message text */
    private final String body;


    public static class Builder
    {
        private int messageId;

        private String sender;

        private String receiver;

        private String body;

        public Builder messageId(int val) { messageId = val; return this; }

        public Builder sender(String val) { sender = val; return this; }

        public Builder receiver(String val) { receiver = val; return this; }

        public Builder body(String val) { body = val; return this; }

        public Message build() { return new Message(this); }
    }

    private Message(Builder builder)
    {
        isRead = false;
        isDeleted = false;
        messageId = builder.messageId;
        sender = builder.sender;
        receiver = builder.receiver;
        body = builder.body;
    }

    // ------------ Getter functions (BEGIN) ------------------ //

    public String text() { return body; }

    public String sender() { return sender; }

    public int id() { return messageId; }

    public boolean isRead() { return isRead; }

    public boolean isDeleted() { return isDeleted; }

    // ------------ Getter functions (END) ------------------ //


    /** Flips message's state to <b>read</b>. */
    public void read() { isRead = true; }
    
    /** Flips message's state to <b>deleted</b>. */
    public void delete() { isDeleted = true; }
}
