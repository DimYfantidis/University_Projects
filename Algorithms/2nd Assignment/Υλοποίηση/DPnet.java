import java.io.*;
import java.lang.System;
import java.lang.StringBuilder;
import java.lang.Integer;


public class DPnet
{
    private static final String endLine = System.lineSeparator();

    private static class Cell    // Linear Matrix's cells
    {
        int val;

        public Cell() {
            val = 0;
        }

        public int value() {
            return val;
        }

        public void assign(int val2) {
            val = val2;
        }
    }

    private static class Matrix
    {
        private final int rows;
        private final int columns;
        private final Cell[][]data;


        public Matrix(final int rows, final int columns) throws IOException
        {
            if (rows <= 0 || columns <= 0) {
                throw new IOException("Dimensions must be positive numbers");
            }

            this.rows = rows;
            this.columns = columns;
            data = new Cell[rows][columns];

            int i;
            int j;

            for (i = 0; i < rows; ++i) {
                for (j = 0; j < columns; ++j) {
                    data[i][j] = new Cell();
                }
            }
        }

        public Matrix(final Matrix other)
        {
            this.rows = other.rows;
            this.columns = other.columns;
            data = new Cell[rows][columns];

            int i;
            int j;

            for (i = 0; i < rows; ++i) {
                for (j = 0; j < columns; ++j) {
                    data[i][j] = new Cell();
                    data[i][j].assign(other.cell(i, j).value());
                }
            }
        }

        public int getRows() {
            return rows;
        }
        public int getColumns() {
            return columns;
        }

        public Cell cell(int i, int j) {
            return data[i][j];
        }

        @Override
        public String toString()
        {
            StringBuilder S = new StringBuilder();

            int i;
            int j;

            for (i = 0; i < rows; ++i) {
                for (j = 0; j < columns; ++j) {
                    S.append(this.cell(i, j).value()).append(" ");
                }
                S.append(endLine);
            }
            return S.toString();
        }

        /**
         *  <p>
         *      Initializes the linear matrix with input values from a file.
         *      Note that the function starts reading the data from the previous
         *      point of the cursor, not from the beginning of the file.
         *  </p>
         *  <p>
         *      For example, a 3x4 matrix to be initialized from file "matrix.txt",
         *      the file (from the last position of the cursor and on) must have the
         *      following structure:
         *  </p>
         *  <blockquote><pre>
         *  matrix.txt:
         *
         *      6 8 21
         *      10 3 5
         *      7 12 78
         *      21 34 214
         *  </pre></blockquote>
         *
         *  @param in the buffered reader that has already opened the file.
         *  @throws IOException thrown when the file is not written in the expected manner.
         */
        public void readFromFile(final BufferedReader in) throws IOException
        {
            int i;
            int j;

            String[] rowStr;
            int[] row = new int[columns];

            for (i = 0; i < rows; ++i)
            {
                rowStr = in.readLine().split(" ");

                if (rowStr[0].equals(""))
                    throw new IOException("Expected " + rows + " rows, found only " + i);

                if (rowStr.length != columns)
                    throw new IOException(
                            "Expected " + columns + " elements in row #" + (i + 1) + ", found " + rowStr.length
                    );

                for (j = 0; j < columns; ++j)
                {
                    row[j] = Integer.parseInt(rowStr[j]);
                    this.cell(i, j).assign(row[j]);
                }
            }
        }
    }

    /**
     *  <p>
     *      Computes the matrix with the least total costs of each task in each virtual machine,
     *      given the costs of the execution of a single task in every VM (A: NxM) and the communication
     *      costs for the transfer of a task from one VM to another. The implementation uses the technique
     *      of dynamic programming.
     *  </p>
     *  @param execution A: NxM matrix where A[i][j] is the cost of performing the task i alone in the system j.
     *  @param communication B: MxM matrix where B[i][j] is the cost of transferring a task from system i to system j.
     *  @return C: NxM matrix where C[i][j] is the least total cost of performing the i-th task in the j-th system
     *          after performing all i-1 tasks before it in any system.
     *  @throws IOException thrown when the input matrices don't have the appropriate defined dimensions.
     */
    public static Matrix totalCost(final Matrix execution, final Matrix communication) throws IOException
    {
        final int tasks = execution.getRows();
        final int VMs   = execution.getColumns();

        if (communication.getRows() != VMs || communication.getColumns() != VMs)
            throw new IOException(
                    "Dimension mismatch of input matrices, expected: NxM and MxM"
            );

        // The resulting matrix, initialized as a copy of the execution cost matrix as they have
        // the same dimensions and the same first row.
        Matrix C = new Matrix(execution);

        int i;
        int j;
        int k;

        // Stores the sum of:
        // -> the previous task's least total execution cost in the k-th VM.
        // -> the communication cost of the k-th VM with the given VM.
        // -> the cost of the given task's execution alone in the given VM.
        int sum;
        // The aforementioned procedure is repeated for every iteration of k and
        // the variable sum is stored to min before resetting to zero, only if it is
        // the least value that has yet been encountered.
        int min;

        for (i = 1; i < tasks; ++i)
        {
            for (j = 0; j < VMs; ++j)
            {
                // Minimum total execution cost of i-th task in the j-th VM is initialized to "infinity".
                min = Integer.MAX_VALUE;

                for (k = 0; k < VMs; ++k)
                {
                    // Reset the execution cost.
                    sum = 0;

                    sum += C.cell(i - 1, k).value();
                    sum += communication.cell(k, j).value();
                    sum += C.cell(i, j).value();

                    if (sum < min) {
                        min = sum;
                    }
                }
                // The least total achievable cost of the i-th task in the j-th system is stored in the resulting matrix.
                C.cell(i, j).assign(min);
            }
        }
        return C;
    }

    public static void main(String[] args)
    {
        int tasks;
        int VMs;
        
        if (args.length == 0) 
        {
            System.err.println("No command line arguements.");
            System.err.println("Please input file");
            return;
        }

        Matrix executionCost;
        Matrix communicationCost;
        
        // Data input through file (dimensions and matrices)
        try (BufferedReader in = new BufferedReader(new FileReader(args[0])))
        {
            // Number of tasks in the first line of the file (n)
            tasks   = Integer.parseInt(in.readLine().replaceAll("\\s", ""));
            // Number of virtual machines in the second line of the file (m)
            VMs     = Integer.parseInt(in.readLine().replaceAll("\\s", ""));

            // replaceAll() removes any spaces to avoid an exception

            // Construction of matrix with the execution costs of tasks (M1: n x m)
            executionCost       = new Matrix(tasks, VMs);
            // Construction of matrix with the communication costs between VMs (M2: m x m)
            communicationCost   = new Matrix(VMs, VMs);

            if (in.readLine().replaceAll("\\s", "").length() != 0)
                throw new IOException("3rd line of file \"" + args[0] + "\" must be empty");

            executionCost.readFromFile(in);

            if (in.readLine().replaceAll("\\s", "").length() != 0)
                throw new IOException("Expected newline between matrices in file \"" + args[0] + "\"");

            communicationCost.readFromFile(in);

            // Algorithm for computing the least total cost of each task in each VM (C: n x m) using Dynamic programming
            Matrix cost = totalCost(executionCost, communicationCost);

            System.out.print(cost);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
