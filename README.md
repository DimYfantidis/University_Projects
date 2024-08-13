# University Projects
An exhaustive list with every programming-related school assignment.

# 1st Semester
* ## Basic Programming Principles (C)
  The lesson's target is the teaching of the C programming language. The program's purpose is to test the student's knowledge of the language of the C language regarding dynamic memory allocation, keyboard I/O, file I/O, strings of characters and functions. The assignment also tests the student's competence in problem solving. Several metric functions regarding the lucky numbers game of the national lottery were implemented.
<br/>

# 2nd Semester
* ## Data Structures (C++)
  The lesson's target is the teaching of basic data structures. Several data structures containing alphanumerics were implemented such as Dynamic Unordered array, Dynamic Ordered array, Binary Search Tree, AVL Tree, Hash Table (Unordered Map). 
<br/>


# 3rd Semester
* ## Numerical analysis (C++)
  The lesson's target is the teaching of solving continuous problems using methods for numeric approximation. **Seven different problems** were requested to be solved. A plethora of numeric methods were implemented: Dichotomy method, Newton-Raphson method, Secant method, $PA = LU$ decomposition, Cholesky decomposition, Gaus-Seidel method, a simplified version of Google's **pagerank** algorithm, Polynomial approximation, Spline interpolation, Least Squares method, Simpson's rule and Trapezoidal rule.
* ## Object Oriented Programming (Java)
  The lesson's target is the teaching of the Java programming language as well as certain of OOP's principles. The assignment requests the implementation of a simple "travel guide" program, similar to Airbnb and booking.com with pseudo-network programming and pseudo-accounts (the entire java program is offline). The pseudo-platform consists of Customer accounts, Admin accounts and Accomodation Provider accounts. The program utilises different types of accommodations, booking mechanics, timing/date mechanics etc. and disposes a GUI implemented with the **Java Swing Library**.
<br/>


# 4th Semester
* ## Algorithms (Java)
  The lesson's target is broaden the students' knowledge on designing and analyzing algorithms. The 1st assignment requests to implement an algorithm of $\mathcal{O}(N logN)$ time complexity for the **skyline problem** of $N$ datapoints. The 2nd assignment requests the implementation of a dynamic-programming algorithm for the solution of **Dynamic Virtual Machine Placement in Cloud Computing**.
* ## Artificial Intelligence (C++)
  The lesson's target is to introduce the students to the basic ideas of AI (Search algorithms, Genetic algorithms, Knowledge representation etc.). The assignment demands the modeling of **Water Sort Game**'s states and the transitions from one state to another based on the game's rules. Subsequently, an implementation of **Breadth-First Search** algorithm is requested to find the optimal route of transitions from a random initial state to a winning state.
* ## Digital Communications (Java)
  The lesson's target is to introduce the stundents to basic principles of Telecommunications. The assignment requests to implement a program that simulates the procedures of sending and receiving messages through a noisy communication channel. Each message sent has a chance to be affected by the noise (bit flips) and when received **error correction** is applied to it through Cyclic Redundancy Check and modulo-2 arithmetic.
<br/>


# 5th Semester 
* ## Computer Networks (Java)
  The lesson's target is to introduce the students to internet protocols (TCP, HTTP etc.) and network programming. The assignment requests the implementation of a program/service that consists of a **Server** side and a **Client** side using sockets or Remote Method Invocation in Java. The two sides of the service can be executed in different computers and interact with each other through access to the internet.
* ## Databases (SQL)
  The lesson's target is to introduce the students to Relational Algebra and SQL. The assignment requests the design of a database with an ER diagram, based on the **Entity-Relationship Model**. Specific conversions are then applied to the ER diagram to generate the equivalent relational model. Finally, the resulting relational model is written in SQL.
* ## Operating Systems (C)
  The lesson's target is to introduce the students to the foundations of operating systems design (time-scheduling algorithms, disk-scheduling algorithms, memory management, interrupts, semaphores) and Linux bash. The assignment demands the implementation of **4 time-scheduling algorithms**: First Come First Served (FCFS), Shortest Job First (SJF), Shortest Remaining Time First (SRTF) and Round-robin (RR).
* ## Systems and Signals (MATLAB)
  The lesson's target is to introduce the students to linear time-invariant and periodic signals, as well as Fourier, Zeta and Laplace transforms; both in their continuous and discrete nature. The assignment requests the implementation of **overlapp-add** and **overlap-save** methods to apply band-pass filtering to digital media.
<br/>


# 6th Semester
* ## Computer Graphics (C)
  The lesson's goal is to introduce students to the field of Computer Graphics and its mathematical foundations, such as Analytic Geometry in 3D space and Numerical Methods. Furthermore, the basics of the OpenGL library are also included in the course, so that students can put the course material into practice using low-level programming. The **three coding assignments** were implemented in the C programming language using [freeglut](https://freeglut.sourceforge.net/).
    1. The first assignment involves opening a window and drawing a 2D fractal shape (Sierpinski's Triangle) using simple OpenGL/GLU/GLUT functions.
    2. The second one involves implementing GLUT callback functions, GLUT window menus/submenus, display lists and affine transformations on a 3D cube.
    3. The final assignment demands the implementation of a simple, low-poly 3D environment with dynamic camera movement, camera perspective, lighting (Phong model), different types of light sources, recursive subdivision of spheres and basic shadow mapping (projected shadows).
* ## Foundations of Cryptography (Python)
  The lesson's target is to introduce students to the foundations of cryptography, its heavy mathematical background (Number Theory) and its clever way of thinking for understanding and designing cryptographic systems. **Two assignments** were given that demand the solution of 25 exercises in total. Most of the exercises are related to Number Theory such as proving mathematical formulas, finding prime numbers, finding the corresponding decryption functions, $f^{-1}$, for given encryption functions, $f$, etc. Other exercises demand the implementation of cryptosystems such as OTP and RC4, or testing their robustness (i.e. their snowball effect) such as AES-128 in ECB mode or AES-128 in CBC mode. Certain exercises are fun CTF-like brain teasers.  

  **Bonus:** A presentation on the topic of *Obfuscation Methods* can also be found in the relevant directory.

* ## Machine Learning (Python)
  The lesson's goal is to introduce students to Machine Learning in the context of Data Analysis. **Three assignments** were given in the form of Interactive Python Notebooks (Headers + report). In all three assignments, the student experiments with different types of ML models and reports the requested results. Also, each assignment includes small open-ended questions to test theory comprehension. Assignment breakdown:
  1. The first assignment involves a revision of the NumPy & Pandas libraries, implementing & training Linear Models (Linear Regression & Logistic Regression), plotting and analyzing results/data, data regularization.
  2. The second assignment involves the teaching of the Scikit-Learn library, its Decision Tree Classifiers and Pipelines, evaluation metrics, model visualization, hyperparameter tuning.
  3. The final assignment involves using and testing ensemble models such as voting classifiers, AdaBoost, Random Forest, Bagging and Microsoft's XG-Boost.
<br/>


# 7th Semester
* ## Concurrency and Security in Software (C)
  The lesson's target is to help students design concurrent algorithms and write concurrent code in different languages, thus extending their programming knowledge and capabilities. Theoretical concepts of Concurrency involve the critical sector's problem, mutual exclusion, semaphores, monitors, time scheduling for multiple processes and distributed systems design. These are all taught by studying concurrent algorithms such as Lamport's bakery algorithm, the Byzantine Generals problem, etc. in a manner that is not language specific (pseudocode). An optional programming assignment was given in which students could choose a problem from the book () and implement its solution in the programming language of their preference.  

  Personally, I chose the problem of the concurrent computation of the binomial coefficient by Zohar Manna and Amir Pnueli. In my implementation I used the **C programming language** and the **pthreads library**.
* ## Microprocessors (x86 Assembly)
* ## Modeling - Digital Image Synthesis (C)
* ## Neural Networks - Deep Learning (Python)
<br/>


# 8th Semester
* ## Robotic Control (Python)
* ## Digital Signal Processing (MATLAB)
* ## Computational Intelligence - Deep Reinforcement Learning (Python)
