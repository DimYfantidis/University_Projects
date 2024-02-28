#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <GL/glut.h>

#define WHITE	1.0f, 1.0f, 1.0f
#define RED		1.0f, 0.0f, 0.0f
#define YELLOW	1.0f, 1.0f, 0.0f
#define GREEN	0.0f, 1.0f, 0.0f
#define BLUE	0.0f, 0.0f, 1.0f
#define CYAN	0.0f, 1.0f, 1.0f
#define MAGENTA	1.0f, 0.0f, 1.0f

#define BASE_SCALING_VEL	0.0002f
#define BASE_ANGULAR_VEL	0.125f


typedef GLfloat vector3f[3];

// Rotation Vector
static const vector3f V = { 1.0f, 2.0f, 2.0f };

/* Let P(x, y, z) be a plane which is defined by V
 * (perpendicular vector) and the origin (0, 0, 0).
 * Then, W is a vector that is perpendicular to V, is
 * istuated on plane P(x, y, z) and has a length of r.  */
static vector3f W;

// Cube's edge length
static const GLfloat a = 5.0f;
// Cube's center at (0, 0, -b)
static const GLfloat b = 70.0f;

static const GLfloat r = 20.0f;

// Dynamic scaling factor
static GLfloat s = 1.0;

static GLfloat angle = 0.0f;

static GLfloat matrixMV[16];

// Accelerators
static GLfloat scale_vel = BASE_SCALING_VEL;
static GLfloat angular_vel = BASE_ANGULAR_VEL;

bool AROUND_AXIS = false;
bool ENABLE_AXIS = false;


void display(void)
{
	// Observation space correction constant
	GLfloat c = (AROUND_AXIS ? 8.0f : 4.0f);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	// glOrtho placement and update

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-a * c, a * c, -a * c, a * c, b - a * c * 2, b + a * c * 2);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	if (AROUND_AXIS)
		glTranslatef(0.0f, 0.0f, -0.8f * b);
	else
		glTranslatef(0.0f, 0.0f, -b);

	// Rotation Axis
	if (ENABLE_AXIS) {
		glColor3f(WHITE);
		glCallList(2);

		// Radius Display
		glColor3f(CYAN);
		glBegin(GL_LINES);
		{
			glVertex3f(V[0], V[1], V[2]);
			glVertex3f(matrixMV[12], matrixMV[13], matrixMV[14]);
		}
		glEnd();
	}

	// Rotation using the V vector
	glRotatef(angle, V[0], V[1], V[2]);

	// If selected, rotaton around a certain point on teh vector with a giver r radius
	if (AROUND_AXIS)
		glTranslatef(W[0], W[1], W[2]);

	// By scaling with a/2, the cube's edge becomes a
	glScalef(s * (a / 2.0f), s * (a / 2.0f), s * (a / 2.0f));


	// Creation and Drawing of the cube using an initial plane
	glPushMatrix();

	// Front face
	glColor3f(RED);
	glCallList(1);

	// Right face
	glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
	glColor3f(GREEN);
	glCallList(1);

	glPopMatrix();
	glPushMatrix();

	// Left face
	glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
	glColor3f(BLUE);
	glCallList(1);

	glPopMatrix();
	glPushMatrix();

	// Back face
	glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
	glColor3f(YELLOW);
	glCallList(1);

	glPopMatrix();
	glPushMatrix();

	// Upper face
	glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
	glColor3f(CYAN);
	glCallList(1);

	glPopMatrix();
	glPushMatrix();

	// Lower face
	glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
	glColor3f(MAGENTA);
	glCallList(1);

	glPopMatrix();

	// Store current location of the cube (relative)

	glGetFloatv(GL_MODELVIEW_MATRIX, matrixMV);

	glutSwapBuffers();
}

void idle(void)
{
	// Signals if the cube grows bigger or smaller
	static bool scale_down = false;

	//Update angle
	if (angle >= 360.0f) {
		angle -= 360.0f;
	}
	angle += angular_vel;

	//Update scale
	if (s >= 2.0f) {
		scale_down = true;
	}
	else if (s <= 1.0f) {
		scale_down = false;
	}
	s += (scale_down ? -scale_vel : scale_vel);

	glutPostRedisplay();
}

//Secial Keyboard button handler
void SpecialKeyHandler(int key, int x, int y) {
	//Increase speed of rotation and scaling
	//Right Arrow = Increase rotation speed
	//Left Arrow = Decrease rotation speed
	//Up Arrow = Increase scale speed
	//Down Arrow = Decrease scale speed
	if (key == GLUT_KEY_RIGHT)
	{
		if (angular_vel * 2.0f <= BASE_ANGULAR_VEL * 64.0f) {
			angular_vel *= 2.0f;
			printf("ANGULAR velocity updated:\t %.6f\n", angular_vel);
		}
	}
	if (key == GLUT_KEY_LEFT)
	{
		if (angular_vel / 2.0f >= BASE_ANGULAR_VEL / 16.0f) {
			angular_vel /= 2.0f;
			printf("ANGULAR velocity updated:\t %.6f\n", angular_vel);
		}
	}
	if (key == GLUT_KEY_UP)
	{
		if (scale_vel * 2.0f <= BASE_SCALING_VEL * 64.0f) {
			scale_vel *= 2.0f;
			printf("SCALING velocity updated:\t %.6f\n", scale_vel);
		}
	}
	if (key == GLUT_KEY_DOWN)
	{
		if (scale_vel / 2.0f > BASE_SCALING_VEL / 16.0f) {
			scale_vel /= 2.0f;
			printf("SCALING velocity updated:\t %.6f\n", scale_vel);
		}
	}
}

// Menu event handler
void menu(int op_id)
{
	switch (op_id)
	{
	case 1:
		AROUND_AXIS = false;
		break;
	case 2:
		AROUND_AXIS = true;
		break;
	case 3:
		ENABLE_AXIS = !ENABLE_AXIS;
		printf("Axes: %s\n", (ENABLE_AXIS ? "Visible" : "Hidden"));
		break;
	case 4:
		exit(EXIT_SUCCESS);
	default:
		fprintf(stderr, "Undefined Input\n");
	}
}


void init_lists(void)
{
	// Square face
	glNewList(1, GL_COMPILE);
	{
		glBegin(GL_POLYGON);
		{
			glVertex3f(-1.0f, 1.0f, 1.0f);
			glVertex3f(1.0f, 1.0f, 1.0f);
			glVertex3f(1.0f, -1.0f, 1.0f);
			glVertex3f(-1.0f, -1.0f, 1.0f);
		}
		glEnd();
	}
	glEndList();

	// Utility
	glNewList(2, GL_COMPILE);
	{
		int lim = 8;

		// V vector
		glColor3f(WHITE);
		glBegin(GL_LINES);
		{
			glVertex3f(lim * a * V[0], lim * a * V[1], lim * a * V[2]);
			glVertex3f(-lim * a * V[0], -lim * a * V[1], -lim * a * V[2]);
		}
		glEnd();

		// X axis
		glColor3f(RED);
		glBegin(GL_LINES);
		{
			glVertex3f(lim * a, 0.0f, 0.0f);
			glVertex3f(-lim * a, 0.0f, 0.0f);
		}
		glEnd();

		// Y axis
		glColor3f(GREEN);
		glBegin(GL_LINES);
		{
			glVertex3f(0.0f, lim * a, 0.0f);
			glVertex3f(0.0f, -lim * a, 0.0f);
		}
		glEnd();

		// Z axis
		glColor3f(BLUE);
		glBegin(GL_LINES);
		{
			glVertex3f(0.0f, 0.0f, lim * a);
			glVertex3f(0.0f, 0.0f, -lim * a);
		}
		glEnd();

	}
	glEndList();

}

void init_vector(void)
{
	// Creation of the V, B and W vector

	GLfloat vx = V[0];
	GLfloat vy = V[1];
	GLfloat vz = V[2];

	if (vx == 0.0f && vy == 0.0f)
	{
		W[0] = 0.0f;
		W[1] = r;
		W[2] = 0.0f;
		return;
	}

	GLfloat bx = vz * vx;
	GLfloat by = vz * vy;
	GLfloat bz = -vx * vx - vy * vy;

	GLfloat k = r / (GLfloat)sqrt((double)(bx * bx + by * by + bz * bz));

	W[0] = k * bx;
	W[1] = k * by;
	W[2] = k * bz;
}


int main(int argc, char** argv)
{
	int menu_id;

	if (V[0] == 0.0f && V[1] == 0.0f && V[2] == 0.0f)
	{
		fprintf(stderr, "Invalid vector of rotation!\n");
		return EXIT_FAILURE;
	}

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(700, 700);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Project 2 - Spinning Cube (AEM1: 3938, AEM2: 3713)");

	// Attributes
	glEnable(GL_DEPTH_TEST);				// Depth Buffer
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);	// Black Background

	// Pre-compiled lists
	init_lists();

	// Necessary vector for rotation (W)
	init_vector();

	// Menu creation (BEGIN)
	menu_id = glutCreateMenu(menu);
	{
		glutAddMenuEntry("Rotate around cube's center", 1);
		glutAddMenuEntry("Rotate around axis", 2);
		glutAddMenuEntry("Toggle axis of rotaion", 3);
		glutAddMenuEntry("Quit", 4);
	}
	glutAttachMenu(GLUT_RIGHT_BUTTON);
	// Menu creation (END)

	glutSpecialFunc(SpecialKeyHandler);

	glutDisplayFunc(display);
	glutIdleFunc(idle);

	printf("PROMPT:\n");
	printf("-- Right/Left arrow keys:  Increase/decrease angular velocity\n");
	printf("-- Up/Down arrow keys:     Increase/decrease scaling velocity\n");
	printf("\n");

	glutMainLoop();

	return EXIT_SUCCESS;
}
