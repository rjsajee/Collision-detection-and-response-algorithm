#include <gl/glut.h>        // OpenGL Utility Toolkit for rendering
#include "app.h"            // Base application class
#include "coreMath.h"       // Core mathematical functions and vector operations
#include "pcontacts.h"      // Particle contact resolution for collision handling
#include "pworld.h"         // Particle world managing physics and interactions
#include <vector>           // STL vector for dynamic array management
#include <cassert>          // Assertion library for debugging
#include <iostream>         // Standard I/O stream for debugging and logging


// Gravity force applied to all particles in the simulation
const Vector2 Vector2::GRAVITY = Vector2(0, -9.81);

// Defines the number of blobs and platforms in the simulation
#define BLOB_COUNT 50    
#define PLATFORM_COUNT 15  

// Represents a static platform where blobs can collide and bounce
class Platform : public ParticleContactGenerator
{
public:
    Vector2 start;  // Starting point of the platform
    Vector2 end;    // Ending point of the platform

    std::vector<Particle*> particles;  // Particles that interact with this platform

    // Detects and resolves collisions between particles and the platform
    unsigned addContact(ParticleContact* contact, unsigned limit) const override;
};

unsigned Platform::addContact(ParticleContact* contact, unsigned limit) const
{
    const static float restitution = 1.0f;  // Defines the bounciness of collisions
    unsigned used = 0;  // Counter for detected collisions

    for (Particle* particle : particles)
    {
        if (used >= limit) return used;  // Stop if contact limit is reached

        Vector2 toParticle = particle->getPosition() - start;
        Vector2 lineDirection = end - start;

        float projected = toParticle * lineDirection;
        float platformSqLength = lineDirection.squareMagnitude();
        float squareRadius = particle->getRadius() * particle->getRadius();

        // Check if the particle is near the platform's start point
        if (projected <= 0)
        {
            if (toParticle.squareMagnitude() < squareRadius)  // Collision detected
            {
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = particle;
                contact->particle[1] = nullptr;
                contact->penetration = particle->getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }
        }
        // Check if the particle is near the platform's end point
        else if (projected >= platformSqLength)
        {
            toParticle = particle->getPosition() - end;
            if (toParticle.squareMagnitude() < squareRadius)  // Collision detected
            {
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = particle;
                contact->particle[1] = nullptr;
                contact->penetration = particle->getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }
        }
        // Check if the particle is between the start and end points
        else
        {
            float distanceToPlatform = toParticle.squareMagnitude() - projected * projected / platformSqLength;
            if (distanceToPlatform < squareRadius)  // Collision detected
            {
                Vector2 closestPoint = start + lineDirection * (projected / platformSqLength);
                contact->contactNormal = (particle->getPosition() - closestPoint).unit();
                contact->restitution = restitution;
                contact->particle[0] = particle;
                contact->particle[1] = nullptr;
                contact->penetration = particle->getRadius() - sqrt(distanceToPlatform);
                used++;
                contact++;
            }
        }
    }
    return used;  // Return the number of detected collisions
}


class BlobDemo : public Application
{
    Particle* blobs[BLOB_COUNT];   // Array of blobs (particles) in the simulation
    Platform* platforms;           // Array of platforms for collision detection
    ParticleWorld world;           // Manages physics updates for particles

private:
    float totalPhysicsTime = 0.0f; // Tracks total simulation time

public:
    BlobDemo();    // Constructor to initialize blobs, platforms, and physics
    virtual ~BlobDemo(); // Destructor to clean up allocated memory

    virtual const char* getTitle();  // Returns the title of the simulation window
    virtual void display();          // Handles rendering of objects in OpenGL
    virtual void update();           // Updates physics and animation per frame
    void handleBlobCollision();      // Detects and resolves blob-to-blob collisions
    void drawBlobConnections();      // Draws lines between nearby blobs
    void countBlobsInGrid();         // Counts blobs in different quadrants
};

// Method definitions
BlobDemo::BlobDemo() : world(PLATFORM_COUNT + BLOB_COUNT, PLATFORM_COUNT)
{
    width = 400;
    height = 400;
    nRange = 100.0;

    float margin = 0.95f;

    // Create the blobs with unique positions, velocities, and properties
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        blobs[i] = new Particle;
        blobs[i]->setPosition(-60.0 + (i % 5) * 40.0, 90.0 - (i / 5) * 30.0);
        blobs[i]->setRadius(3);
        blobs[i]->setVelocity(100.0, 200.0);  // Set initial velocity
        blobs[i]->setDamping(0.9);           // Apply damping to simulate friction
        blobs[i]->setAcceleration(Vector2::GRAVITY * 5.0f * ((i % 5) + 1));
        blobs[i]->setMass(100.0f);
        blobs[i]->clearAccumulator();        // Reset forces applied to the blob
        world.getParticles().push_back(blobs[i]);
    }

    // Create platforms (static boundaries)
    platforms = new Platform[PLATFORM_COUNT];

    // Vertical platforms
    platforms[0].start = Vector2(0.0, 0.0);
    platforms[0].end = Vector2(0.0, -50.0);

    platforms[1].start = Vector2(-nRange * margin, -nRange * margin);
    platforms[1].end = Vector2(-nRange * margin, nRange * margin);

    platforms[2].start = Vector2(nRange * margin, -nRange * margin);
    platforms[2].end = Vector2(nRange * margin, nRange * margin);

    // Horizontal platforms
    platforms[3].start = Vector2(-nRange * margin, -nRange * margin);
    platforms[3].end = Vector2(nRange * margin, -nRange * margin);

    platforms[4].start = Vector2(-nRange * margin, nRange * margin);
    platforms[4].end = Vector2(nRange * margin, nRange * margin);

    // Additional diagonal and vertical platforms for more interaction
    platforms[5].start = Vector2(-50.0, 50.0);
    platforms[5].end = Vector2(0.0, 0.0);

    platforms[6].start = Vector2(50.0, 50.0);
    platforms[6].end = Vector2(0.0, 0.0);

    platforms[7].start = Vector2(-50.0, -50.0);
    platforms[7].end = Vector2(0.0, 0.0);

    platforms[8].start = Vector2(50.0, -50.0);
    platforms[8].end = Vector2(0.0, 0.0);

    // Additional vertical platforms in the middle
    platforms[9].start = Vector2(-30.0, -nRange * margin);
    platforms[9].end = Vector2(-30.0, nRange * margin);

    platforms[10].start = Vector2(30.0, -nRange * margin);
    platforms[10].end = Vector2(30.0, nRange * margin);

    // Additional horizontal platforms in the middle
    platforms[11].start = Vector2(-nRange * margin, -30.0);
    platforms[11].end = Vector2(nRange * margin, -30.0);

    platforms[12].start = Vector2(-nRange * margin, 30.0);
    platforms[12].end = Vector2(nRange * margin, 30.0);

    // Diagonal platforms to create more dynamic interactions
    platforms[13].start = Vector2(-nRange * margin, -nRange * margin);
    platforms[13].end = Vector2(nRange * margin, nRange * margin);

    platforms[14].start = Vector2(-nRange * margin, nRange * margin);
    platforms[14].end = Vector2(nRange * margin, -nRange * margin);

    // Assign blobs to platforms for collision detection
    for (unsigned i = 0; i < PLATFORM_COUNT; i++)
    {
        for (unsigned j = 0; j < BLOB_COUNT; j++) {
            platforms[i].particles.push_back(blobs[j]);
        }
        world.getContactGenerators().push_back(&platforms[i]);
    }
}

void BlobDemo::display()
{
    Application::display();
    drawBlobConnections(); // Draws lines between blobs based on proximity

    // Render the platforms (grid lines)
    glBegin(GL_LINES);
    glColor3f(0, 1, 1); // Set platform color to cyan

    for (unsigned i = 0; i < PLATFORM_COUNT; i++)
    {
        const Vector2& p0 = platforms[i].start;
        const Vector2& p1 = platforms[i].end;
        glVertex2f(p0.x, p0.y);
        glVertex2f(p1.x, p1.y);
    }

    // Render reference grid lines
    glColor3f(1, 1, 1);
    glVertex2f(-100, 0); glVertex2f(100, 0);  // Center horizontal line
    glVertex2f(0, -100); glVertex2f(0, 100);  // Center vertical line
    glEnd();

    // Render blobs with different colors
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        switch (i % 10)  // Assigning different colors to each blob
        {
        case 0: glColor3f(1.0f, 0.0f, 0.0f); break; // Red
        case 1: glColor3f(0.0f, 1.0f, 0.0f); break; // Green
        case 2: glColor3f(0.0f, 0.0f, 1.0f); break; // Blue
        case 3: glColor3f(1.0f, 1.0f, 0.0f); break; // Yellow
        case 4: glColor3f(1.0f, 0.0f, 1.0f); break; // Magenta
        case 5: glColor3f(0.0f, 1.0f, 1.0f); break; // Cyan
        case 6: glColor3f(1.0f, 0.5f, 0.0f); break; // Orange
        case 7: glColor3f(0.5f, 0.0f, 1.0f); break; // Purple
        case 8: glColor3f(1.0f, 0.5f, 0.5f); break; // Pink
        case 9: glColor3f(0.5f, 1.0f, 0.5f); break; // Light Green
        }

        const Vector2& p = blobs[i]->getPosition();

        glPushMatrix();
        glTranslatef(p.x, p.y, 0); // Move blob to its position
        glutSolidSphere(blobs[i]->getRadius(), 12, 12); // Draw as sphere
        glPopMatrix();
    }

    glutSwapBuffers(); // Swap buffers to render the updated frame
}




void BlobDemo::handleBlobCollision()
{
    // Loop through all blobs to check for collisions
    for (unsigned i = 0; i < BLOB_COUNT - 1; i++) {
        for (unsigned j = i + 1; j < BLOB_COUNT; j++) {

            // Compute the vector between two blobs
            Vector2 distanceVec = blobs[j]->getPosition() - blobs[i]->getPosition();
            float distance = distanceVec.magnitude();
            float combinedRadius = blobs[i]->getRadius() + blobs[j]->getRadius();

            // Check if blobs are colliding
            if (distance < combinedRadius) {
                Vector2 normal = distanceVec.unit(); // Normalize direction of collision
                Vector2 relativeVelocity = blobs[j]->getVelocity() - blobs[i]->getVelocity();
                float velocityAlongNormal = relativeVelocity * normal;

                // Skip if blobs are moving apart
                if (velocityAlongNormal > 0) continue;

                float e = 1.0f; // Perfectly elastic collision (coefficient of restitution)
                float m1 = blobs[i]->getMass();
                float m2 = blobs[j]->getMass();

                // Calculate impulse magnitude
                float impulse = (-(1 + e) * velocityAlongNormal) / (1 / m1 + 1 / m2);
                Vector2 impulseVec = normal * impulse;

                // Apply impulse to adjust velocities after collision
                blobs[i]->setVelocity(blobs[i]->getVelocity() - (impulseVec * (1.0f / m1)));
                blobs[j]->setVelocity(blobs[j]->getVelocity() + (impulseVec * (1.0f / m2)));
            }
        }
    }
}


void BlobDemo::drawBlobConnections()
{
    glColor3f(1, 1, 1); // Set color to white for the connection lines
    glBegin(GL_LINES);   // Start drawing lines

    // Iterate through all blobs to check for nearby connections
    for (unsigned i = 0; i < BLOB_COUNT - 1; i++)
    {
        for (unsigned j = i + 1; j < BLOB_COUNT; j++)
        {
            Vector2 pos1 = blobs[i]->getPosition();
            Vector2 pos2 = blobs[j]->getPosition();

            float distance = (pos2 - pos1).magnitude();

            // Draw a line if blobs are within a certain distance threshold
            if (distance < 80.0f)
            {
                glVertex2f(pos1.x, pos1.y);
                glVertex2f(pos2.x, pos2.y);
            }
        }
    }

    glEnd(); // End drawing lines
}



void BlobDemo::countBlobsInGrid()
{
    int topLeft = 0, topRight = 0, bottomLeft = 0, bottomRight = 0; // Initialize quadrant counters

    // Iterate through all blobs to determine their quadrant
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        Vector2 pos = blobs[i]->getPosition(); // Get blob position

        // Categorize blob into the correct quadrant
        if (pos.x < 0 && pos.y > 0) topLeft++;      // Upper-left quadrant
        else if (pos.x > 0 && pos.y > 0) topRight++; // Upper-right quadrant
        else if (pos.x < 0 && pos.y < 0) bottomLeft++; // Lower-left quadrant
        else if (pos.x > 0 && pos.y < 0) bottomRight++; // Lower-right quadrant
    }

    // Output the count of blobs in each quadrant
    std::cout << "Quadrant Counts: "
        << "(TL: TR: BL: BR: "
        << topLeft << ", "
        << topRight << ", "
        << bottomLeft << ", "
        << bottomRight << ")" << std::endl;
}

BlobDemo::~BlobDemo()
{
    // Delete all dynamically allocated blobs to free memory
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        delete blobs[i];
    }

    // Delete dynamically allocated platform array
    delete[] platforms;
}


void BlobDemo::update()
{
    float duration = timeinterval / 1000;  // Convert time interval from milliseconds to seconds

    totalPhysicsTime += duration;  // Keep track of total simulation time

    // Display the running physics time in the console for debugging
    std::cout << "Total Running Physics Time: " << totalPhysicsTime << " seconds" << std::endl;

    world.runPhysics(duration);   // Execute physics simulation for all particles
    handleBlobCollision();        // Detect and resolve collisions between blobs
    countBlobsInGrid();           // Count blobs in each quadrant and print results
    Application::update();        // Call base class update function for additional processing
    glutPostRedisplay();          // Request a screen refresh to update visuals
}


const char* BlobDemo::getTitle()
{
    return "Interactive Physics Simulation: Blobs & Collision Dynamics"; // Returns the application title
}

Application* getApplication()
{
    return new BlobDemo(); // Creates and returns an instance of BlobDemo
}

