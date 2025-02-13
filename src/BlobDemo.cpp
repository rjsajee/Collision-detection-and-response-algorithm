#include <gl/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include <vector>
#include <cassert>
#include <iostream>

const Vector2 Vector2::GRAVITY = Vector2(0, -9.81);

#define BLOB_COUNT 100     // Increase to 100 blobs
#define PLATFORM_COUNT 9  // Nine platforms

class Platform : public ParticleContactGenerator
{
public:
    Vector2 start;
    Vector2 end;
    std::vector<Particle*> particles;

    unsigned addContact(ParticleContact* contact, unsigned limit) const override;
};

unsigned Platform::addContact(ParticleContact* contact, unsigned limit) const
{
    const static float restitution = 1.0f;
    unsigned used = 0;

    for (Particle* particle : particles)
    {
        if (used >= limit) return used;

        Vector2 toParticle = particle->getPosition() - start;
        Vector2 lineDirection = end - start;

        float projected = toParticle * lineDirection;
        float platformSqLength = lineDirection.squareMagnitude();
        float squareRadius = particle->getRadius() * particle->getRadius();

        if (projected <= 0)
        {
            if (toParticle.squareMagnitude() < squareRadius)
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
        else if (projected >= platformSqLength)
        {
            toParticle = particle->getPosition() - end;
            if (toParticle.squareMagnitude() < squareRadius)
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
        else
        {
            float distanceToPlatform = toParticle.squareMagnitude() - projected * projected / platformSqLength;
            if (distanceToPlatform < squareRadius)
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
    return used;
}

class BlobDemo : public Application
{
    Particle* blobs[BLOB_COUNT];
    Platform* platforms;
    ParticleWorld world;

private:
    float totalPhysicsTime = 0.0f;

public:
    BlobDemo();
    virtual ~BlobDemo();

    virtual const char* getTitle();
    virtual void display();
    virtual void update();
    void handleBlobCollision();
    void drawBlobConnections();
    void countBlobsInGrid();  // Function to count blobs in each quadrant
};

// Method definitions
BlobDemo::BlobDemo() : world(PLATFORM_COUNT + BLOB_COUNT, PLATFORM_COUNT)
{
    width = 400;
    height = 400;
    nRange = 100.0;

    float margin = 0.95f;

    // Create the blobs
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        blobs[i] = new Particle;
        blobs[i]->setPosition(-60.0 + (i % 5) * 40.0, 90.0 - (i / 5) * 30.0);
        blobs[i]->setRadius(3);
        blobs[i]->setVelocity(100.0,200.0);
        blobs[i]->setDamping(0.9);
        blobs[i]->setAcceleration(Vector2::GRAVITY * 5.0f * ((i % 5) + 1));
        blobs[i]->setMass(100.0f);
        blobs[i]->clearAccumulator();
        world.getParticles().push_back(blobs[i]);
    }

    // Create platforms
    platforms = new Platform[PLATFORM_COUNT];

    platforms[0].start = Vector2(0.0, 0.0);
    platforms[0].end = Vector2(0.0, -50.0);

    platforms[1].start = Vector2(-nRange * margin, -nRange * margin);
    platforms[1].end = Vector2(-nRange * margin, nRange * margin);

    platforms[2].start = Vector2(-nRange * margin, -nRange * margin);
    platforms[2].end = Vector2(nRange * margin, -nRange * margin);

    platforms[3].start = Vector2(nRange * margin, -nRange * margin);
    platforms[3].end = Vector2(nRange * margin, nRange * margin);

    platforms[4].start = Vector2(-nRange * margin, nRange * margin);
    platforms[4].end = Vector2(nRange * margin, nRange * margin);

    platforms[5].start = Vector2(-50.0, 50.0);
    platforms[5].end = Vector2(0.0, 0.0);

    platforms[6].start = Vector2(50.0, 50.0);
    platforms[6].end = Vector2(0.0, 0.0);

    platforms[7].start = Vector2(-50.0, -50.0);
    platforms[7].end = Vector2(0.0, 0.0);

    platforms[8].start = Vector2(50.0, -50.0);
    platforms[8].end = Vector2(0.0, 0.0);

    // Assign blobs to platforms
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
    drawBlobConnections();
    // Render the platforms (grid lines)
    glBegin(GL_LINES);
    glColor3f(0, 1, 1);
    for (unsigned i = 0; i < PLATFORM_COUNT; i++)
    {
        const Vector2& p0 = platforms[i].start;
        const Vector2& p1 = platforms[i].end;
        glVertex2f(p0.x, p0.y);
        glVertex2f(p1.x, p1.y);
    }

    // Render grid lines for reference
    glColor3f(1, 1, 1);
    glVertex2f(-100, 0); glVertex2f(100, 0);  // Horizontal Line
    glVertex2f(0, -100); glVertex2f(0, 100);  // Vertical Line
    glEnd();

    // Render the blobs with bright, unique colors
    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        switch (i % 10)  // Assigning different bright colors to each blob
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
        case 10: glColor3f(0.5f, 1.0f, 0.5f); break; // Light Green
        }

        const Vector2& p = blobs[i]->getPosition();
        glPushMatrix();
        glTranslatef(p.x, p.y, 0);
        glutSolidSphere(blobs[i]->getRadius(), 12, 12);
        glPopMatrix();
    }

    glutSwapBuffers();
}



void BlobDemo::handleBlobCollision()
{
    for (unsigned i = 0; i < BLOB_COUNT - 1; i++) {
        for (unsigned j = i + 1; j < BLOB_COUNT; j++) {
            Vector2 distanceVec = blobs[j]->getPosition() - blobs[i]->getPosition();
            float distance = distanceVec.magnitude();
            float combinedRadius = blobs[i]->getRadius() + blobs[j]->getRadius();

            if (distance < combinedRadius) {  // Collision detected
                Vector2 normal = distanceVec.unit();
                Vector2 relativeVelocity = blobs[j]->getVelocity() - blobs[i]->getVelocity();
                float velocityAlongNormal = relativeVelocity * normal;

                if (velocityAlongNormal > 0) continue;

                float e = 1.0f; // Coefficient of restitution
                float m1 = blobs[i]->getMass();
                float m2 = blobs[j]->getMass();

                float impulse = (-(1 + e) * velocityAlongNormal) / (1 / m1 + 1 / m2);
                Vector2 impulseVec = normal * impulse;

                blobs[i]->setVelocity(blobs[i]->getVelocity() - (impulseVec * (1.0f / m1)));
                blobs[j]->setVelocity(blobs[j]->getVelocity() + (impulseVec * (1.0f / m2)));
            }
        }
    }
}

void BlobDemo::drawBlobConnections()
{
    glColor3f(1, 1, 1);
    glBegin(GL_LINES);

    for (unsigned i = 0; i < BLOB_COUNT - 1; i++)
    {
        for (unsigned j = i + 1; j < BLOB_COUNT; j++)
        {
            Vector2 pos1 = blobs[i]->getPosition();
            Vector2 pos2 = blobs[j]->getPosition();

            float distance = (pos2 - pos1).magnitude();

            if (distance < 80.0f)
            {
                glVertex2f(pos1.x, pos1.y);
                glVertex2f(pos2.x, pos2.y);
            }
        }
    }

    glEnd();
}


void BlobDemo::countBlobsInGrid()
{
    int topLeft = 0, topRight = 0, bottomLeft = 0, bottomRight = 0;

    for (unsigned i = 0; i < BLOB_COUNT; i++)
    {
        Vector2 pos = blobs[i]->getPosition();

        if (pos.x < 0 && pos.y > 0) topLeft++;
        else if (pos.x > 0 && pos.y > 0) topRight++;
        else if (pos.x < 0 && pos.y < 0) bottomLeft++;
        else if (pos.x > 0 && pos.y < 0) bottomRight++;
    }

    // **Print the count of blobs in each quadrant in the console**
    std::cout << "Quadrant Counts: "
        << "(TL: TR: BL: BR:" << topLeft << "," << topRight << "," << bottomLeft << "," << bottomRight << ")" << std::endl;
}

BlobDemo::~BlobDemo()
{
    for (unsigned i = 0; i < BLOB_COUNT; i++) {
        delete blobs[i];
    }
    delete[] platforms;
}

void BlobDemo::update()
{
    float duration = timeinterval / 1000;  // Convert milliseconds to seconds

    totalPhysicsTime += duration;  // Accumulate the total time

    // Print the running physics time in the console
    std::cout << "Total Running Physics Time: " << totalPhysicsTime << " seconds" << std::endl;

    world.runPhysics(duration);
    handleBlobCollision();
    countBlobsInGrid();  // Print counts in console
    Application::update();

    glutPostRedisplay(); // Ensure display updates continuously
}

const char* BlobDemo::getTitle()
{
    return "Blob Demo with Grid and Blob Count";
}

Application* getApplication()
{
    return new BlobDemo();
}
