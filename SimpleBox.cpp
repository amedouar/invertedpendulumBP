/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <iostream>
#include "InvertedPendulum.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h" 
#include "../CommonInterfaces/CommonRigidBodyBase.h"

static btScalar kp = -104;
static btScalar kd = -3;


struct InvertedPendulumExample : public CommonRigidBodyBase
{
    btRigidBody *rb1;
    btRigidBody *rb2;
    btHingeConstraint *joint1;
    
    bool m_once;
    int m_frameCount;
    
    InvertedPendulumExample(struct GUIHelperInterface* helper);
	virtual ~InvertedPendulumExample(){}
	virtual void initPhysics();
    virtual void stepSimulation(float deltaTime);
    
	void resetCamera()
	{
		float dist = 30;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3]={0,0.46,0};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}
};

InvertedPendulumExample::InvertedPendulumExample(struct GUIHelperInterface* helper)
:CommonRigidBodyBase(helper),m_once(true),m_frameCount(0)
{
}

void InvertedPendulumExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);



	{
        m_dynamicsWorld->setGravity(btVector3(0,-9.8,0));
        btBoxShape* colShape = createBoxShape(btVector3(1,1,1));
		 
		m_collisionShapes.push_back(colShape);

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(0.f);

		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
            colShape->calculateLocalInertia(mass,localInertia);


		startTransform.setOrigin(btVector3(
								 btScalar(0),
								 btScalar(10),
								 btScalar(0)));
		rb1 = createRigidBody(mass,startTransform,colShape);
        
        btBoxShape* colShape2 = createBoxShape(btVector3(0.2,1.5,0.2));
        
        m_collisionShapes.push_back(colShape2);
        
        /// Create Dynamic Objects
        btTransform startTransform2;
        startTransform2.setIdentity();
        
        btScalar	mass2(1.f);
        bool isDynamic2 = (mass2 != 0.f);
        
        btVector3 localInertia2(0,0,0);
        if (isDynamic2)
            colShape2->calculateLocalInertia(mass2,localInertia2);
        
        
        startTransform2.setOrigin(btVector3(
                                           btScalar(0),
                                           btScalar(0),
                                           btScalar(0)));
        rb2 = createRigidBody(mass2,startTransform2,colShape2);
        
        joint1 = new btHingeConstraint(*rb1,*rb2,
                                       btVector3( btScalar( -1.2 ), btScalar( 0.0 ), btScalar( 0.0 ) ),
                                       btVector3( btScalar( 0.0 ), btScalar( 1.0 ), btScalar( 0.0 ) ),
                                       btVector3( btScalar( 1.0 ), btScalar( 0.0 ), btScalar( 0.0 ) ),
                                       btVector3( btScalar( 0.0 ), btScalar( 0.0 ), btScalar( 1.0 ) ) );
        
        m_dynamicsWorld->addConstraint(joint1, true);

	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


void InvertedPendulumExample::stepSimulation(float deltaTime)
{
    static btScalar maxTorque = 100;
    
    m_frameCount++;
    
    // Getting the current angle on the hinge joint
    btScalar currentAngle = joint1->getHingeAngle();
    
    // The hinge joint doesn't have a method to obtain the angular velocity
    // It is obtained from the difference in angular velocity between the two rigid bodies, one of which is static
    btVector3 currentAngularVelocity = rb2-> getAngularVelocity();
    
    // The obtained angular velocity is a 3D vector, but only the velocity about the axis of rotation is of interest
    btScalar currentAngularVelocityX = currentAngularVelocity[0];
    
    // The angle reading is given from -PI to PI, which isn't optimal, since the values are jumping from PI to -PI
    if(currentAngle < 0) currentAngle += 2*SIMD_PI;
    
    // PD controller is implemented here
    // Proportinally multiply the error in displacement, compared to target value of PI
    // Multiply the derivative of the error in displacement, which corresponds to angular velocity
    btScalar controlTorque = kp * (SIMD_PI-currentAngle) + kd*(currentAngularVelocityX);
    
    // Clamping the control command
    btClamp(controlTorque,-maxTorque,maxTorque);
    
    // Applying the control torque, again only about the axis of rotation matter
    rb2->applyTorque(btVector3(controlTorque, btScalar( 0.0 ), btScalar( 0.0 )));
    
    // Next iteration of the simulation
    m_dynamicsWorld->stepSimulation(1./60.,0);
    
}





CommonExampleInterface*    ET_SimpleBoxCreateFunc(CommonExampleOptions& options)
{
	return new InvertedPendulumExample(options.m_guiHelper);
}



