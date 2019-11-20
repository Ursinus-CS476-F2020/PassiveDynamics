/**
 * Build a class on top of the scene canvas that animates
 * world objects by changing their transformation matrix
 */

vec3 = glMatrix.vec3;
mat4 = glMatrix.mat4;
quat = glMatrix.quat;

function Particles() {
    // Step 1: Initialize scene for rendering
    this.scene = {"children":[],
                "cameras":[
                {
                    "pos": [0.00, 1.50, 5.00],
                    "rot": [0.00, 0.00, 0.00, 1.00],
                    "fovy": 1.0
                }],
                "lights":[
                    {
                        "pos":[0, 20, 0],
                        "color":[1, 1, 1]
                    }
                ],
                "materials":{
                    "redambient":{
                        "ka":[0.7, 0.0, 0.0],
                        "kd":[1, 1, 1]
                    }
                }          
    };

    // Step 2: Initialize physics engine
    // Collision configuration contains default setup for memory, collisions setup
    let collisionConfig = new Ammo.btDefaultCollisionConfiguration(); 
    // Use the default collision dispatcher.  For parallel processing you can use
    // a different dispatcher (see Extras/BulletMultiThread)
    let dispatcher = new Ammo.btCollisionDispatcher(collisionConfig);
    // btDbvtBroadphase is a good general purpose broadphase.  You can also try out
    // btAxis3Sweep
    let overlappingPairCache = new Ammo.btDbvtBroadphase();
    // The default constraint solver.  For parallel processing you can use a different
    // solver (see Extras/BulletMultiThreaded)
    let solver = new Ammo.btSequentialImpulseConstraintSolver();
    let dynamicsWorld = new Ammo.btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfig);
    dynamicsWorld.setGravity(new Ammo.btVector3(0, -10, 0));
    this.dynamicsWorld = dynamicsWorld;


    this.randomlyInitSpheres = function(N) {
        for (let i = 0; i < N; i++) {
            let pos = [Math.random()*10-5, Math.random()*10, Math.random()*10-5];
            let radius = 0.5*Math.random();
            let velocity = [Math.random()*0.1, Math.random()*0.1, Math.random()*0.1];
            const mass = Math.random();
            const restitution = Math.random(); // How bouncy the sphere is (between 0 and 1)

            // Step 1: Setup scene graph entry for rendering
            let sphere = {
                "transform":[radius, 0, 0, pos[0],
                             0, radius, 0, pos[1], 
                             0, 0, radius, pos[2],
                             0, 0, 0, 1],
                "scale":[radius, radius, radius],
                "pos":pos,
                "radius":radius,
                "velocity":velocity,
                "shapes":[
                    {"type":"sphere",
                    "material":"redambient"}
                ]
            }
            this.scene.children.push(sphere);
            
            // Step 2: Setup ammo.js physics engine entry
            const colShape = new Ammo.btSphereShape(radius);
            const localInertia = new Ammo.btVector3(velocity[0], velocity[1], velocity[2]);
            colShape.calculateLocalInertia(mass, localInertia);
            // Need to redefine the transformation for the physics engine
            const ptransform = new Ammo.btTransform();
            ptransform.setIdentity();
            ptransform.setOrigin(new Ammo.btVector3(pos[0], pos[1], pos[2]));
            sphere.ptransform = ptransform;
            const motionState = new Ammo.btDefaultMotionState(ptransform);
            const rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, colShape, localInertia);
            // The final rigid body object
            sphere.body = new Ammo.btRigidBody(rbInfo); 
            sphere.body.setRestitution(restitution);
            // Finally, add the rigid body to the simulator
            this.dynamicsWorld.addRigidBody(sphere.body);
        }
    }

    this.time = 0.0;
    this.lastTime = (new Date()).getTime();

    /**
     * Step forward in time in the physics simulation.
     * Then, for each rigid body object in the scene, read out
     * the current position and orientation from the physics
     * engine, and send it over to the rendering engine (ggslac)
     * via a transformation matrix
     */
    this.animate = function() {
        let thisTime = (new Date()).getTime();
        let dt = (thisTime - this.lastTime)/1000.0; // Change in time in seconds
        this.time += dt;
        this.lastTime = thisTime;
        this.dynamicsWorld.stepSimulation(dt, 10);
        for (shape of this.scene.children) {
            //Scale, rotate, and translate the shape appropriately (in that order)
            let trans = shape.ptransform;
            shape.body.getMotionState().getWorldTransform(trans);
            let x = trans.getOrigin().x();
            let y = trans.getOrigin().y();
            let z = trans.getOrigin().z();
            let q = trans.getRotation();
            // Translation matrix
            let TR = mat4.create();
            mat4.translate(TR, TR, [x, y, z]);
            // Rotation matrix
            let quatMat = mat4.create();
            mat4.fromQuat(quatMat, [q.x(), q.y(), q.z(), q.w()]);
            mat4.multiply(TR, TR, quatMat);
            // Scale matrix
            let SMat = mat4.create();
            mat4.identity(SMat);
            mat4.scale(SMat, SMat, shape.scale);
            mat4.multiply(TR, TR, SMat);
            shape.transform = TR;
        }
    }
}
