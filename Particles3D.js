/**
 * Build a class on top of the scene canvas that animates
 * sphere objects by changing their transformation matrix
 */
class Particles {
    constructor() {
        this.initScene();
        this.time = 0.0;
        this.lastTime = (new Date()).getTime();
    }

    /**
     * Setup a scene with one light and a floor
     * @param {float} floory Y position of the floor
     * @param {float} floorextent Half of a side length of floor square
     */
    initScene(floory, floorextent) {
        if (floory === undefined) {
            floory = 0;
        }
        if (floorextent === undefined) {
            floorextent = 1000;
        }
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
        this.floory = floory;
        this.floorextent = floorextent;
        this.scene.children.push({"shapes":[{
            "type":"polygon",
            "vertices":[[-floorextent, floory, -floorextent],
                        [-floorextent, floory, floorextent],
                        [floorextent, floory, floorextent],
                        [floorextent, floory, -floorextent]]
    
        }]});
        this.spheres = []; // Keep an array of spheres for easy access
    }

    /**
     * Initialize a bunch of spheres, each with a random position, radius, mass
     * and initial velocity
     * @param {int} N Number of spheres
     */
    randomlyInitSpheres(N) {
        for (let i = 0; i < N; i++) {
            let pos = [Math.random()*10-5, Math.random()*10, Math.random()*10-5];
            let radius = 0.5*Math.random();
            // Make the mass proportional to the radius
            let mass = (4/3)*Math.PI*Math.pow(radius, 3); 
            let sphere = {
                "transform":[radius, 0, 0, pos[0],
                             0, radius, 0, pos[1], 
                             0, 0, radius, pos[2],
                             0, 0, 0, 1],
                "pos":pos,
                "radius":radius,
                "velocity":[Math.random()*0.1, Math.random()*0.1, Math.random()*0.1],
                "mass":mass,
                "shapes":[
                    {"type":"sphere",
                    "material":"redambient"}
                ]
            }
            this.scene.children.push(sphere); // Adds to scene tree
            this.spheres.push(sphere); // Another array to store the spheres
        }
    }

    /**
     * Do one loop of the animation
     */
    animate() {
        let thisTime = (new Date()).getTime();
        let dt = (thisTime - this.lastTime)/1000.0; // Change in time in seconds
        this.time += dt;
        this.lastTime = thisTime;
        let spheres = this.spheres;
        for (let i = 0; i < spheres.length; i++) {
            for (let k = 0; k < 3; k++) {
                // Updating physics
                spheres[i].pos[k] += dt*spheres[i].velocity[k];
                // Updating the scene
                spheres[i].transform[12+k] = spheres[i].pos[k];
            }
            // TODO: Add acceleration and collision effects
            spheres[i].velocity[1] += -9.81*dt;
            if (spheres[i].pos[1] - spheres[i].radius < 0) {
                spheres[i].velocity[1] = 0.9*Math.abs(spheres[i].velocity[1]);
            }
        }
    }
}
