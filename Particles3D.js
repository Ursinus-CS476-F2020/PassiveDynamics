/**
 * Build a class on top of the scene canvas that animates
 * sphere objects by changing their transformation matrix
 */

let vec3 = glMatrix.vec3;

function Particles() {
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
    this.floory = 0.0;
    this.floorextent = 1000;
    this.scene.children.push({"shapes":[{
        "type":"polygon",
        "vertices":[[-this.floorextent, this.floory, -this.floorextent],
                    [-this.floorextent, this.floory, this.floorextent],
                    [this.floorextent, this.floory, this.floorextent],
                    [this.floorextent, this.floory, -this.floorextent]]

    }]});
    this.spheres = [];

    this.randomlyInitSpheres = function(N) {
        for (let i = 0; i < N; i++) {
            let pos = [Math.random()*10-5, Math.random()*10, Math.random()*10-5];
            let radius = 0.5*Math.random();
            let sphere = {
                "transform":[radius, 0, 0, pos[0],
                             0, radius, 0, pos[1], 
                             0, 0, radius, pos[2],
                             0, 0, 0, 1],
                "pos":pos,
                "radius":radius,
                "gravity":-Math.random()*5,
                "velocity":[Math.random()*0.1, Math.random()*0.1, Math.random()*0.1],
                "shapes":[
                    {"type":"sphere",
                    "material":"redambient"}
                ]
            }
            this.scene.children.push(sphere);
            this.spheres.push(sphere);
        }
    }

    this.time = 0.0;
    this.lastTime = (new Date()).getTime();
    this.animate = function() {
        let thisTime = (new Date()).getTime();
        let dt = (thisTime - this.lastTime)/1000.0; // Change in time in seconds
        this.time += dt;
        this.lastTime = thisTime;
        let spheres = this.spheres;
        for (let i = 0; i < spheres.length; i++) {
            for (let j = i+1; j < spheres.length; j++) {
                let dCenter = vec3.create();
                vec3.subtract(dCenter, spheres[i].pos, spheres[j].pos);
                if (vec3.length(dCenter) < spheres[i].radius + spheres[j].radius) {
                    spheres[i].velocity = [0, 0, 0];
                    spheres[j].velocity = [0, 0, 0];
                }
            }
        }
        for (let i = 0; i < spheres.length; i++) {
            for (let k = 0; k < 3; k++) {
                spheres[i].pos[k] += dt*spheres[i].velocity[k];
                spheres[i].transform[12+k] = spheres[i].pos[k];
            }
            spheres[i].velocity[1] -= 9.81*dt;
            const r = spheres[i].radius;
            if (spheres[i].pos[1] < this.floory + r) {
                // Balls hit the floor
                spheres[i].velocity[1] *= -1;
                spheres[i].pos[1] = this.floory + r + 0.001;
                vec3.scale(spheres[i].velocity, spheres[i].velocity, 0.8);
            }
            // TODO: Add acceleration and collision effects
            
        }
    }
}
