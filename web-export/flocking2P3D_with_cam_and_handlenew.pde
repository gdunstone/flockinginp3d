// Code from http://processing.org/learning/topics/flocking.html
// by Daniel Shiffman
//
// Other code from Arcc.cc



Flock flock;

void setup() {
  size(700, 700, P3D);
  flock = new Flock();
  //add initial
  for (int i=0; i<400; i++) {
    //flock.addBoid(new Boid(random(0,100)),random(0,100));
    flock.addBoid(new Boid(random(700), random(700), random(700)));
  }
}

float valA = 1;
float valB = 1;
float cModA=1;
float cModB=1; 


boolean started = false;

void draw() {
  frameRate(200);
  CheckCamera();
  //if (started==true){
    flock.run();
    
 // }

  //pushMatrix();
  translate(350,350,350);
  
  //noFill();
  
  stroke(255);
  strokeWeight(10);
  fill(0,10); box(800);
  fill(255,255); box(10000);
  
  translate(0,0,0);
 // popMatrix();

  noStroke();
  cModA = valA*0.3;
  cModB = valB*0.3;
  Angle = Angle+0.3;
}


//boid class

class Boid {
  PVector location;
  PVector velocity;
  PVector acceleration;
  float colour;
  float diameter;
  float r;
  float maxforce; //maximum steering force
  float maxspeed; //max speed

    Boid(float x, float y, float z) {
    acceleration = new PVector(0, 0, 0);
    velocity = new PVector(random(-1, 1), random(-1, 1), random(-1, 1));
    location = new PVector(x, y, z);
    r = 5.0;
    maxspeed = 6;
    maxforce = 0.03;
  }

  //run the system
  void run(ArrayList<Boid> boids) {
    flock(boids);
    update();
    borders();
    render();
  }

  void applyForce(PVector force) {
    acceleration.add(force);
  }

  //we accumulate a new acceleration each time base on 3 rules

  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids); //separation
    PVector ali = align(boids); //alignment
    PVector coh = cohesion(boids); //cohesion
    //arbitrarily weight these forces
    sep.mult(1.5);
    ali.mult(1.0);
    coh.mult(1.0);
    //add the force vectors to accel
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
  }

  void update() {
    //update velocity
    velocity.add(acceleration);
    //limit speed
    velocity.limit(maxspeed);
    location.add(velocity);
    //reset acceleration to 0 each cycle
    acceleration.mult(0);
  }

  //a method that calculates and applies a steering force towards a target
  // steer = desired minus velocity
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, location); //a vector pointing from the location to the target
    //normalie desired and scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);
    //steering = desired minus velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce); //limit to the maximum steering force
    return steer;
  }

  void render() {
    //smooth();
    noStroke();
    //fill (colour*((cModA+cModB)/2), colour*cModA, colour*cModB, location.z/2.9);
    fill (location.x/2.9,location.y/2.9,location.z/2.9,120);
    pushMatrix();
    translate(location.x, location.y, location.z);
    sphere((diameter*-0.03));
    //sphere(diameter*-0.4);
    popMatrix();
  }

  //wraparound
  void borders() {
    if (location.x > 700) {
      velocity.x = random(-1, -2);
    } 
    else if (location.x < 0) {
      velocity.x = random(1, 2);
    } // X

    if (location.y > 700) {
      velocity.y = random(-1, -2);
    } 
    else if (location.y < 0) {
      velocity.y = random(1, 2);
    }// Y
    if (location.z > 700){
      velocity.z = random(-1, -2);
    }
    else if (location.z < 200) {
      velocity.z = random(1, 2);
    }// Z
  }
  
  //Separation
  //method checks for nearby boids and steers away
  PVector separate(ArrayList<Boid> boids) {
    float desiredseparation = 25.0f;
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    //for every boid in the system chech if its too close
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      //if the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      diameter = -d+400;
      colour = d-200;
      if ((d > 0) && (d < desiredseparation)) {
        //calculate vector pointing away from neighbor

        PVector diff = PVector.sub(location, other.location);
        diff.normalize();
        diff.div(d); //weight by distance.
        steer.add(diff);
        count++; //keep track of how many times separation occurs
      }
    }
    //Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    //as long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement reynolds: steering = Desired - velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  //Alignment
  //For every nearby boid in the system, calculate the average velocity
  PVector align (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d>0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxforce);
      return steer;
    } 
    else {
      return new PVector(0, 0);
    }
  }

  //cohesion
  // for the average location (ie center) of all nearby boids, calculate steering vetor towards that location
  PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0); //start with empty vector to accumulate all locations
    int count = 0;
    for (Boid other : boids) {
      float d= PVector.dist(location, other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.location); //add location
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum); //steer towards the location
    } 
    else {
      return new PVector(0, 0);
    }
  }
}



// The FLOCK, a list of Boid objects

class Flock {
  ArrayList<Boid> boids; //an arraylist for the boids to live in

  Flock() {
    boids = new ArrayList<Boid>(); //initialize the arraylist
  }
  void run() {
    for (Boid b : boids) {
      b.run(boids); //Passing the entire list of boids to each boid individually
    }
  }
  void addBoid(Boid b) {
    boids.add(b);
  }
}



// Kamerasteuerung:


// Lamppost 
int ZentraleStange_XValue = 350; // nach rechts
int ZentraleStange_YValue = 350; // nach unten 
int ZentraleStange_ZValue = 350; // nach hinten 

 
final int CheckCamera_Type_SinusLevel = 0;    // rotates
final int CheckCamera_Type_SinusSpirale = 1;  // Spirale
final int CheckCamera_Type_SinusMouse = 2;    // Mouse
final int CheckCamera_Type_SinusPath = 3;     // Path
 
float Angle = 0.0; // Angle bei Kreis 
float Height = 70;    // Höhe der Kamera über der Szene
int HeightAdd = 5;   // wird bei Spirale zu der Höhe der Kamera addiert
float Radius = 1200.0;  // Anfangsradius des Kreises
 
// Art des Kameraverhaltens
int CheckCamera_Type = CheckCamera_Type_SinusSpirale;
 
// Camera
int LookAtX = 350;
int LookAtY = 350;
int LookAtZ = 350;
 
// Camera with Path Behaviour (CheckCamera_Type = 3)
boolean boolGoesOnAPath = true;
float PathValue;
float PathValueMaximal = 95.0;
 
// ==========================================================
 
void CheckCamera () {
  // moves Camera depending from CheckCamera_Type
  switch (CheckCamera_Type) {
  case CheckCamera_Type_SinusLevel:
    // im Falle von 0
    CheckCameraSinusLevel ();
    break;
  case CheckCamera_Type_SinusSpirale:
    // im Falle von 1
    CheckCameraSinusSpirale ();
    break; 
  case 2:
    // im Falle von 2
    CheckCameraMouse ();
    break;     
  case 3:
    // im Falle von 3
    CheckCameraPath ();
    break;
  default :
    // sonst
    println ("Error 133");
    exit();
    break;
  }
}
 
// ---------------------------------------------------------
 
void CheckCameraSinusLevel () {
  // Rotates in "Height"
  camera (
  Radius*sin (radians(Angle)) + ZentraleStange_XValue, Height, Radius* cos (radians(Angle)) + ZentraleStange_ZValue,
  ZentraleStange_XValue, ZentraleStange_YValue, ZentraleStange_ZValue,
  0.0, 1.0, 0.0);
}
 
void CheckCameraSinusSpirale () {
  // Rotates and goes up and down = Spirale
  camera (
  Radius*sin (radians(Angle)) + ZentraleStange_XValue, Height, Radius* cos (radians(Angle)) + ZentraleStange_ZValue,
  ZentraleStange_XValue, ZentraleStange_YValue, ZentraleStange_ZValue,
  0.0, 1.0, 0.0);  
 
  // toggle HeightAdd to switch from going up to going down and vice versa
  if ((Height>height+400) || (Height<-0)) {
    HeightAdd = HeightAdd * -1;
  }
  // change Height
  Height = Height + HeightAdd;
}
 
void CheckCameraMouse () {
  // Mouse
  // Der Befehl map ändert die erste Variable gemäß zweier Valueebereiche. Siehe Hilfe.
  float Angle=0.0;
 
  print (mouseX);
  Angle = map(mouseX, 0, 700, 0, 360);
  println ("->" + Angle);
 
  Height = map(mouseY, 0, height, 0, height+400);
 
  camera (
  Radius*sin(radians(Angle)) + ZentraleStange_XValue, Height, Radius*cos(radians(Angle)) + ZentraleStange_ZValue,
  LookAtX, LookAtY, LookAtZ,
  0.0, 1.0, 0.0
    );
}
 
void CheckCameraPath () {
  // Path
  int x1 = 15;
  int y1 = 10;
  int z1 = 40;
  // lerp bestimmt die Flugbahn von Parameter 1 zu Parameter 2;
  // Parameter 3 gibt dabei an, wie weit der Flug schon ist.
  float x = lerp(x1, ZentraleStange_XValue-10, PathValue/100.0) ;
  float y = lerp(y1, ZentraleStange_YValue-40, PathValue/100.0);
  float z = lerp(z1, ZentraleStange_ZValue+10, PathValue/100.0);
 
  println(PathValue);
 
  // PathValueMaximal reached
  // restart Path or stop Path
  if (PathValue>PathValueMaximal) {
    // boolGoesOnAPath=false; // stop Path
    PathValue= 0.0;           // restart Path
  }
 
  camera (
  x, y, z,
  ZentraleStange_XValue, ZentraleStange_YValue, ZentraleStange_ZValue-(1223/2),
  0.0, 1.0, 0.0);
}


// Tab for Inputs 

void keyPressed() {

  // Keyboard

    if (key == CODED) {

    // Spezialtasten wie z.B. Cursortasten: 

    if (keyCode == UP ) { // Hoch
      Height-=10;
    } 
    else if (keyCode == DOWN ) { // Runter 
      Height+=10;
    } 
    else if (keyCode == RIGHT ) { // Rechts 
      Radius++;
    } 
    else if (keyCode == LEFT ) { // Links 
      Radius--;
      // Wenn zu klein, dann bei -2 einfrieren 
      if (Radius<-2) {
        Radius=-2;
      }
    }     
    else {
      //
    }
  } 
  else {   
    // not key == CODED 
    // normale Tasten wie 0 oder 1 
    if (key == '0') { 
      CheckCamera_Type = CheckCamera_Type_SinusLevel;
      print("Camera circles at constant height");
    }    
    else if (key == '1') { 
      CheckCamera_Type = CheckCamera_Type_SinusSpirale;
      print("Camera circles in a spiral up and down");
    }        
    else if (key == '2') { 
      CheckCamera_Type = CheckCamera_Type_SinusMouse;
      print("Camera moves according to mouse");
    }        
    else if (key == '3') { 
      // Path init
      print("Camera moves on a path");
      boolGoesOnAPath = true; 
      PathValue = -40.0; 
      CheckCamera_Type = CheckCamera_Type_SinusPath;
    }        
    else if (key == 'w') { 
      // Path init
      LookAtY--;
    }           
    else if (key == 's') { 
      // Path init
      LookAtY++;
    }            
    else if (key == 'a') { 
      // Path init
      LookAtX++;
    }           
    else if (key == 'd') { 
      // Path init
      LookAtX--;
    }            
    else if (key == 'q') { 
      // Path init
      LookAtZ++;
    }           
    else if (key == 'e') { 
      // Path init
      LookAtZ--;
    }            

    else {
      // do nothing
    }
  }
}


