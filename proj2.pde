// CSCI 5611 Project 2
// Zihe Fan, fanxx478

import peasy.*;
PeasyCam cam;

float sizex = 180;
float sizey = 180;
PImage umn;
float bigR = 60;
Vec3 bigSphere = new Vec3(120, 150, 100);
float moveSpeed = 3;

int numNodesx = 40;
int numNodesy = 40;

//Simulation Parameters
float floor = 600;
Vec3 gravity = new Vec3(0,500,0);
float radius = 5;
float restLen = sizex/numNodesx;
float mass = 0.15;
float k = 1000; 
float kv = 2; 
float mu = 0.8;

String windowTitle = "Cloth";
void setup() {
  size(800, 600, P3D);
  cam = new PeasyCam(this, 600);
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(800);
  surface.setTitle(windowTitle);
  initScene();
  umn = loadImage("umn.png");
}

//Initial positions and velocities of masses
static int maxNodesy = 100;
static int maxNodesx = 100;
Vec3 pos[][] = new Vec3[maxNodesx][maxNodesy];
Vec3 vel[][] = new Vec3[maxNodesx][maxNodesy];
Vec3 acc[][] = new Vec3[maxNodesx][maxNodesy];

void initScene(){
  for (int i = 0; i < numNodesx; i++){
    for (int j = 0; j < numNodesy; j++){
      pos[i][j] = new Vec3(j*(restLen),0,i*(restLen));
      vel[i][j] = new Vec3(0, 0, 0);
    }
  }
}

void calacc(){
  //Reset accelerations each timestep (momenum only applies to velocity)
  for (int i = 0; i < numNodesx; i++){
    for (int j = 0; j < numNodesy; j++){
      acc[i][j] = new Vec3(0,0,0);
      acc[i][j].add(gravity);
    }
  }
  
  //Horiz Hooke's law for each spring
  for (int i = 0; i < numNodesx-1; i++){
    for (int j = 0; j < numNodesy; j++){
      Vec3 diff = pos[i+1][j].minus(pos[i][j]);
      float stringF = -k*(diff.length() - restLen);
      //println(stringF,diff.length(),restLen);
      
      Vec3 stringDir = diff.normalized();
      float projVbot = dot(vel[i][j], stringDir);
      float projVtop = dot(vel[i+1][j], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      
      Vec3 force = stringDir.times(stringF+dampF);
      acc[i][j].add(force.times(-1.0/mass));
      acc[i+1][j].add(force.times(1.0/mass));
      acc[i][j].subtract(vel[i][j].times(mu));
    }
  }
  
  // Vertical Hooke's law for each spring
  for (int i = 0; i < numNodesx; i++){
    for (int j = 0; j < numNodesy-1; j++){
      Vec3 diff = pos[i][j+1].minus(pos[i][j]);
      float stringF = -k*(diff.length() - restLen);
      //println(stringF,diff.length(),restLen);
      
      Vec3 stringDir = diff.normalized();
      float projVbot = dot(vel[i][j], stringDir);
      float projVtop = dot(vel[i][j+1], stringDir);
      float dampF = -kv*(projVtop - projVbot);
      
      Vec3 force = stringDir.times(stringF+dampF);
      acc[i][j].add(force.times(-1.0/mass));
      acc[i][j+1].add(force.times(1.0/mass));
      acc[i][j].subtract(vel[i][j].times(mu));
    }
  }
}

void checkCollesion(){
  //Collision detection and response
  for (int i = 0; i < numNodesx; i++){
    for (int j = 0; j < numNodesy; j++){
      // collision to big Sphere
      float d = bigSphere.distanceTo(pos[i][j]);
      if (d < bigR + 0.2){
        Vec3 n = (bigSphere.minus(pos[i][j])).times(-1);
        n.normalize();
        Vec3 bounce = n.times(dot(n, vel[i][j]));
        vel[i][j].subtract(bounce.times(1.5));
        pos[i][j].add(n.times(bigR+0.5-d));
      }
      // collision to floor
      if (pos[i][j].y+radius > floor){
        vel[i][j].y *= -.2;
        pos[i][j].y = floor - radius;
      }
      
    }
  }
}

void update(float dt){

  calacc();

  //Eulerian integration
  for (int i = 0; i < numNodesx; i++){
    for (int j = 0; j < numNodesy; j++){
      vel[i][j].add(acc[i][j].times(dt));
      pos[i][j].add(vel[i][j].times(dt));
    }
  }
  
  checkCollesion();
  
  // fix top nodes
  for (int i = 0; i<numNodesx; i++){
    //if (i%5 == 0){
      vel[i][0] = new Vec3(0,0,0);
      pos[i][0] = new Vec3(0,0,i*restLen);
    //}
  }
  
}

void updateOb(){
  if (key == 'a'){
    bigSphere.x -= moveSpeed;
  }
  else if (key == 'd'){
    bigSphere.x += moveSpeed;
  }
  if (key == 'w'){
    bigSphere.y -= moveSpeed;
  }
  else if (key == 's'){
    bigSphere.y += moveSpeed;
  }
}

//Draw the scene: one sphere per mass, one line connecting each pair
void draw() {
  background(255,255,255);
  //update(1/(frameRate));
  if (!paused) {
    if (move) {
      updateOb();
    }
    for (int i = 0; i < 20; i++) {
      update(1/(20*frameRate));
    }
  }
  
  if (reseted){
    reseted = false;
    initScene();
  }
  
  drawBigSphere();
  drawCloth();
  
  if (paused)
    surface.setTitle(windowTitle + " [PAUSED]");
  else
    surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

void drawCloth(){
  for (int i = 0; i < numNodesx-1; i++){
    for (int j = 0; j < numNodesy-1; j++){
      float u1 = umn.width / numNodesx * i;
      float v2 = umn.height / numNodesy * (j+1);
      float u2 = umn.width / numNodesx * (i+1);
      float v1 = umn.height / numNodesy * j;
      beginShape();
      texture(umn);
      vertex(pos[i][j].x, pos[i][j].y, pos[i][j].z, u1, v1);
      vertex(pos[i+1][j].x, pos[i+1][j].y, pos[i+1][j].z, u2, v1);
      vertex(pos[i+1][j+1].x, pos[i+1][j+1].y, pos[i+1][j+1].z, u2, v2);
      vertex(pos[i][j+1].x, pos[i][j+1].y, pos[i][j+1].z, u1, v2);
      endShape();
    }
  }
}

void drawBigSphere(){
  pushMatrix();
  noStroke();
  fill(237,189,101);
  translate(bigSphere.x, bigSphere.y, bigSphere.z);
  sphere(bigR);
  fill(255);
  popMatrix();
}

boolean paused = true;
boolean reseted = false;
boolean move = false;
void keyPressed(){
  if (key == ' '){
    paused = !paused;
  }
  if (key == 'r' || key == 'R') {
    reseted = true;
  }
  if (key == 'a' || key == 'w' || key == 's' || key == 'd') {
    move = true;
  }
}

void keyReleased(){
  if (key == 'a' || key == 'w' || key == 's' || key == 'd') {
    move = false;
  }
}

boolean camaraMove;
void mousePressed() {
  if (mouseButton == LEFT) {
    camaraMove = true;
  }
}
void mouseReleased() {
  if (mouseButton == LEFT) {
    camaraMove = false;
  }
}
