//Simulation Parameters
Camera camera;
PVector gravity = new PVector(0,-9.8,0);
float radius = 5;
float stringTopx = 200;
float stringTopy = 50;
float restLen = 4;
float mass = 10;
float p_const = 0.95;
float k = 4000;
float kv = 50;
boolean fall = false;
float floor = 400;

//Initial positions and velocities of masses
static int maxNodes = 100;
PVector pos[][] = new PVector[maxNodes][maxNodes];
PVector vel[][] = new PVector[maxNodes][maxNodes];
int rowNodes = 20; //y
int colNodes = 30; //x

PVector SpherePos = new PVector(283.0, 129.0, 21);
float SphereRadi = 30;


void setup() {
  size(1000, 800, P3D);
  camera = new Camera();
  blendMode(SUBTRACT);
  sphereDetail(10);
  surface.setTitle("Cloth Simulation");
  for (int i=0; i < colNodes; i++) { // y
    for (int j = 0; j < rowNodes; j++) { // x
      pos[i][j] = new PVector(0,0,0);
      pos[i][j].x = stringTopx + 4*i;
      pos[i][j].y = stringTopy + 4*j;
      pos[i][j].z = stringTopy + 3*j;
      vel[i][j] = new PVector(0,1,0);
    }
  }
}

void update(float dt){
  PVector new_vel[][] = vel;
    
  // horizontial cloth connections
  for (int i = 0; i < rowNodes; i++){
    for (int j = 0; j < colNodes-1; j++){
      PVector diff = PVector.sub(pos[j+1][i], (pos[j][i]));;
      float diff_len = diff.mag();
      diff.normalize();
      float vel1 = diff.dot(vel[j][i]);
      float vel2 = diff.dot(vel[j+1][i]);
      float springF = k*(diff_len-restLen) -kv*(vel1-vel2);
      new_vel[j][i].add(PVector.mult(diff, springF * dt));
      new_vel[j+1][i].sub(PVector.mult(diff, springF * dt));
    }
  }
  
  // vertical cloth connections
  for (int i = 0; i < rowNodes-1; i++){
    for (int j = 0; j < colNodes; j++){
      PVector diff = PVector.sub(pos[j][i+1], (pos[j][i]));;
      float diff_len = diff.mag();
      diff.normalize();
      float vel1 = diff.dot(vel[j][i]);
      float vel2 = diff.dot(vel[j][i+1]);
      float springF = k*(diff_len-restLen) -kv*(vel1-vel2);
      new_vel[j][i].add(PVector.mult(diff, springF * dt));
      new_vel[j][i+1].sub(PVector.mult(diff, springF * dt)); 
    }
  }
  
  // update position and velocity
  for (int i = 0; i < rowNodes; i++){ //y
    for (int j = 0; j < colNodes; j++){ //x
      new_vel[j][i].sub(gravity);
      
      if (i == 0) {
        vel[j][i] = new_vel[j][i].limit(2);
        vel[j][i].x = vel[j][i].x * 2;
        vel[j][i].z = vel[j][i].z * 2;
      }
      else {
        vel[j][i] = new_vel[j][i].limit(2);
        vel[j][i].x = vel[j][i].x * 2;
        vel[j][i].z = vel[j][i].z * 2;
        pos[j][i].add(vel[j][i].mult(dt));
      }
      
      
      if (pos[j][i].y >= floor) {
        pos[j][i].y = floor-0.1;
      }
  
      // collision detection TODO: add in continuous collision detec
      float dist = SpherePos.dist(pos[j][i]);
      if (dist < SphereRadi + 0.05) {
        PVector n = PVector.sub(SpherePos, pos[j][i]);
        n = n.mult(-1);
        n.normalize();
        PVector rebound = PVector.mult(n, (vel[j][i].dot(n)));
        vel[j][i].sub(rebound.mult(1.5));
        pos[j][i].add(n.mult(0.1 + SphereRadi-dist));
      }
      
      //// continuous collision detection - does not work
      ////Step 1: Compute V - a normalized velocity vector
      //float v_len = SpherePos.sub(pos[j][i]).mag(); //Save the distance of vel vector
      //PVector v = vel[j][i].normalize();
      
      ////Step 2: Compute W - a displacement vector pointing from the start of the line segment to the 
      ////center of the circle
      //PVector toCircle = SpherePos.sub(pos[j][i]);
      
      ////Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
      //float a = 1;  //Lenght of l_dir (we noramlized it)
      //float b = -2*PVector.dot(v,toCircle); //-2*dot(l_dir,toCircle)
      //float c = toCircle.magSq() - (SphereRadi+4)*(SphereRadi+4); //different of squared distances
      //float d = b*b - 4*a*c; //discriminant 
      
      //boolean colliding = false;
      //float t = 9999999999.0;
      //if (d >=0 ){ 
      //  //If d is positive we know the line is colliding, but we need to check if the collision
      //  //line within the line segment
      //  //  ... this means t will be between 0 and the length of the line segment
      //  float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only take the first collision [is this safe?]
      //  //float t2 = (-b + sqrt(d))/(2*a);
        
      //  if (t1 > 0 && t1 < v_len && t1 < t){
      //    colliding = true;
      //    t = t1;
      //  } 
      //  //else if (t2 > 0 && t2 < v_len && t2 < t){
      //  //  colliding = true;
      //  //  t = t2;
      //  //}
      //  else {
      //    t = 1.0;
      //  }
      //}
      
      //pos[j][i] = SpherePos.sub(pos[j][i]).normalize().mult(t).add(pos[j][i]);
    }
  }
  
  //// add in a drag force - shows no difference
  //for (int i = 0; i < rowNodes-1; i++){ // y //<>//
  //  for (int j = 0; j < colNodes-1; j++){ //x //<>//
  //    PVector r1 = pos[j][i];
  //    PVector r2 = pos[j+1][i+1];
  //    PVector r3 = pos[j+1][i];
  //    PVector n1 = (PVector.sub(r2,r1)).cross(PVector.sub(r3,r1));
  //    float f_aero1 = -0.5 * p_const * mass * ( vel[j][i].dot(vel[j][i]) * vel[j][i].dot(n1) / (2*n1.dot(n1)) );
  //    n1.mult(f_aero1/3); // and apply to each particle at r1,r2,r3...
  //    pos[j][i].add(n1);
  //    pos[j+1][i+1].add(n1);
  //    pos[j+1][i].add(n1);
      
  //    // trianlge 2
  //    PVector r4 = pos[j][i];
  //    PVector r5 = pos[j][i+1];
  //    PVector r6 = pos[j+1][i+1];
  //    PVector n2 = (PVector.sub(r5,r4)).cross(PVector.sub(r6,r4));
  //    float f_aero2 = -0.5 * p_const * mass * ( vel[j][i].dot(vel[j][i]) * vel[j][i].dot(n2) / (2*n2.dot(n2)) );
  //    n2.mult(f_aero2/3); // and apply to each particle at r4,r5,r6...
  //    pos[j][i].add(n2);
  //    pos[j][i+1].add(n2);
  //    pos[j+1][i+1].add(n2);
  //  }
  //}
  
}

boolean paused = true;
void draw() {
  background(255,255,255);
  camera.Update(1/frameRate);
  directionalLight(255.0, 255.0, 255.0, -1, 1, -1);
  
  if (!paused) { 
    for(int i =0; i < frameRate; i++){
        update(1/frameRate);
      }
  }
  
  fill(225,50,0);
  noStroke();
  pushMatrix();
  lights();
  translate(SpherePos.x, SpherePos.y, SpherePos.z);
  sphere(SphereRadi);
  popMatrix();
  
  fill(20,150,255);
  noStroke();
  for (int i = 0; i < colNodes; i++){ // x
    // draw lines in cloth
    if (i % 10 == 5 || i % 10 == 6 || i % 10 == 4){
      fill(0,150,0); }
    else{ fill(20,150,255); }
    // sphere below static top points
    for (int j = 1; j < rowNodes; j++){ // y
      pushMatrix();
      translate(pos[i][j].x,pos[i][j].y, pos[i][j].z);
      sphere(radius);
      popMatrix();
    }
  } //<>//
  if (paused){
    surface.setTitle("[PAUSED]"); }
  else{
    surface.setTitle(nf(frameRate,0,2) + "FPS"); }
}

void keyPressed(){
  if (key == ' '){
    paused = !paused; //<>//
  }
  else{ //<>//
    camera.HandleKeyPressed();
  }
  if (key == 'r') {
  for (int i=0; i < colNodes; i++) { // y
    for (int j = 0; j < rowNodes; j++) { // x
      pos[i][j] = new PVector(0,0,0);
      pos[i][j].x = stringTopx + 4*i;
      pos[i][j].y = stringTopy + 4*j;
      pos[i][j].z = stringTopy + 4*j;
      vel[i][j] = new PVector(0,1,0);
    }
  }
    SpherePos = new PVector(283.0, 129.0, 21);
  }
  if (key == 'f') {
    if (fall) {
      fall =false;
    }
    else{
      fall = true;
    }
  }
}
void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  SpherePos.z = SpherePos.z + e;
}
void keyReleased()
{
  camera.HandleKeyReleased();
}
void mousePressed(){
  SpherePos = new PVector(mouseX, mouseY, SpherePos.z);
}
void mouseDragged(){
  SpherePos = new PVector(mouseX, mouseY, SpherePos.z);
}
