//Vector Library [2D]
//CSCI 5611 Vector 2 Library [Incomplete]

//Instructions: Implement all of the following vector operations--

public class Vec2 {
  public float x, y;
  
  //Construct a new vector (ie set x and y)
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ ", " + y +")";
  }
  
  //Return the length of this vector
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  //Return the square of the length of this vector
  public float lengthSqr(){
    return x*x+y*y;
  }
  
  //Return a new vector with the value of this vector plus the rhs vector
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  //Add the rhs vector to this vector
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  //Return a new vector with the value of this vector minus the rhs vector
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  //Subtract the vector rhs from this vector
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  //Return a new vector that is this vector scaled by rhs
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  //Scale this vector by rhs
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  //Rescale this vector to have a unit length
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  //Return a new vector in the same direction as this one, but with unit length
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  //If the vector is longer than maxL, shrink it to be maxL otherwise do nothing
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  //Grow or shrink the vector have a length of maxL
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  //Interepret the two vectors (this and the rhs) as points. How far away are they?
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
  
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}
