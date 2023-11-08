struct Vector {
  float x; // X is altitude
  float y;
  float z;

  Vector(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }
};

struct Vector add(struct Vector a, struct Vector b) {
  return Vector(a.x + b.x, a.y + b.y, a.z + b.z);
}
struct Vector sub(struct Vector a, struct Vector b) {
  return Vector(a.x - b.x, a.y - b.y, a.z - b.z);
}
struct Vector mul(struct Vector a, float b) {
  return Vector(a.x * b, a.y * b, a.z * b);
}
struct Vector div(struct Vector a, float b) {
  return mul(a, 1/b);
}