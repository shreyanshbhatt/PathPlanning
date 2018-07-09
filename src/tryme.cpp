#include<iostream>

struct trial {
  int a;
  int b;
};

int main() {
  enum p:int {x, y, z};
  trial t;
  t.a = 10;
  t.b = 15;

  trial t1;
  t1 = t;

  std::cout << t.a << " " << t1.a << " " << t.b << " " <<t1.b << std::endl;
  p aa = x;
  std::cout << aa << std::endl;
  return 1;
}
