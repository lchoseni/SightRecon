#include <iostream>

using namespace std;

class A {
   public:
    int a = 2;
};

class B {
   public:
    int a = 1;
};

class C : public A,
          B {
};

int main() {
    C a;
    cout << a.A::a << endl;
}