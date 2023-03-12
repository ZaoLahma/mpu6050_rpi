#include <iostream>

#define UNUSED(arg) (void) arg;

int main(int argc, char **argv)
{
    UNUSED(argc);
    UNUSED(argv);
    std::cout<<"Test application starting\n";
    return 0;
}