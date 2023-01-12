#include <iostream>
#include "basler.h"
#include "image_chunk.h"

int main()
{
    int run_ret = get_chunk();

    if (run_ret == 0)
    {
        std::cout << "Done!" << std::endl;
    }
    else
    {
        std::cout << "Something went wrong... " << run_ret << std::endl;
    }

    return run_ret;
}