#include <iostream>

#include "../src/server.h"
#include "../src/object.hpp"


int main(int argc, char *argv[])
{
    int port{4401};
    connector::server< connector::UDP > receiver( port );
    receiver.init();

    while(true)
    {
        object_t my_obj;
        receiver.receive_udp< object_t >( my_obj );
        std::cout << my_obj.distance << std::endl;
        //free(&my_obj);
    }
}
