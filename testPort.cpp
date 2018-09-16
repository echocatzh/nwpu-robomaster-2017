#include "SerialInterface.h"

int main()
{
    SerialInterface port;
    std::vector<float> rst;
    sleep(1);
    // port.StartThread();
    // port.setGimbal(0.0,10.0);
    //port.setGimbal(0.0,15.0);
        // port.setGimbal(30.0,20.0);
        while(1)
        {
            port.setGimbal(0.0,0.0);
			Delay(50);
            // port.setGimbal(0.0,10.0);
            rst = port.readGimbal();
            std::cout << rst[0] << std::endl; 
        }
        // for(int i = 0; i < 300; i++)
        //while(1)
        // {
            // rst=port.readGimbal();
            // Delay(10);
            // std::cout << rst[0] << ' ' << rst[1] << std::endl;
        // }
    /*for(int i = 0; i < 45; i++)
    {
        port.setGimbal(1.0,0.5);
        rst=port.readGimbal();
        std::cout << "*********************" << std::endl;
        for(float c : rst)
        {
            std::cout << c << std::endl;
        }
        std::cout << "*********************" << std::endl;
        Delay(20);
    }*/
    return 0;
}
