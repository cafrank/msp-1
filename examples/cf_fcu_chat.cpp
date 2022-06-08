#include <FlightController.hpp>
#include <iostream>
#include <cmath>



#define PI 3.14159265

const int MIN_THROTTLE = 194;
const int MAX_THROTTLE = 1794;
const int HALF_RANGE = (MAX_THROTTLE - MIN_THROTTLE)/2;


struct MyIdent : public msp::Message {
    MyIdent(msp::FirmwareVariant v) : Message(v) {}

    virtual msp::ID id() const override { return msp::ID::MSP_IDENT; }

    msp::ByteVector raw_data;

    virtual bool decode(const msp::ByteVector &data) override {
        raw_data = data;
        return true;
    }
};

struct Callbacks {
    void onIdent(const MyIdent &ident) {
        std::cout << "Raw Ident data: ";
        for(auto d : ident.raw_data) {
            std::cout << int(d) << " ";
        }
        std::cout << std::endl;
    }

    void onRc      (const msp::msg::Rc& rc)		{ std::cout << rc; }
    void onAttitude(const msp::msg::Attitude& attitude) { std::cout << attitude; }
    void onAltitude(const msp::msg::Altitude& altitude) { std::cout << altitude; }
    void onRawGPS  (const msp::msg::RawGPS&   rawGps)	{ std::cout << rawGps; }
    void onCompGPS (const msp::msg::CompGPS&  compGps)	{ std::cout << compGps; }
};

int main(int argc, char *argv[]) {
    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;
    std::vector<uint16_t> motors   = {1000, 1000, 1000, 1000, 1000};
    std::vector<uint16_t> channels(16, 1000);


    Callbacks cbs;
    fcu::FlightController fcu;
    fcu.connect(device, baudrate, 0, true);

    // subscribe with costum type
    fcu.subscribe(&Callbacks::onIdent, &cbs, 60);
    fcu.subscribe(&Callbacks::onRc,    &cbs, 15);
    fcu.subscribe(&Callbacks::onAttitude, &cbs, 1);
    fcu.subscribe(&Callbacks::onAltitude, &cbs, 1);
    fcu.subscribe(&Callbacks::onRawGPS,   &cbs, 1);
    fcu.subscribe(&Callbacks::onCompGPS,  &cbs, 1);


    // spin motors 1 to 4
    fcu.setMotors({1100,1100,1100,1100,0,0,0,0});
    std::this_thread::sleep_for(std::chrono::seconds(1));
    fcu.setMotors({1000,1000,1000,1000,1000,1000,1000,1000});

    for(int i=0; i<1000000; i++)
    {
        for(int j=0; j<16; j++)
        {
	    motors  [j%6] = i + 1200 + j*30;
	    channels[j] = MIN_THROTTLE + HALF_RANGE * (1 + sin ((i+j) *PI/18.0));
        }
        fcu.setRc(channels);
        // fcu.setMotors({motors[0],motors[1], motors[2],1000,1000,1000});
        // fcu.setRPYT({1.0 * motors[0], 1.0 * motors[1], 1.0 * motors[2], 1.0 * motors[3]});
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }

    // Ctrl+C to quit
    std::cin.get();
}
